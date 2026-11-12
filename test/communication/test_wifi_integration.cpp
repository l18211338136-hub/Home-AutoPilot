// End-to-end integration of the WiFi uplink with the flight controller.
//
// This drives the REAL production classes end to end, in a single process, with
// a virtual clock (no real-time pacing, no scheduling races, no host timer
// jitter). The uplink path exercised here is exactly the one used by the
// linux_native binary:
//
//     GCS (UdpLink.send) -> loopback UDP -> UdpLink.receive
//        -> CommandReceiver::poll()  (low-priority I/O task)
//        -> CommandReceiver::latest_setpoint() (control loop, no I/O)
//        -> FlightController::step() -> Stabilize
//
// UdpLink lives in the linux_native application target; it is compiled directly
// into this test (see test/CMakeLists.txt) so the uplink transport is exercised
// for real rather than mocked.

#include <cmath>
#include <cstdint>
#include <cstring>

#include "applications/linux_native/udp_link.hpp"
#include "communication/command_receiver.hpp"
#include "communication/control_protocol.hpp"
#include "configuration/flight_config.hpp"
#include "flight_core/flight_controller.hpp"
#include "hal/i_clock.hpp"
#include "homefly_test.hpp"
#include "simulation/sim_clock.hpp"
#include "simulation/sim_imu.hpp"
#include "simulation/sim_motor.hpp"
#include "simulation/sim_plant.hpp"

using namespace homefly;

namespace
{

constexpr uint16_t kFcPortA = 14810;
constexpr uint16_t kGcsPortA = 14811;
constexpr uint16_t kFcPortB = 14820;
constexpr uint16_t kGcsPortB = 14821;

// Mirror the Python GCS struct.pack("<IBBH16sI") wire layout exactly.
void serialize_frame(
	const ControlFrame& frame,
	uint8_t out[kControlFrameSize]
) noexcept
{
	out[0] = static_cast<uint8_t>(frame.magic & 0xFFu);
	out[1] = static_cast<uint8_t>((frame.magic >> 8) & 0xFFu);
	out[2] = static_cast<uint8_t>((frame.magic >> 16) & 0xFFu);
	out[3] = static_cast<uint8_t>((frame.magic >> 24) & 0xFFu);
	out[4] = frame.version;
	out[5] = frame.type;
	out[6] = static_cast<uint8_t>(frame.seq & 0xFFu);
	out[7] = static_cast<uint8_t>((frame.seq >> 8) & 0xFFu);
	std::memcpy(out + 8, frame.payload, kControlPayloadSize);
	out[24] = static_cast<uint8_t>(frame.checksum & 0xFFu);
	out[25] = static_cast<uint8_t>((frame.checksum >> 8) & 0xFFu);
	out[26] = static_cast<uint8_t>((frame.checksum >> 16) & 0xFFu);
	out[27] = static_cast<uint8_t>((frame.checksum >> 24) & 0xFFu);
}

uint16_t g_seq = 0;

void send_frame(
	UdpLink& gcs,
	ControlFrameType type,
	const uint8_t* payload,
	uint8_t payload_len
)
{
	ControlFrame frame{};
	EXPECT_TRUE(build_control_frame(type, payload, payload_len, g_seq++, frame));
	uint8_t buf[kControlFrameSize];
	serialize_frame(frame, buf);
	EXPECT_TRUE(gcs.send(buf, kControlFrameSize));
}

struct Rig
{
	SimClock clock;
	SimPlant plant;
	FlightConfig config;
	SimImu imu;
	SimMotor motor;
	FlightController controller;

	explicit Rig(float initial_roll = 0.2f, float initial_pitch = -0.1f)
		: plant(),
		  config(default_quad_x_config()),
		  imu(plant, clock),
		  motor(plant),
		  controller(imu, motor, clock, config)
	{
		plant.set_initial_attitude(initial_roll, initial_pitch);
		controller.initialize();
	}

	float dt_s() const
	{
		return static_cast<float>(config.control_period_us) * 1.0e-6f;
	}
};

} // namespace

// 1) A WiFi ARM frame followed by periodic SETPOINT frames, delivered over a real
//    UDP socket, drives the controller into Stabilize and the plant converges.
TEST(WifiIntegration, ArmsAndStabilizesOverUdp)
{
	UdpLink fc_link;
	EXPECT_TRUE(fc_link.open(kFcPortA, "127.0.0.1", kFcPortA));
	UdpLink gcs_link;
	EXPECT_TRUE(gcs_link.open(kGcsPortA, "127.0.0.1", kFcPortA));

	// link_timeout disabled: this test verifies the arm/stabilize path only.
	Rig rig;
	CommandReceiver receiver(fc_link, rig.clock, 0u);

	const uint32_t kCycles = 2500;
	uint8_t sp[kControlPayloadSize];

	for (uint32_t i = 0; i < kCycles; ++i)
	{
		// GCS uplink: ARM once, then a SETPOINT every 25 cycles (~40 Hz).
		if (i == 0)
		{
			send_frame(gcs_link, ControlFrameType::Arm, nullptr, 0);
		}
		if (i % 25u == 0u)
		{
			pack_setpoint_payload(0.5f, 0.0f, 0.0f, 0.0f, sp);
			send_frame(gcs_link, ControlFrameType::Setpoint, sp, kControlPayloadSize);
		}

		// Low-priority task: the ONLY place that does socket I/O.
		receiver.poll();

		// Control loop: advances the clock, then steps with the latched setpoint
		// (no I/O). Mirrors the production loop in linux_native/main.cpp.
		rig.clock.advance_us(rig.config.control_period_us);
		rig.controller.step(receiver.latest_setpoint());
		rig.plant.step(rig.dt_s());
	}

	// ARM (1) + SETPOINT at i = 0,25,...,2475 (100) = 101 frames expected.
	EXPECT_TRUE(receiver.frames_received() >= 100u);
	EXPECT_EQ(receiver.parse_errors(), 0u);

	EXPECT_EQ(rig.controller.mode(), FlightMode::Stabilize);
	EXPECT_TRUE(rig.motor.is_armed());
	EXPECT_NEAR(rig.controller.state().attitude.roll, 0.0f, 0.05f);
	EXPECT_NEAR(rig.controller.state().attitude.pitch, 0.0f, 0.05f);
}

// 2) Once armed, if the GCS stops transmitting past the link timeout, the
//    receiver forces a disarm request and the controller drops to Disarmed.
TEST(WifiIntegration, LinkLossDisarmsOverUdp)
{
	UdpLink fc_link;
	EXPECT_TRUE(fc_link.open(kFcPortB, "127.0.0.1", kFcPortB));
	UdpLink gcs_link;
	EXPECT_TRUE(gcs_link.open(kGcsPortB, "127.0.0.1", kFcPortB));

	const uint32_t kTimeoutUs = 400000u; // 400 ms
	Rig rig;
	CommandReceiver receiver(fc_link, rig.clock, kTimeoutUs);

	uint8_t sp[kControlPayloadSize];

	// Phase 1: arm + keep the link alive for 200 cycles.
	for (uint32_t i = 0; i < 200; ++i)
	{
		if (i == 0)
		{
			send_frame(gcs_link, ControlFrameType::Arm, nullptr, 0);
		}
		if (i % 25u == 0u)
		{
			pack_setpoint_payload(0.5f, 0.0f, 0.0f, 0.0f, sp);
			send_frame(gcs_link, ControlFrameType::Setpoint, sp, kControlPayloadSize);
		}
		receiver.poll();
		rig.clock.advance_us(rig.config.control_period_us);
		rig.controller.step(receiver.latest_setpoint());
		rig.plant.step(rig.dt_s());
	}

	EXPECT_EQ(rig.controller.mode(), FlightMode::Stabilize);
	EXPECT_FALSE(receiver.link_lost());

	// Phase 2: GCS goes silent. Advance well past the timeout; poll() returns
	// nothing and latest_setpoint() must now force a disarm.
	for (uint32_t i = 0; i < 600; ++i)
	{
		receiver.poll();
		rig.clock.advance_us(rig.config.control_period_us);
		rig.controller.step(receiver.latest_setpoint());
		rig.plant.step(rig.dt_s());
	}

	EXPECT_TRUE(receiver.link_lost());
	EXPECT_EQ(rig.controller.mode(), FlightMode::Disarmed);
	EXPECT_FALSE(rig.motor.is_armed());
}

HOMEFLY_TEST_MAIN()
