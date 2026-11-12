#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>

#include "homefly_test.hpp"
#include "simulation/gazebo/gazebo_hal.hpp"
#include "simulation/gazebo/gazebo_transport.hpp"
#include "simulation/gazebo/udp_socket.hpp"

using namespace homefly;

namespace
{

bool receive_within(
	UdpSocket& socket,
	GazeboFrame& frame,
	uint32_t timeout_ms = 1000u
)
{
	const auto deadline = std::chrono::steady_clock::now() +
		std::chrono::milliseconds(timeout_ms);

	while (std::chrono::steady_clock::now() < deadline)
	{
		uint32_t length = 0;
		const int result = socket.receive(&frame, sizeof(frame), length);

		if (result > 0 && length == sizeof(GazeboFrame))
		{
			return true;
		}

		std::this_thread::sleep_for(std::chrono::microseconds(200));
	}

	return false;
}

GazeboTransportConfig test_config(
	uint16_t local_port,
	uint16_t remote_port
)
{
	GazeboTransportConfig config{};
	config.local_port = local_port;
	config.remote_port = remote_port;
	config.wait_timeout_ms = 1000u;
	std::strcpy(config.host, "127.0.0.1");
	return config;
}

ImuData sample()
{
	ImuData data{};
	data.timestamp_us = 5000u;
	data.accel_z = 9.80665f;
	return data;
}

} // namespace

TEST(GazeboUdp, LoopbackRoundTrip)
{
	UdpSocket a;
	UdpSocket b;
	EXPECT_TRUE(a.open(14650u, "127.0.0.1", 14651u));
	EXPECT_TRUE(b.open(14651u, "127.0.0.1", 14650u));
	EXPECT_TRUE(a.is_open());
	EXPECT_TRUE(b.is_open());

	const GazeboFrame sent = gazebo_frame_make_imu(11u, 1000u, sample());
	EXPECT_TRUE(a.send(&sent, sizeof(sent)));

	GazeboFrame received{};
	EXPECT_TRUE(receive_within(b, received));
	EXPECT_TRUE(gazebo_frame_validate(received));
	EXPECT_EQ(received.seq, 11u);
	EXPECT_EQ(received.timestamp_us, 1000u);

	a.close();
	EXPECT_TRUE(!a.is_open());
	b.close();
}

TEST(GazeboUdp, ClosedSocketCannotSend)
{
	UdpSocket socket;
	GazeboFrame frame{};
	EXPECT_TRUE(!socket.send(&frame, sizeof(frame)));
	uint32_t length = 0;
	EXPECT_TRUE(socket.receive(&frame, sizeof(frame), length) < 0);
}

TEST(GazeboTransport, ReceivesImuAndPublishesMotor)
{
	UdpSocket peer;
	EXPECT_TRUE(peer.open(14661u, "127.0.0.1", 14660u));

	GazeboTransport transport;
	EXPECT_TRUE(transport.start(test_config(14660u, 14661u)));
	EXPECT_TRUE(transport.running());

	const GazeboFrame imu = gazebo_frame_make_imu(1u, 1000u, sample());
	EXPECT_TRUE(peer.send(&imu, sizeof(imu)));

	EXPECT_TRUE(transport.wait_for_imu());
	GazeboFrame received{};
	EXPECT_TRUE(transport.pop_imu(received));
	EXPECT_EQ(received.seq, 1u);
	EXPECT_EQ(transport.pending_imu(), 0u);

	// Duplicate seq must be rejected (the mock bridge retransmits while the
	// flight controller is starting up).
	EXPECT_TRUE(peer.send(&imu, sizeof(imu)));
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	EXPECT_EQ(transport.pending_imu(), 0u);

	const float outputs[4]{0.1f, 0.2f, 0.3f, 0.4f};
	transport.push_motor(gazebo_frame_make_motor(1u, 1000u, outputs, 4u));

	GazeboFrame motor{};
	EXPECT_TRUE(receive_within(peer, motor));
	EXPECT_TRUE(gazebo_frame_validate(motor));
	EXPECT_EQ(motor.type, static_cast<uint8_t>(GazeboFrameType::Motor));
	EXPECT_EQ(motor.seq, 1u);

	// Let the transport task finish its counters before snapshotting them.
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	const GazeboTransportStats stats = transport.stats();
	EXPECT_EQ(stats.imu_received, 1u);
	EXPECT_TRUE(stats.imu_rejected >= 1u);
	EXPECT_EQ(stats.motor_sent, 1u);
	EXPECT_EQ(stats.send_failures, 0u);

	transport.stop();
	EXPECT_TRUE(!transport.running());
	peer.close();
}

TEST(GazeboTransport, WaitTimesOutWithoutFrames)
{
	GazeboTransport transport;
	GazeboTransportConfig config = test_config(14662u, 14663u);
	config.wait_timeout_ms = 30u;
	EXPECT_TRUE(transport.start(config));
	EXPECT_TRUE(!transport.wait_for_imu());
	transport.stop();
}

TEST(GazeboHal, ClockTracksConsumedImuTimestamp)
{
	GazeboClock clock;
	EXPECT_EQ(clock.now_us(), 0u);
	clock.set_served_us(7000u);
	EXPECT_EQ(clock.now_us(), 7000u);
}

TEST(GazeboHal, ImuFaultAndEmptyRing)
{
	GazeboTransport transport;
	GazeboClock clock;
	GazeboImu imu(transport, clock);

	EXPECT_TRUE(imu.initialize());
	ImuData data{};
	EXPECT_TRUE(!imu.read(data)); // no frame queued yet

	imu.set_fault(true);
	EXPECT_TRUE(!imu.read(data));
	imu.set_fault(false);
	EXPECT_TRUE(!imu.read(data));
}

TEST(GazeboHal, MotorInterlockAndZeroOutput)
{
	GazeboTransport transport;
	GazeboClock clock;
	GazeboMotor motor(transport, clock);

	EXPECT_TRUE(motor.initialize());
	EXPECT_EQ(motor.channel_count(), 4u);
	EXPECT_TRUE(!motor.is_armed());
	EXPECT_TRUE(motor.motors_zero());

	const float outputs[4]{0.5f, 0.5f, 0.5f, 0.5f};
	EXPECT_TRUE(!motor.write(outputs, 2u)); // count must be exactly 4
	EXPECT_TRUE(motor.arm());
	EXPECT_TRUE(motor.write(outputs, 4u));
	EXPECT_TRUE(!motor.motors_zero());

	motor.disarm();
	EXPECT_TRUE(motor.write(outputs, 4u)); // disarmed -> publishes zeros
	EXPECT_TRUE(motor.motors_zero());
}

HOMEFLY_TEST_MAIN()
