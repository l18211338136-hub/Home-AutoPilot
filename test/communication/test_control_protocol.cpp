#include "communication/control_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "homefly_test.hpp"

using namespace homefly;

namespace
{

constexpr float kEps = 1e-6f;

} // namespace

TEST(ControlProtocol, BuildAndParseArmRoundTrip)
{
	ControlFrame frame{};
	EXPECT_TRUE(build_control_frame(ControlFrameType::Arm, nullptr, 0, 7, frame));

	EXPECT_EQ(frame.magic, kControlFrameMagic);
	EXPECT_EQ(frame.version, kControlFrameVersion);
	EXPECT_EQ(frame.type, static_cast<uint8_t>(ControlFrameType::Arm));
	EXPECT_EQ(frame.seq, 7u);

	// Re-parse the exact raw bytes we just built.
	uint8_t raw[kControlFrameSize];
	std::memcpy(raw, &frame, kControlFrameSize);

	ControlFrame parsed{};
	EXPECT_TRUE(parse_control_frame(raw, kControlFrameSize, parsed));
	EXPECT_EQ(parsed.type, static_cast<uint8_t>(ControlFrameType::Arm));
	EXPECT_EQ(parsed.seq, 7u);
}

TEST(ControlProtocol, RawLayoutMatchesWireFormat)
{
	// Locks the byte layout so the Python GCS stays compatible.
	uint8_t payload[kControlPayloadSize]{};
	pack_setpoint_payload(0.5f, 0.1f, -0.2f, 0.3f, payload);

	ControlFrame frame{};
	EXPECT_TRUE(build_control_frame(ControlFrameType::Setpoint, payload, kControlPayloadSize, 1, frame));

	uint8_t raw[kControlFrameSize];
	std::memcpy(raw, &frame, kControlFrameSize);

	// magic (LE) at [0,4)
	EXPECT_EQ(raw[0], 0x43u);
	EXPECT_EQ(raw[1], 0x4Cu);
	EXPECT_EQ(raw[2], 0x46u);
	EXPECT_EQ(raw[3], 0x48u);
	// version at [4], type at [5] (=3 Setpoint)
	EXPECT_EQ(raw[4], 1u);
	EXPECT_EQ(raw[5], static_cast<uint8_t>(ControlFrameType::Setpoint));
	// payload floats little-endian at [8,24)
	float throttle = 0.0f;
	std::memcpy(&throttle, raw + 8, sizeof(float));
	EXPECT_NEAR(throttle, 0.5f, kEps);

	// checksum at [24,28) must equal a recomputation over [0,24).
	const uint32_t recomputed = control_frame_checksum(raw, 24u);
	uint32_t stored = 0u;
	std::memcpy(&stored, raw + 24, sizeof(uint32_t));
	EXPECT_EQ(stored, recomputed);
}

TEST(ControlProtocol, RejectsWrongMagicSizeAndChecksum)
{
	ControlFrame frame{};
	EXPECT_TRUE(build_control_frame(ControlFrameType::Ping, nullptr, 0, 0, frame));
	uint8_t raw[kControlFrameSize];
	std::memcpy(raw, &frame, kControlFrameSize);

	// wrong size
	ControlFrame out{};
	EXPECT_FALSE(parse_control_frame(raw, kControlFrameSize - 1, out));

	// corrupted payload -> checksum mismatch
	uint8_t broken[kControlFrameSize];
	std::memcpy(broken, raw, kControlFrameSize);
	broken[10] ^= 0xFF;
	EXPECT_FALSE(parse_control_frame(broken, kControlFrameSize, out));

	// unknown frame type
	uint8_t unknown[kControlFrameSize];
	std::memcpy(unknown, raw, kControlFrameSize);
	unknown[5] = 99;
	// recompute checksum so only the type is "unknown"
	uint32_t cs = control_frame_checksum(unknown, 24u);
	std::memcpy(unknown + 24, &cs, sizeof(uint32_t));
	EXPECT_FALSE(parse_control_frame(unknown, kControlFrameSize, out));

	// valid still parses
	EXPECT_TRUE(parse_control_frame(raw, kControlFrameSize, out));
}

TEST(ControlProtocol, SetpointPayloadRoundTrip)
{
	float t = 0.0f, r = 0.0f, p = 0.0f, y = 0.0f;
	uint8_t payload[kControlPayloadSize]{};
	pack_setpoint_payload(0.75f, 0.05f, -0.05f, 1.25f, payload);
	unpack_setpoint_payload(payload, t, r, p, y);
	EXPECT_NEAR(t, 0.75f, kEps);
	EXPECT_NEAR(r, 0.05f, kEps);
	EXPECT_NEAR(p, -0.05f, kEps);
	EXPECT_NEAR(y, 1.25f, kEps);
}

TEST(ControlProtocol, BuildRejectsOversizedPayload)
{
	ControlFrame frame{};
	uint8_t big[20]{};
	EXPECT_FALSE(build_control_frame(ControlFrameType::Setpoint, big, 20, 0, frame));
	EXPECT_TRUE(build_control_frame(ControlFrameType::Setpoint, big, 16, 0, frame));
}

HOMEFLY_TEST_MAIN()
