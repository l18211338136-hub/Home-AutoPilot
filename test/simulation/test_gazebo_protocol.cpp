#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include "homefly_test.hpp"
#include "simulation/gazebo/gazebo_protocol.hpp"
#include "simulation/gazebo/spsc_ring.hpp"

using namespace homefly;

namespace
{

// Golden vector produced by tools/gazebo_bridge.py:
//   make_imu(7, 123456, (1,-2,3), (4,5,-6), 25.0)
// Keeping it here proves the C++ and Python wire formats agree byte for byte,
// which is the single most important cross-language contract in SITL.
const uint8_t kGoldenFrame[64]{
	0x47, 0x5A, 0x46, 0x48, 0x07, 0x00, 0x00, 0x00, 0x40, 0xE2, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x9E, 0x90, 0xF8, 0x5B, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0xC0,
	0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0xA0, 0x40, 0x00, 0x00, 0xC0, 0xC0,
	0x00, 0x00, 0xC8, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

ImuData golden_sample()
{
	ImuData data{};
	data.timestamp_us = 123456;
	data.gyro_x = 1.0f;
	data.gyro_y = -2.0f;
	data.gyro_z = 3.0f;
	data.accel_x = 4.0f;
	data.accel_y = 5.0f;
	data.accel_z = -6.0f;
	data.temperature_c = 25.0f;
	return data;
}

} // namespace

TEST(GazeboProtocol, FrameIsExactly64BytesAndNaturallyAligned)
{
	EXPECT_EQ(sizeof(GazeboFrame), static_cast<size_t>(kGazeboFrameSize));
	EXPECT_EQ(alignof(GazeboFrame), alignof(uint64_t));
}

TEST(GazeboProtocol, MatchesPythonGoldenVectorByteForByte)
{
	const GazeboFrame frame = gazebo_frame_make_imu(7u, 123456u, golden_sample());
	EXPECT_EQ(std::memcmp(&frame, kGoldenFrame, sizeof(frame)), 0);
}

TEST(GazeboProtocol, ParsesPythonGoldenVector)
{
	GazeboFrame frame{};
	std::memcpy(&frame, kGoldenFrame, sizeof(frame));
	EXPECT_TRUE(gazebo_frame_validate(frame));

	ImuData data{};
	EXPECT_TRUE(gazebo_frame_to_imu(frame, data));
	EXPECT_EQ(data.timestamp_us, 123456u);
	EXPECT_NEAR(data.gyro_x, 1.0f, 1.0e-6f);
	EXPECT_NEAR(data.gyro_y, -2.0f, 1.0e-6f);
	EXPECT_NEAR(data.accel_z, -6.0f, 1.0e-6f);
	EXPECT_NEAR(data.temperature_c, 25.0f, 1.0e-6f);
}

TEST(GazeboProtocol, SealThenValidateRoundTrip)
{
	GazeboFrame frame{};
	frame.seq = 42;
	frame.type = static_cast<uint8_t>(GazeboFrameType::Imu);
	frame.values[0] = 0.25f;
	EXPECT_TRUE(!gazebo_frame_validate(frame));
	gazebo_frame_seal(frame);
	EXPECT_TRUE(gazebo_frame_validate(frame));
}

TEST(GazeboProtocol, RejectsTamperedPayload)
{
	GazeboFrame frame = gazebo_frame_make_imu(1u, 1000u, golden_sample());
	frame.values[3] += 1.0f;
	EXPECT_TRUE(!gazebo_frame_validate(frame));
}

TEST(GazeboProtocol, RejectsWrongMagicVersionTypeAndTail)
{
	GazeboFrame frame = gazebo_frame_make_imu(1u, 1000u, golden_sample());

	frame.magic ^= 1u;
	EXPECT_TRUE(!gazebo_frame_validate(frame));

	frame = gazebo_frame_make_imu(1u, 1000u, golden_sample());
	frame.version = 2;
	EXPECT_TRUE(!gazebo_frame_validate(frame));

	frame = gazebo_frame_make_imu(1u, 1000u, golden_sample());
	frame.type = 99;
	EXPECT_TRUE(!gazebo_frame_validate(frame));

	frame = gazebo_frame_make_imu(1u, 1000u, golden_sample());
	frame.tail = 1;
	EXPECT_TRUE(!gazebo_frame_validate(frame));
}

TEST(GazeboProtocol, RejectsNonFiniteValues)
{
	GazeboFrame frame = gazebo_frame_make_imu(1u, 1000u, golden_sample());
	frame.values[1] = std::numeric_limits<float>::quiet_NaN();
	gazebo_frame_seal(frame);
	EXPECT_TRUE(!gazebo_frame_validate(frame));
}

TEST(GazeboProtocol, MotorFrameRoundTrip)
{
	const float outputs[4]{0.1f, 0.2f, 0.3f, 0.4f};
	const GazeboFrame frame = gazebo_frame_make_motor(9u, 2000u, outputs, 4u);
	EXPECT_TRUE(gazebo_frame_validate(frame));

	float decoded[4]{};
	EXPECT_TRUE(gazebo_frame_to_motor(frame, decoded, 4u));
	EXPECT_NEAR(decoded[0], 0.1f, 1.0e-6f);
	EXPECT_NEAR(decoded[3], 0.4f, 1.0e-6f);
	EXPECT_TRUE(!gazebo_frame_to_motor(frame, decoded, 2u));

	ImuData data{};
	EXPECT_TRUE(!gazebo_frame_to_imu(frame, data));
}

TEST(GazeboProtocol, MotorFrameClampsNullToZero)
{
	const GazeboFrame frame = gazebo_frame_make_motor(1u, 0u, nullptr, 0u);
	float decoded[4]{1.0f, 1.0f, 1.0f, 1.0f};
	EXPECT_TRUE(gazebo_frame_to_motor(frame, decoded, 4u));
	EXPECT_NEAR(decoded[0], 0.0f, 1.0e-6f);
	EXPECT_NEAR(decoded[3], 0.0f, 1.0e-6f);
}

TEST(SpscRing, FifoOrderAndPending)
{
	SpscRing<uint32_t, 8> ring;
	EXPECT_EQ(ring.pending(), 0u);

	for (uint32_t i = 0; i < 5u; ++i)
	{
		EXPECT_TRUE(ring.push(i));
	}

	EXPECT_EQ(ring.pending(), 5u);

	for (uint32_t i = 0; i < 5u; ++i)
	{
		uint32_t value = 999u;
		EXPECT_TRUE(ring.pop(value));
		EXPECT_EQ(value, i);
	}

	EXPECT_EQ(ring.pending(), 0u);
	uint32_t unused = 0u;
	EXPECT_TRUE(!ring.pop(unused));
}

TEST(SpscRing, FullRingDropsAndCounts)
{
	SpscRing<uint32_t, 4> ring;

	for (uint32_t i = 0; i < 4u; ++i)
	{
		EXPECT_TRUE(ring.push(i));
	}

	EXPECT_TRUE(!ring.push(99u));
	EXPECT_EQ(ring.dropped(), 1u);
	EXPECT_EQ(ring.pending(), 4u);

	ring.reset();
	EXPECT_EQ(ring.pending(), 0u);
	EXPECT_EQ(ring.dropped(), 0u);
}

HOMEFLY_TEST_MAIN()
