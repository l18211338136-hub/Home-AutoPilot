#include "simulation/gazebo/gazebo_protocol.hpp"

#include <cmath>
#include <cstddef>

namespace homefly
{

namespace
{

constexpr uint32_t kChecksumOffset = offsetof(GazeboFrame, checksum);

bool finite_payload(
	const GazeboFrame& frame
) noexcept
{
	for (const float value : frame.values)
	{
		if (!std::isfinite(value))
		{
			return false;
		}
	}

	return true;
}

} // namespace

uint32_t gazebo_frame_checksum(
	const GazeboFrame& frame
) noexcept
{
	const auto* bytes = reinterpret_cast<const uint8_t*>(&frame);
	uint32_t hash = 2166136261u;

	for (uint32_t i = 0; i < sizeof(GazeboFrame); ++i)
	{
		if (i >= kChecksumOffset && i < kChecksumOffset + sizeof(uint32_t))
		{
			continue;
		}

		hash ^= bytes[i];
		hash *= 16777619u;
	}

	return hash;
}

void gazebo_frame_seal(
	GazeboFrame& frame
) noexcept
{
	frame.checksum = gazebo_frame_checksum(frame);
}

bool gazebo_frame_validate(
	const GazeboFrame& frame
) noexcept
{
	if (frame.magic != kGazeboFrameMagic ||
		frame.version != kGazeboProtocolVersion ||
		frame.tail != 0u ||
		frame.checksum != gazebo_frame_checksum(frame))
	{
		return false;
	}

	if (frame.type < static_cast<uint8_t>(GazeboFrameType::Imu) ||
		frame.type > static_cast<uint8_t>(GazeboFrameType::Shutdown))
	{
		return false;
	}

	return finite_payload(frame);
}

GazeboFrame gazebo_frame_make_imu(
	uint32_t seq,
	uint64_t timestamp_us,
	const ImuData& data
) noexcept
{
	GazeboFrame frame{};
	frame.seq = seq;
	frame.timestamp_us = timestamp_us;
	frame.type = static_cast<uint8_t>(GazeboFrameType::Imu);
	frame.values[0] = data.gyro_x;
	frame.values[1] = data.gyro_y;
	frame.values[2] = data.gyro_z;
	frame.values[3] = data.accel_x;
	frame.values[4] = data.accel_y;
	frame.values[5] = data.accel_z;
	frame.values[6] = data.temperature_c;
	gazebo_frame_seal(frame);
	return frame;
}

GazeboFrame gazebo_frame_make_motor(
	uint32_t seq,
	uint64_t timestamp_us,
	const float* motors,
	uint8_t count
) noexcept
{
	GazeboFrame frame{};
	frame.seq = seq;
	frame.timestamp_us = timestamp_us;
	frame.type = static_cast<uint8_t>(GazeboFrameType::Motor);

	if (motors != nullptr)
	{
		const uint8_t limit = count < kGazeboMotorCount ? count : kGazeboMotorCount;

		for (uint8_t i = 0; i < limit; ++i)
		{
			frame.values[i] = motors[i];
		}
	}

	gazebo_frame_seal(frame);
	return frame;
}

bool gazebo_frame_to_imu(
	const GazeboFrame& frame,
	ImuData& data
) noexcept
{
	if (frame.type != static_cast<uint8_t>(GazeboFrameType::Imu))
	{
		return false;
	}

	data = ImuData{};
	data.timestamp_us = frame.timestamp_us;
	data.gyro_x = frame.values[0];
	data.gyro_y = frame.values[1];
	data.gyro_z = frame.values[2];
	data.accel_x = frame.values[3];
	data.accel_y = frame.values[4];
	data.accel_z = frame.values[5];
	data.temperature_c = frame.values[6];
	return true;
}

bool gazebo_frame_to_motor(
	const GazeboFrame& frame,
	float* motors,
	uint8_t capacity
) noexcept
{
	if (frame.type != static_cast<uint8_t>(GazeboFrameType::Motor) ||
		motors == nullptr || capacity < kGazeboMotorCount)
	{
		return false;
	}

	for (uint8_t i = 0; i < kGazeboMotorCount; ++i)
	{
		motors[i] = frame.values[i];
	}

	return true;
}

} // namespace homefly
