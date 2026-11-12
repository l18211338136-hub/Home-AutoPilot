#pragma once

#include <cstdint>

namespace homefly
{

// Body frame, right-handed, z up.
// Gyroscope: rad/s. Accelerometer: m/s^2 (specific force).
struct ImuData
{
	uint64_t timestamp_us{0};
	float gyro_x{0.0f};
	float gyro_y{0.0f};
	float gyro_z{0.0f};
	float accel_x{0.0f};
	float accel_y{0.0f};
	float accel_z{0.0f};
	float temperature_c{0.0f};
};

class IImu
{
public:
	virtual ~IImu() = default;

	virtual bool initialize() = 0;

	virtual bool read(
		ImuData& data
	) = 0;
};

} // namespace homefly
