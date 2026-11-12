#pragma once

#include "hal/i_imu.hpp"

namespace homefly
{

// Euler angles in radians, body frame, z up:
//   roll  > 0 : right side down
//   pitch > 0 : nose up
//   yaw   > 0 : nose right
struct Attitude
{
	float roll{0.0f};
	float pitch{0.0f};
	float yaw{0.0f};
};

// Complementary filter: gyro integration for high frequency, accelerometer
// gravity vector for low frequency. Yaw is gyro-only in phase 1 (no mag fuse).
// No allocation, no blocking, bounded execution time.
class AttitudeEstimator
{
public:
	explicit AttitudeEstimator(
		float gyro_weight = 0.98f
	) noexcept;

	void reset() noexcept;

	void set_gyro_weight(
		float gyro_weight
	) noexcept;

	// Returns false when the accelerometer sample is not usable.
	bool update(
		const ImuData& data,
		float dt_s
	) noexcept;

	const Attitude& attitude() const noexcept;

	bool valid() const noexcept;

private:
	float _gyro_weight;
	Attitude _attitude{};
	bool _valid{false};
	bool _aligned{false};
};

} // namespace homefly
