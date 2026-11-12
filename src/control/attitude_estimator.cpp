#include "control/attitude_estimator.hpp"

#include <cmath>

#include "control/math_types.hpp"

namespace homefly
{

AttitudeEstimator::AttitudeEstimator(
	float gyro_weight
) noexcept
	: _gyro_weight(clamp(gyro_weight, 0.0f, 1.0f))
{
}

void AttitudeEstimator::reset() noexcept
{
	_attitude = Attitude{};
	_valid = false;
	_aligned = false;
}

void AttitudeEstimator::set_gyro_weight(
	float gyro_weight
) noexcept
{
	_gyro_weight = clamp(gyro_weight, 0.0f, 1.0f);
}

bool AttitudeEstimator::update(
	const ImuData& data,
	float dt_s
) noexcept
{
	const Vec3 accel{data.accel_x, data.accel_y, data.accel_z};
	const Vec3 gyro{data.gyro_x, data.gyro_y, data.gyro_z};

	if (!is_finite(accel) || !is_finite(gyro))
	{
		_valid = false;
		return false;
	}

	const float accel_norm = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

	// A saturated, free-falling or inverted accelerometer must not be fused.
	if (accel_norm < 0.5f * kGravity || accel_norm > 2.0f * kGravity)
	{
		_valid = false;
		return false;
	}

	const float dt = clamp(dt_s, 1.0e-4f, 0.5f);
	const float accel_roll = std::atan2(accel.y, accel.z);
	const float accel_pitch = std::atan2(-accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));

	// Align to the gravity vector on the first valid sample. Without this the
	// filter would need ~50 ms to converge from zero and the controller would
	// command a large transient on startup.
	if (!_aligned)
	{
		_attitude.roll = accel_roll;
		_attitude.pitch = accel_pitch;
		_attitude.yaw = gyro.z * dt;
		_aligned = true;
		_valid = true;
		return true;
	}

	// Small angle body-to-Euler approximation. Valid for the attitude envelope
	// of a stabilized micro quad; a quaternion EKF replaces this in phase 2.
	const float gyro_roll = _attitude.roll + gyro.x * dt;
	const float gyro_pitch = _attitude.pitch + gyro.y * dt;

	_attitude.roll = _gyro_weight * gyro_roll + (1.0f - _gyro_weight) * accel_roll;
	_attitude.pitch = _gyro_weight * gyro_pitch + (1.0f - _gyro_weight) * accel_pitch;
	_attitude.yaw += gyro.z * dt;

	if (_attitude.yaw > kPi)
	{
		_attitude.yaw -= 2.0f * kPi;
	}
	else if (_attitude.yaw < -kPi)
	{
		_attitude.yaw += 2.0f * kPi;
	}

	_valid = true;
	return true;
}

const Attitude& AttitudeEstimator::attitude() const noexcept
{
	return _attitude;
}

bool AttitudeEstimator::valid() const noexcept
{
	return _valid;
}

} // namespace homefly
