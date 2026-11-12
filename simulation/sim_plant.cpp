#include "simulation/sim_plant.hpp"

#include <cmath>

namespace homefly
{

namespace
{
constexpr float kTorqueGain = 8.0f;
constexpr float kAngularDamping = 2.5f;
} // namespace

void SimPlant::reset() noexcept
{
	for (float& motor : _motors)
	{
		motor = 0.0f;
	}

	_attitude = Attitude{};
	_rate = Vec3{};
	_gyro_bias = 0.0f;
}

void SimPlant::set_initial_attitude(
	float roll_rad,
	float pitch_rad
) noexcept
{
	_attitude.roll = roll_rad;
	_attitude.pitch = pitch_rad;
	_attitude.yaw = 0.0f;
	_rate = Vec3{};
}

void SimPlant::set_gyro_bias(
	float bias_rad_s
) noexcept
{
	_gyro_bias = bias_rad_s;
}

void SimPlant::set_motor_outputs(
	const float* outputs,
	uint8_t count
) noexcept
{
	if (outputs == nullptr)
	{
		return;
	}

	const uint8_t limit = count < MotorMixer::kMotorCount ? count : MotorMixer::kMotorCount;

	for (uint8_t i = 0; i < limit; ++i)
	{
		_motors[i] = clamp(outputs[i], 0.0f, 1.0f);
	}
}

void SimPlant::step(
	float dt_s
) noexcept
{
	const float dt = clamp(dt_s, 1.0e-4f, 0.5f);

	// Differential thrust -> body torques. Signs match MotorMixer quad-X.
	const float roll_torque = (_motors[1] + _motors[3] - _motors[0] - _motors[2]) * 0.25f;
	const float pitch_torque = (_motors[0] + _motors[1] - _motors[2] - _motors[3]) * 0.25f;
	const float yaw_torque = (_motors[0] + _motors[3] - _motors[1] - _motors[2]) * 0.25f;

	_rate.x += (roll_torque * kTorqueGain - _rate.x * kAngularDamping) * dt;
	_rate.y += (pitch_torque * kTorqueGain - _rate.y * kAngularDamping) * dt;
	_rate.z += (yaw_torque * kTorqueGain - _rate.z * kAngularDamping) * dt;

	_attitude.roll += _rate.x * dt;
	_attitude.pitch += _rate.y * dt;
	_attitude.yaw += _rate.z * dt;
}

ImuData SimPlant::imu_sample(
	uint64_t timestamp_us
) const noexcept
{
	ImuData data{};
	data.timestamp_us = timestamp_us;
	data.gyro_x = _rate.x + _gyro_bias;
	data.gyro_y = _rate.y;
	data.gyro_z = _rate.z;
	data.accel_x = -kGravity * std::sin(_attitude.pitch);
	data.accel_y = kGravity * std::sin(_attitude.roll);
	data.accel_z = kGravity * std::cos(_attitude.roll) * std::cos(_attitude.pitch);
	data.temperature_c = 25.0f;
	return data;
}

Attitude SimPlant::attitude() const noexcept
{
	return _attitude;
}

Vec3 SimPlant::rate() const noexcept
{
	return _rate;
}

float SimPlant::motor_output(
	uint8_t index
) const noexcept
{
	return index < MotorMixer::kMotorCount ? _motors[index] : 0.0f;
}

} // namespace homefly
