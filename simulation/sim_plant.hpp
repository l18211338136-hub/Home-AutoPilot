#pragma once

#include <cstdint>

#include "control/attitude_estimator.hpp"
#include "control/math_types.hpp"
#include "control/motor_mixer.hpp"
#include "hal/i_imu.hpp"

namespace homefly
{

// Minimal rigid body attitude plant for closed loop simulation.
// It is deliberately small: no aerodynamics, no altitude, no latency model.
// Its only job is to prove that the core loop can read a sensor, estimate,
// control and drive an output without any hardware.
class SimPlant
{
public:
	void reset() noexcept;

	void set_initial_attitude(
		float roll_rad,
		float pitch_rad
	) noexcept;

	void set_gyro_bias(
		float bias_rad_s
	) noexcept;

	void set_motor_outputs(
		const float* outputs,
		uint8_t count
	) noexcept;

	void step(
		float dt_s
	) noexcept;

	ImuData imu_sample(
		uint64_t timestamp_us
	) const noexcept;

	Attitude attitude() const noexcept;

	Vec3 rate() const noexcept;

	float motor_output(
		uint8_t index
	) const noexcept;

private:
	float _motors[MotorMixer::kMotorCount]{};
	Attitude _attitude{};
	Vec3 _rate{};
	float _gyro_bias{0.0f};
};

} // namespace homefly
