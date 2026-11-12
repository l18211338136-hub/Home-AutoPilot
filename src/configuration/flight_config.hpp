#pragma once

#include <cstdint>

#include "control/math_types.hpp"
#include "control/motor_mixer.hpp"
#include "control/pid_controller.hpp"

namespace homefly
{

// All tunable flight parameters live here. Hardware differences are resolved
// by configuration + platform adapters, never by changing FlightCore.
struct FlightConfig
{
	// Inner loop: 1000 Hz, max 5 ms stale IMU sample, overrun at 3 periods.
	uint32_t control_period_us{1000};
	uint32_t imu_timeout_us{5000};
	uint32_t overrun_factor{3};

	float gyro_weight{0.95f};

	// Cascaded P(attitude) -> PID(rate) gains.
	PidGains angle_roll{4.0f, 0.0f, 0.0f};
	PidGains angle_pitch{4.0f, 0.0f, 0.0f};
	PidGains rate_roll{0.6f, 0.2f, 0.08f};
	PidGains rate_pitch{0.6f, 0.2f, 0.08f};
	PidGains rate_yaw{0.8f, 0.1f, 0.0f};

	float max_rate_command{3.0f};
	float max_angle_command_rad{30.0f * kDegToRad};
	float max_torque_command{1.0f};

	float motor_output_min{0.0f};
	float motor_output_max{1.0f};
	uint8_t motor_count{MotorMixer::kMotorCount};
};

FlightConfig default_quad_x_config() noexcept;

} // namespace homefly
