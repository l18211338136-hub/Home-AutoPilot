#include "flight_core/prearm_checker.hpp"

#include <cmath>

namespace homefly
{

bool PreArmReport::all_passed() const noexcept
{
	for (std::size_t i = 0; i < static_cast<std::size_t>(PreArmCheck::Count); ++i)
	{
		if (!passed[i])
		{
			return false;
		}
	}
	return true;
}

PreArmCheck PreArmReport::first_failure() const noexcept
{
	for (std::size_t i = 0; i < static_cast<std::size_t>(PreArmCheck::Count); ++i)
	{
		if (!passed[i])
		{
			return static_cast<PreArmCheck>(i);
		}
	}
	return PreArmCheck::Count;
}

namespace
{

bool motor_config_valid(
	const FlightConfig& c
) noexcept
{
	if (!(c.motor_output_min >= 0.0f && c.motor_output_min <= 1.0f))
	{
		return false;
	}
	if (!(c.motor_output_max >= 0.0f && c.motor_output_max <= 1.0f))
	{
		return false;
	}
	if (!(c.motor_output_min < c.motor_output_max))
	{
		return false;
	}
	if (c.motor_count == 0 || c.motor_count > MotorMixer::kMotorCount)
	{
		return false;
	}
	return true;
}

bool flight_params_valid(
	const FlightConfig& c
) noexcept
{
	if (c.control_period_us == 0)
	{
		return false;
	}
	if (c.imu_timeout_us == 0)
	{
		return false;
	}
	if (!(c.gyro_weight > 0.0f && c.gyro_weight <= 1.0f))
	{
		return false;
	}
	if (!std::isfinite(c.max_rate_command) || c.max_rate_command <= 0.0f)
	{
		return false;
	}
	if (!std::isfinite(c.max_angle_command_rad) || c.max_angle_command_rad <= 0.0f)
	{
		return false;
	}
	if (!std::isfinite(c.max_torque_command) || c.max_torque_command <= 0.0f)
	{
		return false;
	}

	const PidGains* gains[] = {
		&c.angle_roll, &c.angle_pitch,
		&c.rate_roll, &c.rate_pitch, &c.rate_yaw
	};
	for (const PidGains* g : gains)
	{
		if (!std::isfinite(g->kp) || !std::isfinite(g->ki) || !std::isfinite(g->kd))
		{
			return false;
		}
		if (g->kp < 0.0f || g->ki < 0.0f || g->kd < 0.0f)
		{
			return false;
		}
	}
	return true;
}

} // namespace

PreArmReport PreArmChecker::check(
	const PreArmInputs& in
) const noexcept
{
	PreArmReport r{};

	const bool config_ok = in.config != nullptr;

	r.passed[static_cast<std::size_t>(PreArmCheck::ImuInitialized)] = in.imu_initialized;
	r.passed[static_cast<std::size_t>(PreArmCheck::ImuDataValid)] = in.imu_data_valid;
	r.passed[static_cast<std::size_t>(PreArmCheck::ThrottleAtMinimum)] =
		std::isfinite(in.throttle) && in.throttle <= kThrottleArmMax;
	r.passed[static_cast<std::size_t>(PreArmCheck::MotorConfigValid)] =
		config_ok && motor_config_valid(*in.config);
	r.passed[static_cast<std::size_t>(PreArmCheck::FlightParamsValid)] =
		config_ok && flight_params_valid(*in.config);
	r.passed[static_cast<std::size_t>(PreArmCheck::NoSevereFault)] = !in.has_severe_fault;
	r.passed[static_cast<std::size_t>(PreArmCheck::CalibrationOk)] =
		!in.calibration_required || in.calibration_ok;
	r.passed[static_cast<std::size_t>(PreArmCheck::BatteryInRange)] =
		!in.has_battery ||
		(std::isfinite(in.battery_voltage) &&
		 in.battery_voltage >= in.battery_min_v &&
		 in.battery_voltage <= in.battery_max_v);

	return r;
}

} // namespace homefly
