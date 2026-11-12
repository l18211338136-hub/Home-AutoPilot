#include "flight_core/safety_monitor.hpp"

#include <cmath>

#include "control/math_types.hpp"

namespace homefly
{

SafetyMonitor::SafetyMonitor(
	uint32_t imu_timeout_us,
	uint32_t overrun_factor
) noexcept
	: _imu_timeout_us(imu_timeout_us),
	  _overrun_factor(overrun_factor == 0 ? 1 : overrun_factor)
{
}

FailsafeReason SafetyMonitor::validate_imu(
	const ImuData& data,
	uint64_t now_us
) const noexcept
{
	const Vec3 gyro{data.gyro_x, data.gyro_y, data.gyro_z};
	const Vec3 accel{data.accel_x, data.accel_y, data.accel_z};

	if (!is_finite(gyro) || !is_finite(accel) || !is_finite(data.temperature_c))
	{
		return FailsafeReason::InvalidImuData;
	}

	const float accel_norm = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

	if (accel_norm < 0.5f * kGravity || accel_norm > 2.0f * kGravity)
	{
		return FailsafeReason::InvalidImuData;
	}

	// A timestamp from the future means clock skew; treat it as stale data.
	if (data.timestamp_us > now_us || now_us - data.timestamp_us > _imu_timeout_us)
	{
		return FailsafeReason::ImuTimeout;
	}

	return FailsafeReason::None;
}

FailsafeReason SafetyMonitor::check_timing(
	uint64_t now_us,
	uint64_t previous_cycle_us,
	uint32_t period_us
) const noexcept
{
	if (previous_cycle_us == 0 || now_us <= previous_cycle_us)
	{
		return FailsafeReason::None;
	}

	const uint64_t limit = static_cast<uint64_t>(period_us) * _overrun_factor;

	if (now_us - previous_cycle_us > limit)
	{
		return FailsafeReason::ControlOverrun;
	}

	return FailsafeReason::None;
}

} // namespace homefly
