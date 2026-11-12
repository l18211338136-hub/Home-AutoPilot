#pragma once

#include <cstdint>

#include "flight_core/types.hpp"
#include "hal/i_imu.hpp"

namespace homefly
{

// Pure checks, no state, no I/O. Called once per control cycle.
class SafetyMonitor
{
public:
	SafetyMonitor(
		uint32_t imu_timeout_us,
		uint32_t overrun_factor
	) noexcept;

	FailsafeReason validate_imu(
		const ImuData& data,
		uint64_t now_us
	) const noexcept;

	FailsafeReason check_timing(
		uint64_t now_us,
		uint64_t previous_cycle_us,
		uint32_t period_us
	) const noexcept;

private:
	uint32_t _imu_timeout_us;
	uint32_t _overrun_factor;
};

} // namespace homefly
