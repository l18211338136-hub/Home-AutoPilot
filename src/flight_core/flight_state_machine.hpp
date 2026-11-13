#pragma once

#include "flight_core/types.hpp"

namespace homefly
{

enum class StateEvent : uint8_t
{
	None = 0,
	ArmRequest,
	DisarmRequest,
	SensorFault,
	MotorFault,
	ControlFault
};

// Explicit transition table. Failsafe is sticky: leaving it requires a
// disarm request AND valid sensors.
class FlightStateMachine
{
public:
	FlightMode update(
		StateEvent event,
		bool sensors_valid,
		bool prearm_ok = true,
		FailsafeReason fault_reason = FailsafeReason::None
	) noexcept;

	FlightMode mode() const noexcept;

	FailsafeReason failsafe_reason() const noexcept;

private:
	FlightMode _mode{FlightMode::Disarmed};
	FailsafeReason _reason{FailsafeReason::None};
};

} // namespace homefly
