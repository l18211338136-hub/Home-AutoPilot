#include "flight_core/flight_state_machine.hpp"

namespace homefly
{

FlightMode FlightStateMachine::update(
	StateEvent event,
	bool sensors_valid,
	FailsafeReason fault_reason
) noexcept
{
	switch (event)
	{
	case StateEvent::SensorFault:
	case StateEvent::MotorFault:
	case StateEvent::ControlFault:
		_mode = FlightMode::Failsafe;
		_reason = fault_reason;
		break;

	case StateEvent::DisarmRequest:
		if (_mode != FlightMode::Failsafe || sensors_valid)
		{
			_mode = FlightMode::Disarmed;
			_reason = FailsafeReason::None;
		}
		break;

	case StateEvent::ArmRequest:
		if (_mode == FlightMode::Disarmed)
		{
			if (sensors_valid)
			{
				_mode = FlightMode::Stabilize;
				_reason = FailsafeReason::None;
			}
			else
			{
				_mode = FlightMode::Failsafe;
				_reason = FailsafeReason::ImuTimeout;
			}
		}
		break;

	case StateEvent::None:
	default:
		break;
	}

	// Defense in depth: a sensor fault during flight must never leave the
	// controller in Stabilize, even if the caller forgot the event.
	if (!sensors_valid && _mode == FlightMode::Stabilize)
	{
		_mode = FlightMode::Failsafe;
		_reason = FailsafeReason::ImuTimeout;
	}

	return _mode;
}

FlightMode FlightStateMachine::mode() const noexcept
{
	return _mode;
}

FailsafeReason FlightStateMachine::failsafe_reason() const noexcept
{
	return _reason;
}

} // namespace homefly
