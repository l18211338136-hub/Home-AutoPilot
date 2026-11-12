#include "control/pid_controller.hpp"

#include "control/math_types.hpp"

namespace homefly
{

PidController::PidController(
	const PidGains& gains,
	float output_min,
	float output_max,
	float integral_max
) noexcept
	: _gains(gains),
	  _output_min(output_min),
	  _output_max(output_max),
	  _integral_max(integral_max)
{
}

void PidController::reset() noexcept
{
	_integral = 0.0f;
	_prev_measurement = 0.0f;
	_last_output = 0.0f;
	_has_prev = false;
}

void PidController::set_gains(
	const PidGains& gains
) noexcept
{
	_gains = gains;
}

float PidController::update(
	float setpoint,
	float measurement,
	float dt_s
) noexcept
{
	// Guard against a stalled or non monotonic clock: never divide by zero
	// and never let one long gap produce an unbounded control output.
	const float dt = clamp(dt_s, 1.0e-4f, 0.5f);
	const float error = setpoint - measurement;

	_integral = clamp(_integral + error * dt, -_integral_max, _integral_max);

	float derivative = 0.0f;

	if (_has_prev)
	{
		derivative = -(measurement - _prev_measurement) / dt;
	}

	_prev_measurement = measurement;
	_has_prev = true;

	const float raw = _gains.kp * error + _gains.ki * _integral + _gains.kd * derivative;
	_last_output = clamp(raw, _output_min, _output_max);
	return _last_output;
}

float PidController::last_output() const noexcept
{
	return _last_output;
}

float PidController::integral() const noexcept
{
	return _integral;
}

} // namespace homefly
