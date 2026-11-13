#include "drivers/motor/drv_pwm_motor.hpp"

#include <cmath>

namespace homefly
{

PwmMotor::PwmMotor(
	IPwmOutput& pwm,
	uint16_t min_pulse_us,
	uint16_t max_pulse_us
) noexcept
	: _pwm(pwm),
	  _min_pulse_us(min_pulse_us),
	  _max_pulse_us(max_pulse_us)
{
}

bool PwmMotor::initialize()
{
	_armed = false;

	if (!_pwm.initialize())
	{
		return false;
	}

	_pwm.set_all_min();
	return true;
}

bool PwmMotor::arm()
{
	_armed = true;
	return true;
}

void PwmMotor::disarm()
{
	_armed = false;
	_pwm.set_all_min();
}

bool PwmMotor::write(
	const float* outputs,
	uint8_t count
)
{
	if (outputs == nullptr || count > _pwm.channel_count())
	{
		return false;
	}

	// Hardware interlock: a disarmed driver can never emit a running pulse.
	if (!_armed)
	{
		_pwm.set_all_min();
		return true;
	}

	const uint16_t span = static_cast<uint16_t>(_max_pulse_us - _min_pulse_us);

	for (uint8_t channel = 0; channel < count; ++channel)
	{
		float value = outputs[channel];

		// A non-finite thrust command must never reach the ESC. A NaN/Inf slips
		// past the <0 / >1 clamp (both comparisons are false) and would produce
		// an undefined pulse width. Treat it as a driver fault: drop to the safe
		// (disarmed, minimum-pulse) state and stop emitting
		// (HomeFly Flight Safety & Code Review skill 3.9).
		if (!std::isfinite(value))
		{
			disarm();
			return false;
		}

		if (value < 0.0f)
		{
			value = 0.0f;
		}
		else if (value > 1.0f)
		{
			value = 1.0f;
		}

		const uint16_t pulse_us = static_cast<uint16_t>(
			_min_pulse_us + static_cast<uint16_t>(span * value));

		if (!_pwm.write_pulse_us(channel, pulse_us))
		{
			disarm();
			return false;
		}
	}

	return true;
}

bool PwmMotor::is_armed() const
{
	return _armed;
}

uint8_t PwmMotor::channel_count() const
{
	return _pwm.channel_count();
}

} // namespace homefly
