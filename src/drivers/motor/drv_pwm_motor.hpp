#pragma once

#include <cstdint>

#include "hal/i_motor.hpp"
#include "hal/i_pwm_output.hpp"

namespace homefly
{

// Adapts a generic IPwmOutput capability into the IMotor interface:
// normalized thrust -> pulse width in microseconds.
class PwmMotor : public IMotor
{
public:
	PwmMotor(
		IPwmOutput& pwm,
		uint16_t min_pulse_us,
		uint16_t max_pulse_us
	) noexcept;

	bool initialize() override;

	bool arm() override;

	void disarm() override;

	bool write(
		const float* outputs,
		uint8_t count
	) override;

	bool is_armed() const override;

	uint8_t channel_count() const override;

private:
	IPwmOutput& _pwm;
	uint16_t _min_pulse_us;
	uint16_t _max_pulse_us;
	bool _armed{false};
};

} // namespace homefly
