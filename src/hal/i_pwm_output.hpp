#pragma once

#include <cstdint>

namespace homefly
{

// Low level PWM capability. Pulse width in microseconds.
class IPwmOutput
{
public:
	virtual ~IPwmOutput() = default;

	virtual bool initialize() = 0;

	virtual bool write_pulse_us(
		uint8_t channel,
		uint16_t pulse_us
	) = 0;

	virtual void set_all_min() = 0;

	virtual uint8_t channel_count() const = 0;
};

} // namespace homefly
