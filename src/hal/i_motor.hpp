#pragma once

#include <cstdint>

namespace homefly
{

// Motor output array. Each value is normalized thrust in [0.0f, 1.0f].
// Implementations must output the safe state when disarmed.
class IMotor
{
public:
	virtual ~IMotor() = default;

	virtual bool initialize() = 0;

	virtual bool arm() = 0;

	virtual void disarm() = 0;

	virtual bool write(
		const float* outputs,
		uint8_t count
	) = 0;

	virtual bool is_armed() const = 0;

	virtual uint8_t channel_count() const = 0;
};

} // namespace homefly
