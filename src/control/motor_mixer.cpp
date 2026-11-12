#include "control/motor_mixer.hpp"

#include "control/math_types.hpp"

namespace homefly
{

MotorMixer::MotorMixer(
	float output_min,
	float output_max
) noexcept
	: _output_min(output_min),
	  _output_max(output_max)
{
}

uint8_t MotorMixer::mix(
	const MixerInput& input,
	float* outputs,
	uint8_t capacity
) const noexcept
{
	if (outputs == nullptr || capacity < kMotorCount)
	{
		return 0;
	}

	const float throttle = clamp(input.throttle, 0.0f, 1.0f);
	const float roll = clamp(input.roll, -1.0f, 1.0f);
	const float pitch = clamp(input.pitch, -1.0f, 1.0f);
	const float yaw = clamp(input.yaw, -1.0f, 1.0f);

	const float raw[kMotorCount] =
	{
		throttle - roll + pitch + yaw,
		throttle + roll + pitch - yaw,
		throttle - roll - pitch - yaw,
		throttle + roll - pitch + yaw
	};

	for (uint8_t i = 0; i < kMotorCount; ++i)
	{
		outputs[i] = clamp(raw[i], _output_min, _output_max);
	}

	return kMotorCount;
}

} // namespace homefly
