#pragma once

#include <cstdint>

namespace homefly
{

// Normalized control demands. Torques are in [-1, 1], throttle in [0, 1].
struct MixerInput
{
	float throttle{0.0f};
	float roll{0.0f};
	float pitch{0.0f};
	float yaw{0.0f};
};

// Quad-X allocation, motor order:
//   0 front left  (CW)   1 front right (CCW)
//   2 rear left   (CCW)  3 rear right  (CW)
// roll > 0 -> right side down, pitch > 0 -> nose up, yaw > 0 -> nose right.
class MotorMixer
{
public:
	static constexpr uint8_t kMotorCount = 4;

	MotorMixer(
		float output_min,
		float output_max
	) noexcept;

	// Returns the number of channels written, 0 if capacity is insufficient.
	uint8_t mix(
		const MixerInput& input,
		float* outputs,
		uint8_t capacity
	) const noexcept;

private:
	float _output_min;
	float _output_max;
};

} // namespace homefly
