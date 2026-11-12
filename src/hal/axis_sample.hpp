#pragma once

#include <cstdint>

namespace homefly
{

// Generic 3-axis sample. Units are defined by each sensor interface.
struct AxisSample
{
	uint64_t timestamp_us{0};
	float x{0.0f};
	float y{0.0f};
	float z{0.0f};
};

} // namespace homefly
