#pragma once

#include <cmath>

namespace homefly
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravity = 9.80665f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

struct Vec3
{
	float x{0.0f};
	float y{0.0f};
	float z{0.0f};
};

inline float clamp(
	float value,
	float lower,
	float upper
) noexcept
{
	return value < lower ? lower : (value > upper ? upper : value);
}

inline bool is_finite(float value) noexcept
{
	return std::isfinite(value);
}

inline bool is_finite(
	const Vec3& value
) noexcept
{
	return is_finite(value.x) && is_finite(value.y) && is_finite(value.z);
}

} // namespace homefly
