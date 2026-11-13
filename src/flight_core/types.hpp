#pragma once

#include <cstdint>

#include "control/attitude_estimator.hpp"
#include "control/math_types.hpp"
#include "control/motor_mixer.hpp"

namespace homefly
{

enum class FlightMode : uint8_t
{
	Disarmed = 0,
	Stabilize = 1,
	Failsafe = 2
};

enum class FailsafeReason : uint8_t
{
	None = 0,
	ImuReadFault,
	ImuTimeout,
	InvalidImuData,
	ControlOverrun,
	MotorFault,
	PreArmFailed
};

struct VehicleState
{
	Attitude attitude{};
	Vec3 rate{};
	uint64_t timestamp_us{0};
	bool valid{false};
};

// Commands from a pilot or a ground station. Angles in radians.
// Armed state changes are explicit requests, never an implicit level.
struct Setpoint
{
	float throttle{0.0f};
	float roll{0.0f};
	float pitch{0.0f};
	float yaw_rate{0.0f};
	bool arm_request{false};
	bool disarm_request{false};
};

struct ControlOutput
{
	float motors[MotorMixer::kMotorCount]{};
	uint8_t count{MotorMixer::kMotorCount};
};

} // namespace homefly
