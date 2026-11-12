#pragma once

#include "configuration/flight_config.hpp"
#include "control/attitude_estimator.hpp"
#include "control/motor_mixer.hpp"
#include "control/pid_controller.hpp"
#include "flight_core/flight_state_machine.hpp"
#include "flight_core/safety_monitor.hpp"
#include "flight_core/types.hpp"
#include "hal/i_clock.hpp"
#include "hal/i_imu.hpp"
#include "hal/i_motor.hpp"

namespace homefly
{

// One step() is one control cycle. The class owns no hardware: every
// dependency is injected through the constructor (dependency inversion).
// step() performs no allocation, no blocking I/O and has bounded runtime.
class FlightController
{
public:
	FlightController(
		IImu& imu,
		IMotor& motor,
		IClock& clock,
		const FlightConfig& config
	) noexcept;

	bool initialize() noexcept;

	// Returns false when the cycle ended in failsafe (motors are disarmed).
	bool step(
		const Setpoint& setpoint
	) noexcept;

	const VehicleState& state() const noexcept;

	const ControlOutput& output() const noexcept;

	FlightMode mode() const noexcept;

	FailsafeReason failsafe_reason() const noexcept;

private:
	void enter_failsafe(
		FailsafeReason reason
	) noexcept;

	void output_safe_state() noexcept;

	IImu& _imu;
	IMotor& _motor;
	IClock& _clock;
	const FlightConfig& _config;

	AttitudeEstimator _estimator;
	PidController _roll_angle_pid;
	PidController _pitch_angle_pid;
	PidController _roll_rate_pid;
	PidController _pitch_rate_pid;
	PidController _yaw_rate_pid;
	MotorMixer _mixer;
	SafetyMonitor _safety;
	FlightStateMachine _fsm;

	VehicleState _state{};
	ControlOutput _output{};
	uint64_t _last_step_us{0};
};

} // namespace homefly
