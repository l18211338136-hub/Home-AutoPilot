#include "flight_core/flight_controller.hpp"

#include "control/math_types.hpp"

namespace homefly
{

FlightController::FlightController(
	IImu& imu,
	IMotor& motor,
	IClock& clock,
	const FlightConfig& config
) noexcept
	: _imu(imu),
	  _motor(motor),
	  _clock(clock),
	  _config(config),
	  _estimator(config.gyro_weight),
	  _roll_angle_pid(config.angle_roll, -config.max_rate_command, config.max_rate_command, config.max_rate_command),
	  _pitch_angle_pid(config.angle_pitch, -config.max_rate_command, config.max_rate_command, config.max_rate_command),
	  _roll_rate_pid(config.rate_roll, -config.max_torque_command, config.max_torque_command, config.max_torque_command),
	  _pitch_rate_pid(config.rate_pitch, -config.max_torque_command, config.max_torque_command, config.max_torque_command),
	  _yaw_rate_pid(config.rate_yaw, -config.max_torque_command, config.max_torque_command, config.max_torque_command),
	  _mixer(config.motor_output_min, config.motor_output_max),
	  _safety(config.imu_timeout_us, config.overrun_factor)
{
}

bool FlightController::initialize() noexcept
{
	_estimator.reset();
	_roll_angle_pid.reset();
	_pitch_angle_pid.reset();
	_roll_rate_pid.reset();
	_pitch_rate_pid.reset();
	_yaw_rate_pid.reset();
	_state = VehicleState{};
	_output = ControlOutput{};
	_last_step_us = 0;

	_motor.disarm();

	_hardware_ready = _imu.initialize() && _motor.initialize();
	return _hardware_ready;
}

bool FlightController::step(
	const Setpoint& setpoint
) noexcept
{
	const uint64_t now_us = _clock.now_us();
	const uint64_t previous_step_us = _last_step_us;
	_last_step_us = now_us;

	float dt_s = static_cast<float>(_config.control_period_us) * 1.0e-6f;

	if (previous_step_us != 0 && now_us > previous_step_us)
	{
		dt_s = static_cast<float>(now_us - previous_step_us) * 1.0e-6f;
	}

	const FailsafeReason timing_fault = _safety.check_timing(
		now_us, previous_step_us, _config.control_period_us);

	if (timing_fault != FailsafeReason::None)
	{
		enter_failsafe(timing_fault);
		return false;
	}

	ImuData sample{};

	if (!_imu.read(sample))
	{
		enter_failsafe(FailsafeReason::ImuReadFault);
		return false;
	}

	// Validate against a timestamp taken AFTER the sample was produced. On real
	// hardware the IMU stamps data in its own ISR, i.e. at or before this
	// point. Using the earlier now_us would reject every sample on a host
	// monotonic clock: a few microseconds of skew look like "future" data.
	// Stale samples are still detected (their timestamp is far behind).
	const uint64_t validation_now_us = _clock.now_us();
	const FailsafeReason sample_fault = _safety.validate_imu(sample, validation_now_us);

	if (sample_fault != FailsafeReason::None)
	{
		enter_failsafe(sample_fault);
		return false;
	}

	if (!_estimator.update(sample, dt_s))
	{
		enter_failsafe(FailsafeReason::InvalidImuData);
		return false;
	}

	_state.attitude = _estimator.attitude();
	_state.rate = Vec3{sample.gyro_x, sample.gyro_y, sample.gyro_z};
	_state.timestamp_us = sample.timestamp_us;
	_state.valid = true;

	StateEvent event = StateEvent::None;

	if (setpoint.arm_request)
	{
		event = StateEvent::ArmRequest;
	}
	else if (setpoint.disarm_request)
	{
		event = StateEvent::DisarmRequest;
	}

	// PreArm gate (SKILL.md 3.4): every unlock must clear all pre-arm checks.
	// Throttle-at-min only matters at the arm transition; once Stabilize the FSM
	// ignores prearm_ok (it is only consulted on ArmRequest from Disarmed).
	_prearm_report = _prearm.check(PreArmInputs{
		.imu_initialized = _hardware_ready,
		.imu_data_valid = _state.valid,
		.throttle = setpoint.throttle,
		.config = &_config,
		.calibration_required = false, // TODO(HF-004): no calibration subsystem yet
		.calibration_ok = true,
		.has_battery = false,
		.has_severe_fault = (_fsm.mode() == FlightMode::Failsafe)
	});
	const bool prearm_ok = _prearm_report.all_passed();

	const FlightMode mode = _fsm.update(event, _state.valid, prearm_ok);

	if (mode != FlightMode::Stabilize)
	{
		output_safe_state();
		return true;
	}

	if (!_motor.is_armed() && !_motor.arm())
	{
		enter_failsafe(FailsafeReason::MotorFault);
		return false;
	}

	const float roll_sp = clamp(setpoint.roll, -_config.max_angle_command_rad, _config.max_angle_command_rad);
	const float pitch_sp = clamp(setpoint.pitch, -_config.max_angle_command_rad, _config.max_angle_command_rad);

	const float roll_rate_sp = _roll_angle_pid.update(roll_sp, _state.attitude.roll, dt_s);
	const float pitch_rate_sp = _pitch_angle_pid.update(pitch_sp, _state.attitude.pitch, dt_s);

	const float roll_torque = _roll_rate_pid.update(roll_rate_sp, _state.rate.x, dt_s);
	const float pitch_torque = _pitch_rate_pid.update(pitch_rate_sp, _state.rate.y, dt_s);
	const float yaw_torque = _yaw_rate_pid.update(setpoint.yaw_rate, _state.rate.z, dt_s);

	const MixerInput input{setpoint.throttle, roll_torque, pitch_torque, yaw_torque};
	_output.count = _mixer.mix(input, _output.motors, MotorMixer::kMotorCount);

	if (_output.count != MotorMixer::kMotorCount ||
	    !_motor.write(_output.motors, _output.count))
	{
		enter_failsafe(FailsafeReason::MotorFault);
		return false;
	}

	return true;
}

void FlightController::enter_failsafe(
	FailsafeReason reason
) noexcept
{
	StateEvent event = StateEvent::SensorFault;

	if (reason == FailsafeReason::MotorFault)
	{
		event = StateEvent::MotorFault;
	}
	else if (reason == FailsafeReason::ControlOverrun)
	{
		event = StateEvent::ControlFault;
	}

	_fsm.update(event, false, true, reason);
	_state.valid = false;
	output_safe_state();
}

void FlightController::output_safe_state() noexcept
{
	for (float& motor_output : _output.motors)
	{
		motor_output = _config.motor_output_min;
	}

	_output.count = MotorMixer::kMotorCount;
	_motor.disarm();
	_motor.write(_output.motors, _output.count);
}

const VehicleState& FlightController::state() const noexcept
{
	return _state;
}

const ControlOutput& FlightController::output() const noexcept
{
	return _output;
}

FlightMode FlightController::mode() const noexcept
{
	return _fsm.mode();
}

FailsafeReason FlightController::failsafe_reason() const noexcept
{
	return _fsm.failsafe_reason();
}

const PreArmReport& FlightController::prearm_report() const noexcept
{
	return _prearm_report;
}

} // namespace homefly
