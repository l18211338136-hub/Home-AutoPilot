#pragma once

namespace homefly
{

struct PidGains
{
	float kp{0.0f};
	float ki{0.0f};
	float kd{0.0f};
};

// Fixed point in time, allocation free PID.
// Derivative acts on the measurement to avoid setpoint kicks.
// Integral is hard clamped (anti-windup), output is clamped too.
class PidController
{
public:
	PidController(
		const PidGains& gains,
		float output_min,
		float output_max,
		float integral_max
	) noexcept;

	void reset() noexcept;

	void set_gains(
		const PidGains& gains
	) noexcept;

	float update(
		float setpoint,
		float measurement,
		float dt_s
	) noexcept;

	float last_output() const noexcept;

	float integral() const noexcept;

private:
	PidGains _gains;
	float _output_min;
	float _output_max;
	float _integral_max;
	float _integral{0.0f};
	float _prev_measurement{0.0f};
	float _last_output{0.0f};
	bool _has_prev{false};
};

} // namespace homefly
