#pragma once

#include "control/motor_mixer.hpp"
#include "hal/i_motor.hpp"
#include "simulation/sim_plant.hpp"

namespace homefly
{

// HAL implementation that feeds the simulated plant. A hardware interlock
// forces zero thrust whenever the driver is disarmed.
class SimMotor : public IMotor
{
public:
	explicit SimMotor(
		SimPlant& plant
	) noexcept;

	bool initialize() override;

	bool arm() override;

	void disarm() override;

	bool write(
		const float* outputs,
		uint8_t count
	) override;

	bool is_armed() const override;

	uint8_t channel_count() const override;

	float applied_output(
		uint8_t index
	) const noexcept;

private:
	SimPlant& _plant;
	bool _armed{false};
};

} // namespace homefly
