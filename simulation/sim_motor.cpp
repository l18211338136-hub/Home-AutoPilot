#include "simulation/sim_motor.hpp"

namespace homefly
{

SimMotor::SimMotor(
	SimPlant& plant
) noexcept
	: _plant(plant)
{
}

bool SimMotor::initialize()
{
	_armed = false;
	const float zero[MotorMixer::kMotorCount]{};
	_plant.set_motor_outputs(zero, MotorMixer::kMotorCount);
	return true;
}

bool SimMotor::arm()
{
	_armed = true;
	return true;
}

void SimMotor::disarm()
{
	_armed = false;
	const float zero[MotorMixer::kMotorCount]{};
	_plant.set_motor_outputs(zero, MotorMixer::kMotorCount);
}

bool SimMotor::write(
	const float* outputs,
	uint8_t count
)
{
	if (outputs == nullptr || count != MotorMixer::kMotorCount)
	{
		return false;
	}

	if (!_armed)
	{
		disarm();
		return true;
	}

	_plant.set_motor_outputs(outputs, count);
	return true;
}

bool SimMotor::is_armed() const
{
	return _armed;
}

uint8_t SimMotor::channel_count() const
{
	return MotorMixer::kMotorCount;
}

float SimMotor::applied_output(
	uint8_t index
) const noexcept
{
	return _plant.motor_output(index);
}

} // namespace homefly
