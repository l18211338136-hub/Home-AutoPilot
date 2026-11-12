#include "simulation/sim_imu.hpp"

namespace homefly
{

SimImu::SimImu(
	const SimPlant& plant,
	const IClock& clock
) noexcept
	: _plant(plant),
	  _clock(clock)
{
}

void SimImu::set_fault(
	bool fault
) noexcept
{
	_fault = fault;
}

bool SimImu::initialize()
{
	_fault = false;
	_initialized = true;
	return true;
}

bool SimImu::read(
	ImuData& data
)
{
	if (!_initialized || _fault)
	{
		return false;
	}

	data = _plant.imu_sample(_clock.now_us());
	return true;
}

} // namespace homefly
