#pragma once

#include "hal/i_clock.hpp"
#include "hal/i_imu.hpp"
#include "simulation/sim_plant.hpp"

namespace homefly
{

// HAL implementation backed by the simulated plant. set_fault() emulates a
// sensor dropout so the failsafe path can be tested end to end.
class SimImu : public IImu
{
public:
	SimImu(
		const SimPlant& plant,
		const IClock& clock
	) noexcept;

	void set_fault(
		bool fault
	) noexcept;

	bool initialize() override;

	bool read(
		ImuData& data
	) override;

private:
	const SimPlant& _plant;
	const IClock& _clock;
	bool _initialized{false};
	bool _fault{false};
};

} // namespace homefly
