#pragma once

#include "hal/axis_sample.hpp"

namespace homefly
{

class IAccelerometer
{
public:
	virtual ~IAccelerometer() = default;

	virtual bool initialize() = 0;

	// Specific force in m/s^2, body frame.
	virtual bool read(
		AxisSample& data
	) = 0;
};

} // namespace homefly
