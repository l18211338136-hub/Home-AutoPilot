#pragma once

#include "hal/axis_sample.hpp"

namespace homefly
{

class IMagnetometer
{
public:
	virtual ~IMagnetometer() = default;

	virtual bool initialize() = 0;

	// Magnetic field in microtesla, body frame.
	virtual bool read(
		AxisSample& data
	) = 0;
};

} // namespace homefly
