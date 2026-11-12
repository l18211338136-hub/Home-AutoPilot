#pragma once

#include "hal/axis_sample.hpp"

namespace homefly
{

class IGyroscope
{
public:
	virtual ~IGyroscope() = default;

	virtual bool initialize() = 0;

	// Angular rate in rad/s, body frame.
	virtual bool read(
		AxisSample& data
	) = 0;
};

} // namespace homefly
