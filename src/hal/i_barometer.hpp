#pragma once

#include <cstdint>

namespace homefly
{

struct BaroData
{
	uint64_t timestamp_us{0};
	float pressure_pa{0.0f};
	float altitude_m{0.0f};
	float temperature_c{0.0f};
};

class IBarometer
{
public:
	virtual ~IBarometer() = default;

	virtual bool initialize() = 0;

	virtual bool read(
		BaroData& data
	) = 0;
};

} // namespace homefly
