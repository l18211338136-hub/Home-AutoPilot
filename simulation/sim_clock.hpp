#pragma once

#include <cstdint>

#include "hal/i_clock.hpp"

namespace homefly
{

// Deterministic virtual time. The application advances it by exactly one
// control period per cycle, so tests and simulation are bit reproducible.
class SimClock : public IClock
{
public:
	uint64_t now_us() const override;

	void advance_us(
		uint64_t delta_us
	) noexcept;

	void reset() noexcept;

private:
	uint64_t _now_us{0};
};

} // namespace homefly
