#include "simulation/sim_clock.hpp"

namespace homefly
{

uint64_t SimClock::now_us() const
{
	return _now_us;
}

void SimClock::advance_us(
	uint64_t delta_us
) noexcept
{
	_now_us += delta_us;
}

void SimClock::reset() noexcept
{
	_now_us = 0;
}

} // namespace homefly
