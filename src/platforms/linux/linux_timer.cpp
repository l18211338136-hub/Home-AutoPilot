#include "platforms/linux/linux_timer.hpp"

#include <chrono>
#include <thread>

namespace homefly
{

LinuxTimer::LinuxTimer(
	const IClock& clock
) noexcept
	: _clock(clock)
{
}

void LinuxTimer::sleep_until_us(
	uint64_t target_us
)
{
	const uint64_t now_us = _clock.now_us();

	if (target_us <= now_us)
	{
		return;
	}

	std::this_thread::sleep_for(std::chrono::microseconds(target_us - now_us));
}

} // namespace homefly
