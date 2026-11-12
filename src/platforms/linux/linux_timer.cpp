#include "platforms/linux/linux_timer.hpp"

#include <chrono>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <mmsystem.h>
#endif

namespace homefly
{

namespace
{

// Sleep until close to the target, then busy-spin the residual. A pure
// sleep_for() can wake up milliseconds late (scheduler tick + competitor
// threads), which on a 1 kHz loop shows up as ControlOverrun. Spinning the
// last few hundred microseconds keeps the period accurate; the CPU cost is
// bounded to that window and only applies to the application pacing loop.
constexpr uint64_t kSpinWindowUs = 400;

} // namespace

LinuxTimer::LinuxTimer(
	const IClock& clock
) noexcept
	: _clock(clock)
{
#ifdef _WIN32
	// Raise the Windows scheduler tick from the default ~15.6 ms to 1 ms so the
	// real-time application loop actually paces at 1 kHz. On a stock Windows
	// timer every ~16th control cycle overshoots the 3-period overrun limit and
	// trips ControlOverrun, which would otherwise make real-time host runs
	// (including the WiFi uplink) fail spuriously. Restored in the destructor.
	timeBeginPeriod(1);
#endif
}

LinuxTimer::~LinuxTimer() noexcept
{
#ifdef _WIN32
	timeEndPeriod(1);
#endif
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

	const uint64_t remaining_us = target_us - now_us;

	if (remaining_us > kSpinWindowUs)
	{
		std::this_thread::sleep_for(
			std::chrono::microseconds(remaining_us - kSpinWindowUs));
	}

	while (_clock.now_us() < target_us)
	{
		// Busy-wait the residual. Never used inside FlightController::step().
	}
}

} // namespace homefly
