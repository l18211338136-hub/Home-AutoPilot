#pragma once

#include <cstdint>

#include "hal/i_clock.hpp"
#include "hal/i_timer.hpp"

namespace homefly
{

// Pacing for the application loop only. Never used inside step().
class LinuxTimer : public ITimer
{
public:
	explicit LinuxTimer(
		const IClock& clock
	) noexcept;

	void sleep_until_us(
		uint64_t target_us
	) override;

private:
	const IClock& _clock;
};

} // namespace homefly
