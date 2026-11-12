#pragma once

#include <chrono>
#include <cstdint>

#include "hal/i_clock.hpp"

namespace homefly
{

// Wall-clock independent monotonic clock. Also usable on Windows hosts for
// Linux Native development because it only relies on the C++ standard library.
class LinuxClock : public IClock
{
public:
	uint64_t now_us() const override;

private:
	std::chrono::steady_clock::time_point _epoch{std::chrono::steady_clock::now()};
};

} // namespace homefly
