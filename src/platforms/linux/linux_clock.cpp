#include "platforms/linux/linux_clock.hpp"

namespace homefly
{

uint64_t LinuxClock::now_us() const
{
	const auto elapsed = std::chrono::steady_clock::now() - _epoch;
	return static_cast<uint64_t>(
		std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

} // namespace homefly
