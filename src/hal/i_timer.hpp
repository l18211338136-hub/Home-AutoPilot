#pragma once

#include <cstdint>

namespace homefly
{

// Pacing helper for application loops. Never called from an ISR or from
// inside the high frequency control path.
class ITimer
{
public:
	virtual ~ITimer() = default;

	virtual void sleep_until_us(
		uint64_t target_us
	) = 0;
};

} // namespace homefly
