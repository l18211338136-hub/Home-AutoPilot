#pragma once

#include <cstdint>

namespace homefly
{

// Monotonic clock in microseconds. Must never go backwards.
class IClock
{
public:
	virtual ~IClock() = default;

	virtual uint64_t now_us() const = 0;
};

} // namespace homefly
