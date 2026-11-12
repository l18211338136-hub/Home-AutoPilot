#pragma once

#include <cstdint>

namespace homefly
{

// Binary safe output channel. Implementations live in platform backends
// (Console / File / Telemetry). The common layer never performs any I/O.
//
// write() is called from the low priority drain task, NEVER from the control
// loop, so a blocking implementation is acceptable here.
class ILogSink
{
public:
	virtual ~ILogSink() = default;

	virtual bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept = 0;

	virtual void flush() noexcept {}
};

} // namespace homefly
