#pragma once

#include <cstddef>
#include <cstdint>

namespace homefly
{

class ISpi
{
public:
	virtual ~ISpi() = default;

	virtual bool initialize() = 0;

	// Full duplex transfer. Length is bounded by the caller supplied buffers.
	virtual bool transfer(
		const uint8_t* tx,
		uint8_t* rx,
		std::size_t length
	) = 0;
};

} // namespace homefly
