#pragma once

#include <cstddef>
#include <cstdint>

namespace homefly
{

class IUart
{
public:
	virtual ~IUart() = default;

	virtual bool initialize() = 0;

	// Returns the number of bytes actually written / read.
	virtual std::size_t write(
		const uint8_t* data,
		std::size_t length
	) = 0;

	virtual std::size_t read(
		uint8_t* buffer,
		std::size_t capacity
	) = 0;
};

} // namespace homefly
