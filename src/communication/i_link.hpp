#pragma once

#include <cstddef>
#include <cstdint>

namespace homefly
{

// Byte oriented link. MAVLink / RC / GCS adapters implement this interface;
// the protocol layer never sees a UART or a radio chip.
class ILink
{
public:
	virtual ~ILink() = default;

	virtual bool send(
		const uint8_t* data,
		std::size_t length
	) = 0;

	virtual std::size_t receive(
		uint8_t* buffer,
		std::size_t capacity
	) = 0;
};

} // namespace homefly
