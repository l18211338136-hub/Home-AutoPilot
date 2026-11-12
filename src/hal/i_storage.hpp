#pragma once

#include <cstddef>
#include <cstdint>

namespace homefly
{

class IStorage
{
public:
	virtual ~IStorage() = default;

	virtual bool initialize() = 0;

	// Returns the number of bytes read, or 0 when the key is absent.
	virtual std::size_t load(
		uint16_t key,
		uint8_t* buffer,
		std::size_t capacity
	) = 0;

	virtual bool save(
		uint16_t key,
		const uint8_t* data,
		std::size_t length
	) = 0;
};

} // namespace homefly
