#pragma once

#include <cstddef>
#include <cstdint>

namespace homefly
{

class II2c
{
public:
	virtual ~II2c() = default;

	virtual bool initialize() = 0;

	virtual bool write_register(
		uint8_t device_address,
		uint8_t register_address,
		const uint8_t* data,
		std::size_t length
	) = 0;

	virtual bool read_register(
		uint8_t device_address,
		uint8_t register_address,
		uint8_t* data,
		std::size_t length
	) = 0;
};

} // namespace homefly
