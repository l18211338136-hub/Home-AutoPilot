#pragma once

#include <cstddef>
#include <cstdint>

#include "communication/i_link.hpp"

namespace homefly
{

// Loopback link: records traffic without any transport dependency.
class SimLink : public ILink
{
public:
	bool send(
		const uint8_t* data,
		std::size_t length
	) override;

	std::size_t receive(
		uint8_t* buffer,
		std::size_t capacity
	) override;

	std::size_t sent_frames() const noexcept;

	std::size_t last_length() const noexcept;

private:
	std::size_t _sent_frames{0};
	std::size_t _last_length{0};
};

} // namespace homefly
