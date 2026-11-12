#include "simulation/sim_link.hpp"

namespace homefly
{

bool SimLink::send(
	const uint8_t* data,
	std::size_t length
)
{
	if (data == nullptr || length == 0)
	{
		return false;
	}

	++_sent_frames;
	_last_length = length;
	return true;
}

std::size_t SimLink::receive(
	uint8_t* buffer,
	std::size_t capacity
)
{
	(void)buffer;
	(void)capacity;
	return 0;
}

std::size_t SimLink::sent_frames() const noexcept
{
	return _sent_frames;
}

std::size_t SimLink::last_length() const noexcept
{
	return _last_length;
}

} // namespace homefly
