#include "applications/linux_native/udp_link.hpp"

#include <cstdint>

namespace homefly
{

bool UdpLink::open(
	uint16_t local_port,
	const char* remote_host,
	uint16_t remote_port
) noexcept
{
	return _socket.open(local_port, remote_host, remote_port);
}

bool UdpLink::send(
	const uint8_t* data,
	std::size_t length
)
{
	return _socket.send(data, static_cast<uint32_t>(length));
}

std::size_t UdpLink::receive(
	uint8_t* buffer,
	std::size_t capacity
)
{
	uint32_t length = 0;
	const int result = _socket.receive(buffer, static_cast<uint32_t>(capacity), length);
	if (result > 0)
	{
		return static_cast<std::size_t>(length);
	}
	return 0;
}

} // namespace homefly
