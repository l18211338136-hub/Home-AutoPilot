#pragma once

#include <cstdint>

namespace homefly
{

// Minimal non blocking UDP endpoint. The flight controller never touches this
// class from the control loop: only the low priority transport task calls it.
// POSIX / Winsock differences are confined to udp_socket.cpp.
class UdpSocket
{
public:
	UdpSocket() noexcept;
	~UdpSocket();

	UdpSocket(const UdpSocket&) = delete;
	UdpSocket& operator=(const UdpSocket&) = delete;

	// Binds local_port on any interface and sets the default send target.
	bool open(
		uint16_t local_port,
		const char* remote_host,
		uint16_t remote_port
	) noexcept;

	void close() noexcept;

	bool is_open() const noexcept;

	bool send(
		const void* data,
		uint32_t length
	) noexcept;

	// Returns: >0 bytes received, 0 = no datagram available, <0 = error.
	int receive(
		void* buffer,
		uint32_t capacity,
		uint32_t& length
	) noexcept;

private:
#ifdef _WIN32
	uintptr_t _handle{0};
#else
	int _handle{-1};
#endif
	bool _open{false};
	struct SockAddrStorage
	{
		uint32_t words[4]{};
	};
	SockAddrStorage _remote{};
};

} // namespace homefly
