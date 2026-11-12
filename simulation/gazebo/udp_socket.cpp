#include "simulation/gazebo/udp_socket.hpp"

#include <cstdio>
#include <cstring>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace homefly
{

namespace
{

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;

// Process wide, intentionally never cleaned up: the OS reclaims Winsock when
// the process exits, and a static destructor could run before late sockets.
bool winsock_ready() noexcept
{
	static struct Guard
	{
		bool ok{false};
		Guard() noexcept
		{
			WSADATA data{};
			ok = WSAStartup(MAKEWORD(2, 2), &data) == 0;
		}
	} guard;
	return guard.ok;
}

bool make_non_blocking(SocketHandle handle) noexcept
{
	u_long mode = 1u;
	return ioctlsocket(handle, FIONBIO, &mode) == 0;
}

bool would_block() noexcept
{
	return WSAGetLastError() == WSAEWOULDBLOCK;
}
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;

bool winsock_ready() noexcept
{
	return true;
}

bool make_non_blocking(SocketHandle handle) noexcept
{
	const int flags = fcntl(handle, F_GETFL, 0);
	return flags >= 0 && fcntl(handle, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool would_block() noexcept
{
	return errno == EWOULDBLOCK || errno == EAGAIN;
}
#endif

} // namespace

UdpSocket::UdpSocket() noexcept = default;

UdpSocket::~UdpSocket()
{
	close();
}

bool UdpSocket::open(
	uint16_t local_port,
	const char* remote_host,
	uint16_t remote_port
) noexcept
{
	close();

	if (!winsock_ready() || remote_host == nullptr)
	{
		return false;
	}

	const SocketHandle handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (handle == kInvalidSocket || !make_non_blocking(handle))
	{
		if (handle != kInvalidSocket)
		{
#ifdef _WIN32
			closesocket(handle);
#else
			::close(handle);
#endif
		}

		return false;
	}

	sockaddr_in local{};
	local.sin_family = AF_INET;
	local.sin_port = htons(local_port);
	local.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(handle, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0)
	{
#ifdef _WIN32
		closesocket(handle);
#else
		::close(handle);
#endif
		return false;
	}

	char port_text[8]{};
	std::snprintf(port_text, sizeof(port_text), "%u", static_cast<unsigned>(remote_port));

	addrinfo hints{};
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	addrinfo* resolved = nullptr;

	if (getaddrinfo(remote_host, port_text, &hints, &resolved) != 0 || resolved == nullptr)
	{
#ifdef _WIN32
		closesocket(handle);
#else
		::close(handle);
#endif
		return false;
	}

	sockaddr_in remote{};
	std::memcpy(&remote, resolved->ai_addr, sizeof(remote));
	remote.sin_family = AF_INET;
	freeaddrinfo(resolved);

	std::memcpy(&_remote, &remote, sizeof(remote));
#ifdef _WIN32
	_handle = static_cast<uintptr_t>(handle);
#else
	_handle = handle;
#endif
	_open = true;
	return true;
}

void UdpSocket::close() noexcept
{
	if (_open)
	{
#ifdef _WIN32
		closesocket(static_cast<SocketHandle>(_handle));
#else
		::close(_handle);
#endif
		_open = false;
	}

#ifdef _WIN32
	_handle = 0;
#else
	_handle = kInvalidSocket;
#endif
}

bool UdpSocket::is_open() const noexcept
{
	return _open;
}

bool UdpSocket::send(
	const void* data,
	uint32_t length
) noexcept
{
	if (!is_open() || data == nullptr)
	{
		return false;
	}

	sockaddr_in remote{};
	std::memcpy(&remote, &_remote, sizeof(remote));

#ifdef _WIN32
	const SocketHandle handle = static_cast<SocketHandle>(_handle);
	const int sent = sendto(handle, static_cast<const char*>(data),
		static_cast<int>(length), 0,
		reinterpret_cast<const sockaddr*>(&remote), sizeof(remote));
#else
	const int sent = static_cast<int>(sendto(_handle, data, length, 0,
		reinterpret_cast<const sockaddr*>(&remote), sizeof(remote)));
#endif
	return sent == static_cast<int>(length);
}

int UdpSocket::receive(
	void* buffer,
	uint32_t capacity,
	uint32_t& length
) noexcept
{
	length = 0;

	if (!is_open() || buffer == nullptr)
	{
		return -1;
	}

#ifdef _WIN32
	const SocketHandle handle = static_cast<SocketHandle>(_handle);
	int received = recvfrom(handle, static_cast<char*>(buffer),
		static_cast<int>(capacity), 0, nullptr, nullptr);
#else
	ssize_t raw = recvfrom(_handle, buffer, capacity, 0, nullptr, nullptr);
	const int received = static_cast<int>(raw);
#endif

	if (received >= 0)
	{
		length = static_cast<uint32_t>(received);
		return received;
	}

	return would_block() ? 0 : -1;
}

} // namespace homefly
