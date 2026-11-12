#pragma once

#include <cstddef>
#include <cstdint>

#include "communication/i_link.hpp"
#include "simulation/gazebo/udp_socket.hpp"

namespace homefly
{

// ILink implementation over a non-blocking UDP socket. The flight controller
// only ever calls this from the low-priority command task (via CommandReceiver);
// the control loop never sees a socket.
//
// Reuses the cross-platform UdpSocket from the SITL transport (POSIX / Winsock
// branches already live there) rather than duplicating socket code. This is a
// real feature (ground-station link), not simulation, but on Linux Native it is
// exercised the same way: UDP datagrams in, Setpoint out.
class UdpLink : public ILink
{
public:
	bool open(
		uint16_t local_port,
		const char* remote_host,
		uint16_t remote_port
	) noexcept;

	bool send(
		const uint8_t* data,
		std::size_t length
	) override;

	std::size_t receive(
		uint8_t* buffer,
		std::size_t capacity
	) override;

private:
	UdpSocket _socket{};
};

} // namespace homefly
