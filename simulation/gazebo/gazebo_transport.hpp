#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include "simulation/gazebo/gazebo_protocol.hpp"
#include "simulation/gazebo/spsc_ring.hpp"
#include "simulation/gazebo/udp_socket.hpp"

namespace homefly
{

struct GazeboTransportConfig
{
	uint16_t local_port{14550};   // HomeFly receives IMU here
	uint16_t remote_port{14551};  // HomeFly sends motors here
	char host[64]{"127.0.0.1"};
	uint32_t wait_timeout_ms{3000};
};

struct GazeboTransportStats
{
	uint32_t imu_received{0};
	uint32_t imu_rejected{0};  // bad checksum / wrong type / duplicate seq
	uint32_t imu_dropped{0};   // ring full
	uint32_t motor_sent{0};
	uint32_t motor_dropped{0};
	uint32_t send_failures{0};
};

// Owns the ONLY UDP I/O in the SITL path. A low priority task receives IMU
// datagrams into an SPSC ring and drains motor frames out of another ring.
// The control loop only calls wait_for_imu() / pop_imu() / push_motor(), all
// of which are allocation free, non blocking and perform no system calls.
class GazeboTransport
{
public:
	GazeboTransport() noexcept;
	~GazeboTransport();

	GazeboTransport(const GazeboTransport&) = delete;
	GazeboTransport& operator=(const GazeboTransport&) = delete;

	bool start(
		const GazeboTransportConfig& config
	) noexcept;

	void stop() noexcept;

	// Control loop side (no I/O).
	bool wait_for_imu() noexcept;
	bool pop_imu(GazeboFrame& frame) noexcept;
	void push_motor(const GazeboFrame& frame) noexcept;

	uint32_t pending_imu() const noexcept;
	GazeboTransportStats stats() const noexcept;
	bool running() const noexcept;

private:
	void task() noexcept;

	UdpSocket _socket;
	SpscRing<GazeboFrame, 32> _imu_in;
	SpscRing<GazeboFrame, 32> _motor_out;
	GazeboTransportConfig _config{};
	std::thread _thread;
	std::atomic<bool> _stop{true};
	std::atomic<bool> _running{false};
	std::atomic<uint32_t> _last_seq{0};
	std::atomic<bool> _have_seq{false}; // becomes true after the first accepted frame
	std::atomic<uint32_t> _imu_received{0};
	std::atomic<uint32_t> _imu_rejected{0};
	std::atomic<uint32_t> _imu_dropped{0};
	std::atomic<uint32_t> _motor_sent{0};
	std::atomic<uint32_t> _motor_dropped{0};
	std::atomic<uint32_t> _send_failures{0};
};

} // namespace homefly
