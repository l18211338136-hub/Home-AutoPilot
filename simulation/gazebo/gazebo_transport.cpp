#include "simulation/gazebo/gazebo_transport.hpp"

#include <chrono>

namespace homefly
{

namespace
{
constexpr uint32_t kTaskBudget = 16u;
constexpr auto kIdleSleep = std::chrono::microseconds(100);
}

GazeboTransport::GazeboTransport() noexcept = default;

GazeboTransport::~GazeboTransport()
{
	stop();
}

bool GazeboTransport::start(
	const GazeboTransportConfig& config
) noexcept
{
	stop();

	if (!_socket.open(config.local_port, config.host, config.remote_port))
	{
		return false;
	}

	_config = config;
	_imu_in.reset();
	_motor_out.reset();
	_last_seq.store(0u, std::memory_order_relaxed);
	_imu_received.store(0u, std::memory_order_relaxed);
	_imu_rejected.store(0u, std::memory_order_relaxed);
	_imu_dropped.store(0u, std::memory_order_relaxed);
	_motor_sent.store(0u, std::memory_order_relaxed);
	_motor_dropped.store(0u, std::memory_order_relaxed);
	_send_failures.store(0u, std::memory_order_relaxed);
	_stop.store(false, std::memory_order_release);
	_running.store(true, std::memory_order_release);

	try
	{
		_thread = std::thread([this]() noexcept { task(); });
	}
	catch (...)
	{
		_stop.store(true, std::memory_order_release);
		_running.store(false, std::memory_order_release);
		_socket.close();
		return false;
	}

	return true;
}

void GazeboTransport::stop() noexcept
{
	_stop.store(true, std::memory_order_release);

	if (_thread.joinable())
	{
		_thread.join();
	}

	_running.store(false, std::memory_order_release);
	_socket.close();
}

void GazeboTransport::task() noexcept
{
	while (!_stop.load(std::memory_order_acquire))
	{
		bool did_work = false;

		// Outbound: motor commands queued by the control loop.
		GazeboFrame motor{};

		for (uint32_t i = 0; i < kTaskBudget; ++i)
		{
			if (!_motor_out.pop(motor))
			{
				break;
			}

			if (_socket.send(&motor, sizeof(motor)))
			{
				_motor_sent.fetch_add(1u, std::memory_order_relaxed);
			}
			else
			{
				_send_failures.fetch_add(1u, std::memory_order_relaxed);
			}

			did_work = true;
		}

		// Inbound: Gazebo IMU datagrams. Duplicate seq is ignored so the
		// bridge can safely retransmit while HomeFly is still starting.
		for (uint32_t i = 0; i < kTaskBudget; ++i)
		{
			GazeboFrame frame{};
			uint32_t length = 0;
			const int received = _socket.receive(&frame, sizeof(frame), length);

			if (received <= 0)
			{
				break;
			}

			did_work = true;

			if (length != sizeof(GazeboFrame) ||
				!gazebo_frame_validate(frame) ||
				frame.type != static_cast<uint8_t>(GazeboFrameType::Imu) ||
				frame.seq <= _last_seq.load(std::memory_order_relaxed))
			{
				_imu_rejected.fetch_add(1u, std::memory_order_relaxed);
				continue;
			}

			_last_seq.store(frame.seq, std::memory_order_relaxed);

			if (_imu_in.push(frame))
			{
				_imu_received.fetch_add(1u, std::memory_order_relaxed);
			}
			else
			{
				_imu_dropped.fetch_add(1u, std::memory_order_relaxed);
			}
		}

		if (did_work)
		{
			std::this_thread::yield();
		}
		else
		{
			std::this_thread::sleep_for(kIdleSleep);
		}
	}

	// Final sweep: the control loop queues the last motor frame and may stop
	// the transport immediately afterwards. Without this flush the bridge
	// would wait forever for that frame and time out after a full SITL run.
	GazeboFrame motor{};

	while (_motor_out.pop(motor))
	{
		if (_socket.send(&motor, sizeof(motor)))
		{
			_motor_sent.fetch_add(1u, std::memory_order_relaxed);
		}
		else
		{
			_send_failures.fetch_add(1u, std::memory_order_relaxed);
		}
	}

	_running.store(false, std::memory_order_release);
}

bool GazeboTransport::wait_for_imu() noexcept
{
	const auto deadline = std::chrono::steady_clock::now() +
		std::chrono::milliseconds(_config.wait_timeout_ms);

	while (_imu_in.pending() == 0u)
	{
		if (_stop.load(std::memory_order_acquire) ||
			std::chrono::steady_clock::now() >= deadline)
		{
			return false;
		}

		std::this_thread::sleep_for(kIdleSleep);
	}

	return true;
}

bool GazeboTransport::pop_imu(GazeboFrame& frame) noexcept
{
	return _imu_in.pop(frame);
}

void GazeboTransport::push_motor(const GazeboFrame& frame) noexcept
{
	if (!_motor_out.push(frame))
	{
		_motor_dropped.fetch_add(1u, std::memory_order_relaxed);
	}
}

uint32_t GazeboTransport::pending_imu() const noexcept
{
	return _imu_in.pending();
}

GazeboTransportStats GazeboTransport::stats() const noexcept
{
	GazeboTransportStats result{};
	result.imu_received = _imu_received.load(std::memory_order_relaxed);
	result.imu_rejected = _imu_rejected.load(std::memory_order_relaxed);
	result.imu_dropped = _imu_dropped.load(std::memory_order_relaxed);
	result.motor_sent = _motor_sent.load(std::memory_order_relaxed);
	result.motor_dropped = _motor_dropped.load(std::memory_order_relaxed);
	result.send_failures = _send_failures.load(std::memory_order_relaxed);
	return result;
}

bool GazeboTransport::running() const noexcept
{
	return _running.load(std::memory_order_acquire);
}

} // namespace homefly
