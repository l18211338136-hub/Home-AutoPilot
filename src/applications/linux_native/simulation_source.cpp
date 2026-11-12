#include "applications/linux_native/simulation_source.hpp"

#include "control/motor_mixer.hpp"
#include "platforms/common/logging/log.hpp"
#include "platforms/linux/linux_clock.hpp"
#include "platforms/linux/linux_timer.hpp"
#include "simulation/gazebo/gazebo_hal.hpp"
#include "simulation/sim_clock.hpp"
#include "simulation/sim_imu.hpp"
#include "simulation/sim_motor.hpp"
#include "simulation/sim_plant.hpp"

namespace homefly
{

namespace
{

class LocalSimSource final : public SimulationSource
{
public:
	explicit LocalSimSource(
		const SimulationSourceConfig& config
	) noexcept
		: _config(config),
		  _timer(_linux_clock),
		  _imu(_plant, config.realtime
				? static_cast<const IClock&>(_linux_clock)
				: static_cast<const IClock&>(_sim_clock)),
		  _motor(_plant)
	{
		_plant.set_initial_attitude(config.initial_roll_rad, config.initial_pitch_rad);
		_next_cycle_us = _linux_clock.now_us() + config.control_period_us;
	}

	IImu& imu() noexcept override
	{
		return _imu;
	}

	IMotor& motor() noexcept override
	{
		return _motor;
	}

	IClock& clock() noexcept override
	{
		return _config.realtime
			? static_cast<IClock&>(_linux_clock)
			: static_cast<IClock&>(_sim_clock);
	}

	bool begin_cycle() noexcept override
	{
		if (_config.realtime)
		{
			_timer.sleep_until_us(_next_cycle_us);
			_next_cycle_us += _config.control_period_us;
		}
		else
		{
			_sim_clock.advance_us(_config.control_period_us);
		}

		++_cycles;
		return true;
	}

	void end_cycle(float dt_s) noexcept override
	{
		_plant.step(dt_s);
	}

	ImuData sensor_sample(uint64_t timestamp_us) const noexcept override
	{
		return _plant.imu_sample(timestamp_us);
	}

	bool motors_zero() const noexcept override
	{
		for (uint8_t i = 0; i < MotorMixer::kMotorCount; ++i)
		{
			if (_motor.applied_output(i) > 1.0e-6f)
			{
				return false;
			}
		}

		return true;
	}

	void inject_fault(bool fault) noexcept override
	{
		_imu.set_fault(fault);
	}

	const char* name() const noexcept override
	{
		return _config.realtime ? "internal_plant(host_clock)" : "internal_plant(virtual_clock)";
	}

	uint32_t cycles() const noexcept override
	{
		return _cycles;
	}

	void report_stats() const noexcept override
	{
		// Nothing external to report for the in-process plant.
	}

	void shutdown() noexcept override
	{
		// No background task in this mode.
	}

private:
	SimulationSourceConfig _config;
	SimClock _sim_clock;
	LinuxClock _linux_clock;
	LinuxTimer _timer;
	SimPlant _plant;
	SimImu _imu;
	SimMotor _motor;
	uint64_t _next_cycle_us{0};
	uint32_t _cycles{0};
};

class GazeboSource final : public SimulationSource
{
public:
	explicit GazeboSource(
		GazeboTransportConfig config
	) noexcept
		: _imu(_transport, _clock),
		  _motor(_transport, _clock)
	{
		_transport_config = config;
	}

	bool start() noexcept
	{
		return _transport.start(_transport_config);
	}

	IImu& imu() noexcept override
	{
		return _imu;
	}

	IMotor& motor() noexcept override
	{
		return _motor;
	}

	IClock& clock() noexcept override
	{
		return _clock;
	}

	bool begin_cycle() noexcept override
	{
		if (!_transport.wait_for_imu())
		{
			return false;
		}

		++_cycles;
		return true;
	}

	void end_cycle(float dt_s) noexcept override
	{
		// Gazebo owns physics and advances on its own; the bridge applies our
		// motor frame before publishing the next IMU sample (lock step).
		(void)dt_s;
	}

	ImuData sensor_sample(uint64_t timestamp_us) const noexcept override
	{
		ImuData data = _imu.last_sample();
		data.timestamp_us = timestamp_us;
		return data;
	}

	bool motors_zero() const noexcept override
	{
		return _motor.motors_zero();
	}

	void inject_fault(bool fault) noexcept override
	{
		_imu.set_fault(fault);
	}

	const char* name() const noexcept override
	{
		return "gazebo_sitl(udp)";
	}

	uint32_t cycles() const noexcept override
	{
		return _cycles;
	}

	void report_stats() const noexcept override
	{
		const GazeboTransportStats stats = _transport.stats();
		HF_LOG_INFO("GAZEBO", "imu_rx=%u imu_bad=%u imu_drop=%u | motor_tx=%u motor_drop=%u send_fail=%u",
			stats.imu_received, stats.imu_rejected, stats.imu_dropped,
			stats.motor_sent, stats.motor_dropped, stats.send_failures);
	}

	void shutdown() noexcept override
	{
		_transport.stop();
	}

private:
	GazeboTransportConfig _transport_config{};
	GazeboClock _clock;
	GazeboTransport _transport;
	GazeboImu _imu;
	GazeboMotor _motor;
	uint32_t _cycles{0};
};

} // namespace

std::unique_ptr<SimulationSource> create_simulation_source(
	const SimulationSourceConfig& config
)
{
	if (!config.gazebo)
	{
		return std::make_unique<LocalSimSource>(config);
	}

	GazeboTransportConfig transport{};
	transport.local_port = config.gazebo_local_port;
	transport.remote_port = config.gazebo_remote_port;
	transport.wait_timeout_ms = config.gazebo_timeout_ms;

	if (config.gazebo_host != nullptr)
	{
		uint32_t i = 0;

		for (; i + 1u < sizeof(transport.host) && config.gazebo_host[i] != '\0'; ++i)
		{
			transport.host[i] = config.gazebo_host[i];
		}

		transport.host[i] = '\0';
	}

	auto source = std::make_unique<GazeboSource>(transport);

	if (!source->start())
	{
		return nullptr;
	}

	return source;
}

} // namespace homefly
