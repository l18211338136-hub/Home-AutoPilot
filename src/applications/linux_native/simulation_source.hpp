#pragma once

#include <cstdint>
#include <memory>

#include "hal/i_clock.hpp"
#include "hal/i_imu.hpp"
#include "hal/i_motor.hpp"

namespace homefly
{

struct SimulationSourceConfig
{
	uint32_t control_period_us{1000};
	uint32_t steps{2000};
	bool realtime{false};
	bool gazebo{false};
	uint16_t gazebo_local_port{14550};
	uint16_t gazebo_remote_port{14551};
	const char* gazebo_host{"127.0.0.1"};
	uint32_t gazebo_timeout_ms{3000};
	float initial_roll_rad{0.20f};
	float initial_pitch_rad{-0.12f};
};

// The ONLY thing that differs between the built-in deterministic plant and an
// external Gazebo SITL session. FlightCore, logging, telemetry and flight log
// are identical in both modes: the application injects one of these sources.
class SimulationSource
{
public:
	virtual ~SimulationSource() = default;

	virtual IImu& imu() noexcept = 0;
	virtual IMotor& motor() noexcept = 0;
	virtual IClock& clock() noexcept = 0;

	// Advance (or wait for) one control cycle. Returns false on timeout.
	virtual bool begin_cycle() noexcept = 0;

	// Advance the plant, if the source owns one.
	virtual void end_cycle(float dt_s) noexcept = 0;

	// IMU sample used for the structured flight log (no hardware access).
	virtual ImuData sensor_sample(uint64_t timestamp_us) const noexcept = 0;

	virtual bool motors_zero() const noexcept = 0;
	virtual void inject_fault(bool fault) noexcept = 0;
	virtual const char* name() const noexcept = 0;
	virtual uint32_t cycles() const noexcept = 0;
	virtual void report_stats() const noexcept = 0;
	virtual void shutdown() noexcept = 0;
};

// Returns nullptr when the requested source could not be started (e.g. UDP
// port already in use). Caller must report the failure before exiting.
std::unique_ptr<SimulationSource> create_simulation_source(
	const SimulationSourceConfig& config
);

} // namespace homefly
