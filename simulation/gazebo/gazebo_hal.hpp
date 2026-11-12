#pragma once

#include <atomic>
#include <cstdint>

#include "hal/i_clock.hpp"
#include "hal/i_imu.hpp"
#include "hal/i_motor.hpp"
#include "simulation/gazebo/gazebo_transport.hpp"

namespace homefly
{

// Simulation clock driven by Gazebo timestamps, NOT by the host wall clock.
// now_us() is deliberately updated when the control loop CONSUMES an IMU
// frame (see GazeboImu::read), so FlightController's stale-sample check
// always compares against the very same timestamp. Using "latest received"
// here would falsely trigger ImuTimeout when the bridge runs ahead.
class GazeboClock : public IClock
{
public:
	uint64_t now_us() const override;

	void set_served_us(
		uint64_t timestamp_us
	) noexcept;

private:
	std::atomic<uint64_t> _served_us{0};
};

// IImu implementation backed by Gazebo IMU datagrams. read() only pops from
// the transport's SPSC ring: no socket call, no blocking, no allocation.
class GazeboImu : public IImu
{
public:
	GazeboImu(
		GazeboTransport& transport,
		GazeboClock& clock
	) noexcept;

	void set_fault(
		bool fault
	) noexcept;

	bool initialize() override;

	bool read(
		ImuData& data
	) override;

	const ImuData& last_sample() const noexcept;

private:
	GazeboTransport& _transport;
	GazeboClock& _clock;
	ImuData _last_sample{};
	bool _initialized{false};
	bool _fault{false};
};

// IMotor implementation that publishes normalized Quad-X thrusts to Gazebo.
// A disarmed driver always publishes zero thrust (hardware interlock).
class GazeboMotor : public IMotor
{
public:
	GazeboMotor(
		GazeboTransport& transport,
		GazeboClock& clock
	) noexcept;

	bool initialize() override;
	bool arm() override;
	void disarm() override;

	bool write(
		const float* outputs,
		uint8_t count
	) override;

	bool is_armed() const override;
	uint8_t channel_count() const override;

	bool motors_zero() const noexcept;

private:
	void publish(
		const float* outputs
	) noexcept;

	GazeboTransport& _transport;
	GazeboClock& _clock;
	float _last_outputs[4]{};
	std::atomic<uint32_t> _seq{0};
	bool _armed{false};
	bool _initialized{false};
};

} // namespace homefly
