#include <cmath>
#include <cstdint>
#include <limits>

#include "configuration/flight_config.hpp"
#include "flight_core/flight_controller.hpp"
#include "hal/i_clock.hpp"
#include "homefly_test.hpp"
#include "simulation/sim_clock.hpp"
#include "simulation/sim_imu.hpp"
#include "simulation/sim_motor.hpp"
#include "simulation/sim_plant.hpp"

using namespace homefly;

namespace
{

// Injectable IMU for fault scenarios that a healthy plant cannot produce.
class ScriptedImu : public IImu
{
public:
	explicit ScriptedImu(
		const IClock& clock
	) noexcept
		: _clock(clock)
	{
	}

	bool initialize() override
	{
		return true;
	}

	bool read(
		ImuData& data
	) override
	{
		if (!_ok)
		{
			return false;
		}

		data = _sample;
		data.timestamp_us = _clock.now_us() - _stale_us;
		return true;
	}

	void set_sample(
		const ImuData& sample
	) noexcept
	{
		_sample = sample;
	}

	void set_stale_us(
		uint64_t stale_us
	) noexcept
	{
		_stale_us = stale_us;
	}

	void set_ok(
		bool ok
	) noexcept
	{
		_ok = ok;
	}

private:
	const IClock& _clock;
	ImuData _sample{};
	uint64_t _stale_us{0};
	bool _ok{true};
};

ImuData healthy_sample()
{
	ImuData data{};
	data.accel_z = kGravity;
	return data;
}

struct Rig
{
	SimClock clock;
	SimPlant plant;
	FlightConfig config;
	SimImu imu;
	SimMotor motor;
	FlightController controller;

	Rig()
		: imu(plant, clock),
		  motor(plant),
		  controller(imu, motor, clock, config)
	{
		plant.set_initial_attitude(0.2f, -0.1f);
		controller.initialize();
	}
};

void run_cycles(
	Rig& rig,
	uint32_t cycles,
	const Setpoint& setpoint
)
{
	const float dt_s = static_cast<float>(rig.config.control_period_us) * 1.0e-6f;

	for (uint32_t i = 0; i < cycles; ++i)
	{
		rig.clock.advance_us(rig.config.control_period_us);
		rig.controller.step(setpoint);
		rig.plant.step(dt_s);
	}
}

bool motors_are_zero(
	const SimMotor& motor
)
{
	for (uint8_t i = 0; i < MotorMixer::kMotorCount; ++i)
	{
		if (motor.applied_output(i) > 1.0e-6f)
		{
			return false;
		}
	}

	return true;
}

} // namespace

TEST(FlightController, DisarmedByDefault)
{
	Rig rig;
	Setpoint setpoint{};
	setpoint.throttle = 0.5f;
	run_cycles(rig, 10, setpoint);
	EXPECT_TRUE(rig.controller.mode() == FlightMode::Disarmed);
	EXPECT_TRUE(motors_are_zero(rig.motor));
	EXPECT_FALSE(rig.motor.is_armed());
}

TEST(FlightController, StabilizesInitialAttitude)
{
	Rig rig;
	Setpoint setpoint{};
	setpoint.throttle = 0.5f;
	setpoint.arm_request = true;
	run_cycles(rig, 2000, setpoint);
	EXPECT_TRUE(rig.controller.mode() == FlightMode::Stabilize);
	EXPECT_TRUE(rig.motor.is_armed());
	EXPECT_TRUE(std::fabs(rig.controller.state().attitude.roll) < 0.05f);
	EXPECT_TRUE(std::fabs(rig.controller.state().attitude.pitch) < 0.05f);
}

TEST(FlightController, ImuDropoutTriggersFailsafe)
{
	Rig rig;
	Setpoint setpoint{};
	setpoint.throttle = 0.5f;
	setpoint.arm_request = true;
	run_cycles(rig, 100, setpoint);

	rig.imu.set_fault(true);
	rig.clock.advance_us(rig.config.control_period_us);
	EXPECT_FALSE(rig.controller.step(setpoint));
	EXPECT_TRUE(rig.controller.mode() == FlightMode::Failsafe);
	EXPECT_TRUE(rig.controller.failsafe_reason() == FailsafeReason::ImuReadFault);
	EXPECT_TRUE(motors_are_zero(rig.motor));
}

TEST(FlightController, FailsafeIsStickyAfterSensorRecovery)
{
	Rig rig;
	Setpoint setpoint{};
	setpoint.throttle = 0.5f;
	setpoint.arm_request = true;
	run_cycles(rig, 100, setpoint);

	rig.imu.set_fault(true);
	rig.clock.advance_us(rig.config.control_period_us);
	rig.controller.step(setpoint);
	rig.imu.set_fault(false);
	run_cycles(rig, 10, setpoint);
	EXPECT_TRUE(rig.controller.mode() == FlightMode::Failsafe);
	EXPECT_TRUE(motors_are_zero(rig.motor));
}

TEST(FlightController, ExplicitDisarmStopsMotors)
{
	Rig rig;
	Setpoint armed{};
	armed.throttle = 0.5f;
	armed.arm_request = true;
	run_cycles(rig, 100, armed);
	EXPECT_TRUE(rig.motor.is_armed());

	Setpoint disarmed{};
	disarmed.disarm_request = true;
	run_cycles(rig, 1, disarmed);
	EXPECT_TRUE(rig.controller.mode() == FlightMode::Disarmed);
	EXPECT_TRUE(motors_are_zero(rig.motor));
}

TEST(FlightController, StaleImuSampleTriggersTimeout)
{
	SimClock clock;
	SimPlant plant;
	FlightConfig config;
	ScriptedImu imu(clock);
	imu.set_sample(healthy_sample());
	imu.set_stale_us(config.imu_timeout_us + 1000u);
	SimMotor motor(plant);
	FlightController controller(imu, motor, clock, config);
	EXPECT_TRUE(controller.initialize());

	Setpoint setpoint{};
	setpoint.arm_request = true;
	clock.advance_us(config.control_period_us);
	EXPECT_FALSE(controller.step(setpoint));
	EXPECT_TRUE(controller.failsafe_reason() == FailsafeReason::ImuTimeout);
	EXPECT_TRUE(motors_are_zero(motor));
}

TEST(FlightController, InvalidImuDataTriggersFailsafe)
{
	SimClock clock;
	SimPlant plant;
	FlightConfig config;
	ScriptedImu imu(clock);
	ImuData bad = healthy_sample();
	bad.gyro_x = std::numeric_limits<float>::quiet_NaN();
	imu.set_sample(bad);
	SimMotor motor(plant);
	FlightController controller(imu, motor, clock, config);
	EXPECT_TRUE(controller.initialize());

	Setpoint setpoint{};
	clock.advance_us(config.control_period_us);
	EXPECT_FALSE(controller.step(setpoint));
	EXPECT_TRUE(controller.failsafe_reason() == FailsafeReason::InvalidImuData);
}

TEST(FlightController, ControlOverrunTriggersFailsafe)
{
	Rig rig;
	Setpoint setpoint{};
	setpoint.throttle = 0.5f;
	setpoint.arm_request = true;
	run_cycles(rig, 100, setpoint);

	rig.clock.advance_us(rig.config.control_period_us * rig.config.overrun_factor + 1000u);
	EXPECT_FALSE(rig.controller.step(setpoint));
	EXPECT_TRUE(rig.controller.failsafe_reason() == FailsafeReason::ControlOverrun);
	EXPECT_TRUE(motors_are_zero(rig.motor));
}

HOMEFLY_TEST_MAIN()
