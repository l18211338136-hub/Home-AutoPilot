#include <cstdint>

#include "drivers/motor/drv_pwm_motor.hpp"
#include "homefly_test.hpp"

using namespace homefly;

namespace
{

class RecordingPwm : public IPwmOutput
{
public:
	bool initialize() override
	{
		_initialized = true;
		return true;
	}

	bool write_pulse_us(
		uint8_t channel,
		uint16_t pulse_us
	) override
	{
		if (channel >= 4u)
		{
			return false;
		}

		_pulses[channel] = pulse_us;
		++_writes;
		return true;
	}

	void set_all_min() override
	{
		for (uint16_t& pulse : _pulses)
		{
			pulse = kMinPulse;
		}

		++_min_calls;
	}

	uint8_t channel_count() const override
	{
		return 4u;
	}

	uint16_t pulse(
		uint8_t channel
	) const noexcept
	{
		return _pulses[channel];
	}

	uint32_t writes() const noexcept
	{
		return _writes;
	}

	uint32_t min_calls() const noexcept
	{
		return _min_calls;
	}

	bool initialized() const noexcept
	{
		return _initialized;
	}

	static constexpr uint16_t kMinPulse = 1000u;

private:
	uint16_t _pulses[4]{kMinPulse, kMinPulse, kMinPulse, kMinPulse};
	uint32_t _writes{0};
	uint32_t _min_calls{0};
	bool _initialized{false};
};

} // namespace

TEST(PwmMotor, InitializeForcesMinimumPulse)
{
	RecordingPwm pwm;
	PwmMotor motor(pwm, 1000u, 2000u);
	EXPECT_TRUE(motor.initialize());
	EXPECT_TRUE(pwm.initialized());
	EXPECT_TRUE(pwm.min_calls() >= 1u);
	EXPECT_FALSE(motor.is_armed());
	EXPECT_EQ(motor.channel_count(), 4);
}

TEST(PwmMotor, DisarmedDriverCannotSpinMotors)
{
	RecordingPwm pwm;
	PwmMotor motor(pwm, 1000u, 2000u);
	motor.initialize();
	const float outputs[4]{1.0f, 1.0f, 1.0f, 1.0f};
	EXPECT_TRUE(motor.write(outputs, 4));
	EXPECT_EQ(pwm.writes(), 0u);
	EXPECT_EQ(pwm.pulse(0), RecordingPwm::kMinPulse);
}

TEST(PwmMotor, MapsNormalizedThrustToPulseWidth)
{
	RecordingPwm pwm;
	PwmMotor motor(pwm, 1000u, 2000u);
	motor.initialize();
	EXPECT_TRUE(motor.arm());
	const float outputs[4]{0.0f, 0.5f, 1.0f, 0.25f};
	EXPECT_TRUE(motor.write(outputs, 4));
	EXPECT_EQ(pwm.pulse(0), 1000);
	EXPECT_EQ(pwm.pulse(1), 1500);
	EXPECT_EQ(pwm.pulse(2), 2000);
	EXPECT_EQ(pwm.pulse(3), 1250);
}

TEST(PwmMotor, DisarmRestoresSafeState)
{
	RecordingPwm pwm;
	PwmMotor motor(pwm, 1000u, 2000u);
	motor.initialize();
	motor.arm();
	const float outputs[4]{1.0f, 1.0f, 1.0f, 1.0f};
	motor.write(outputs, 4);
	motor.disarm();
	EXPECT_EQ(pwm.pulse(0), RecordingPwm::kMinPulse);
	EXPECT_FALSE(motor.is_armed());
}

TEST(PwmMotor, RejectsInvalidOutputCount)
{
	RecordingPwm pwm;
	PwmMotor motor(pwm, 1000u, 2000u);
	motor.initialize();
	motor.arm();
	const float outputs[5]{0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
	EXPECT_FALSE(motor.write(outputs, 5));
	EXPECT_FALSE(motor.write(nullptr, 4));
}

HOMEFLY_TEST_MAIN()
