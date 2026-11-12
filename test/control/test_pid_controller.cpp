#include <cmath>

#include "control/pid_controller.hpp"
#include "homefly_test.hpp"

using namespace homefly;

TEST(PidController, ProportionalResponse)
{
	PidController pid(PidGains{2.0f, 0.0f, 0.0f}, -10.0f, 10.0f, 5.0f);
	EXPECT_NEAR(pid.update(1.0f, 0.0f, 0.01f), 2.0f, 1.0e-5f);
	EXPECT_NEAR(pid.update(-1.0f, 0.0f, 0.01f), -2.0f, 1.0e-5f);
}

TEST(PidController, OutputIsClamped)
{
	PidController pid(PidGains{100.0f, 0.0f, 0.0f}, -1.0f, 1.0f, 5.0f);
	EXPECT_NEAR(pid.update(1.0f, 0.0f, 0.01f), 1.0f, 1.0e-5f);
	EXPECT_NEAR(pid.update(-1.0f, 0.0f, 0.01f), -1.0f, 1.0e-5f);
}

TEST(PidController, IntegralAntiWindup)
{
	PidController pid(PidGains{0.0f, 10.0f, 0.0f}, -1.0f, 1.0f, 0.5f);

	for (int i = 0; i < 1000; ++i)
	{
		pid.update(1.0f, 0.0f, 0.01f);
	}

	EXPECT_NEAR(pid.integral(), 0.5f, 1.0e-5f);
	EXPECT_NEAR(pid.last_output(), 1.0f, 1.0e-5f);
}

TEST(PidController, DerivativeOnMeasurementAvoidsSetpointKick)
{
	PidController pid(PidGains{0.0f, 0.0f, 1.0f}, -1000.0f, 1000.0f, 5.0f);
	pid.update(0.0f, 0.0f, 0.01f);
	EXPECT_NEAR(pid.update(5.0f, 0.0f, 0.01f), 0.0f, 1.0e-4f);
	EXPECT_NEAR(pid.update(0.0f, 1.0f, 0.01f), -100.0f, 1.0e-2f);
}

TEST(PidController, InvalidDtIsGuarded)
{
	PidController pid(PidGains{1.0f, 0.0f, 0.0f}, -10.0f, 10.0f, 5.0f);
	const float output = pid.update(1.0f, 0.0f, 0.0f);
	EXPECT_TRUE(std::isfinite(output));
	EXPECT_NEAR(output, 1.0f, 1.0e-5f);
}

TEST(PidController, ResetClearsState)
{
	PidController pid(PidGains{1.0f, 1.0f, 0.0f}, -10.0f, 10.0f, 5.0f);
	pid.update(1.0f, 0.0f, 0.01f);
	pid.reset();
	EXPECT_NEAR(pid.integral(), 0.0f, 1.0e-6f);
	EXPECT_NEAR(pid.last_output(), 0.0f, 1.0e-6f);
}

HOMEFLY_TEST_MAIN()
