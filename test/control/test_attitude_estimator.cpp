#include <cmath>
#include <limits>

#include "control/attitude_estimator.hpp"
#include "control/math_types.hpp"
#include "homefly_test.hpp"

using namespace homefly;

namespace
{

ImuData level_sample()
{
	ImuData data{};
	data.timestamp_us = 0;
	data.accel_z = kGravity;
	return data;
}

} // namespace

TEST(AttitudeEstimator, LevelIsZero)
{
	AttitudeEstimator estimator(0.0f);
	EXPECT_TRUE(estimator.update(level_sample(), 0.001f));
	EXPECT_TRUE(estimator.valid());
	EXPECT_NEAR(estimator.attitude().roll, 0.0f, 1.0e-5f);
	EXPECT_NEAR(estimator.attitude().pitch, 0.0f, 1.0e-5f);
}

TEST(AttitudeEstimator, RecoversPitchFromGravity)
{
	AttitudeEstimator estimator(0.0f);
	ImuData data = level_sample();
	data.accel_x = -kGravity * std::sin(0.2f);
	data.accel_z = kGravity * std::cos(0.2f);
	EXPECT_TRUE(estimator.update(data, 0.001f));
	EXPECT_NEAR(estimator.attitude().pitch, 0.2f, 1.0e-3f);
}

TEST(AttitudeEstimator, RecoversRollFromGravity)
{
	AttitudeEstimator estimator(0.0f);
	ImuData data = level_sample();
	data.accel_y = kGravity * std::sin(-0.15f);
	data.accel_z = kGravity * std::cos(-0.15f);
	EXPECT_TRUE(estimator.update(data, 0.001f));
	EXPECT_NEAR(estimator.attitude().roll, -0.15f, 1.0e-3f);
}

TEST(AttitudeEstimator, IntegratesGyroYaw)
{
	AttitudeEstimator estimator(1.0f);
	ImuData data = level_sample();
	data.gyro_z = 1.0f;
	EXPECT_TRUE(estimator.update(data, 0.1f));
	EXPECT_NEAR(estimator.attitude().yaw, 0.1f, 1.0e-4f);
}

TEST(AttitudeEstimator, RejectsNonFiniteData)
{
	AttitudeEstimator estimator(0.98f);
	ImuData data = level_sample();
	data.accel_x = std::numeric_limits<float>::quiet_NaN();
	EXPECT_FALSE(estimator.update(data, 0.001f));
	EXPECT_FALSE(estimator.valid());
}

TEST(AttitudeEstimator, RejectsFreeFall)
{
	AttitudeEstimator estimator(0.98f);
	ImuData data{};
	EXPECT_FALSE(estimator.update(data, 0.001f));
	EXPECT_FALSE(estimator.valid());
}

HOMEFLY_TEST_MAIN()
