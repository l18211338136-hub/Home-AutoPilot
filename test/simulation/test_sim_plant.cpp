#include <cmath>

#include "control/math_types.hpp"
#include "homefly_test.hpp"
#include "simulation/sim_plant.hpp"

using namespace homefly;

TEST(SimPlant, ImuSampleMatchesGravityAtLevel)
{
	SimPlant plant;
	const ImuData data = plant.imu_sample(1234u);
	EXPECT_EQ(data.timestamp_us, 1234u);
	EXPECT_NEAR(data.accel_x, 0.0f, 1.0e-5f);
	EXPECT_NEAR(data.accel_y, 0.0f, 1.0e-5f);
	EXPECT_NEAR(data.accel_z, kGravity, 1.0e-4f);
}

TEST(SimPlant, ImuSampleReflectsRoll)
{
	SimPlant plant;
	plant.set_initial_attitude(0.2f, 0.0f);
	const ImuData data = plant.imu_sample(0u);
	EXPECT_NEAR(data.accel_y, kGravity * std::sin(0.2f), 1.0e-4f);
}

TEST(SimPlant, LeftHeavyMotorsProduceNegativeRollRate)
{
	SimPlant plant;
	const float outputs[MotorMixer::kMotorCount]{1.0f, 0.0f, 1.0f, 0.0f};
	plant.set_motor_outputs(outputs, MotorMixer::kMotorCount);
	plant.step(0.01f);
	EXPECT_TRUE(plant.rate().x < 0.0f);
	EXPECT_TRUE(plant.attitude().roll < 0.0f);
}

TEST(SimPlant, EqualThrustKeepsAttitude)
{
	SimPlant plant;
	const float outputs[MotorMixer::kMotorCount]{0.5f, 0.5f, 0.5f, 0.5f};
	plant.set_motor_outputs(outputs, MotorMixer::kMotorCount);
	plant.step(0.01f);
	EXPECT_NEAR(plant.rate().x, 0.0f, 1.0e-6f);
	EXPECT_NEAR(plant.attitude().roll, 0.0f, 1.0e-6f);
}

TEST(SimPlant, ResetClearsStateAndOutputs)
{
	SimPlant plant;
	plant.set_initial_attitude(0.3f, -0.2f);
	const float outputs[MotorMixer::kMotorCount]{1.0f, 0.0f, 0.0f, 0.0f};
	plant.set_motor_outputs(outputs, MotorMixer::kMotorCount);
	plant.step(0.01f);
	plant.reset();
	EXPECT_NEAR(plant.attitude().roll, 0.0f, 1.0e-6f);
	EXPECT_NEAR(plant.motor_output(0), 0.0f, 1.0e-6f);
}

HOMEFLY_TEST_MAIN()
