#include "control/motor_mixer.hpp"
#include "homefly_test.hpp"

using namespace homefly;

TEST(MotorMixer, HoverIsSymmetric)
{
	MotorMixer mixer(0.0f, 1.0f);
	float outputs[MotorMixer::kMotorCount]{};
	EXPECT_EQ(mixer.mix(MixerInput{0.5f, 0.0f, 0.0f, 0.0f}, outputs, MotorMixer::kMotorCount), 4);

	for (float value : outputs)
	{
		EXPECT_NEAR(value, 0.5f, 1.0e-6f);
	}
}

TEST(MotorMixer, PositiveRollLiftsRightSide)
{
	MotorMixer mixer(0.0f, 1.0f);
	float outputs[MotorMixer::kMotorCount]{};
	mixer.mix(MixerInput{0.5f, 0.2f, 0.0f, 0.0f}, outputs, MotorMixer::kMotorCount);
	EXPECT_TRUE(outputs[1] > outputs[0]);
	EXPECT_TRUE(outputs[3] > outputs[2]);
	EXPECT_NEAR(outputs[0], 0.3f, 1.0e-6f);
	EXPECT_NEAR(outputs[1], 0.7f, 1.0e-6f);
}

TEST(MotorMixer, PositivePitchLiftsFront)
{
	MotorMixer mixer(0.0f, 1.0f);
	float outputs[MotorMixer::kMotorCount]{};
	mixer.mix(MixerInput{0.5f, 0.0f, 0.2f, 0.0f}, outputs, MotorMixer::kMotorCount);
	EXPECT_TRUE(outputs[0] > outputs[2]);
	EXPECT_TRUE(outputs[1] > outputs[3]);
}

TEST(MotorMixer, YawIsAntisymmetric)
{
	MotorMixer mixer(0.0f, 1.0f);
	float outputs[MotorMixer::kMotorCount]{};
	mixer.mix(MixerInput{0.5f, 0.0f, 0.0f, 0.1f}, outputs, MotorMixer::kMotorCount);
	EXPECT_NEAR(outputs[0], 0.6f, 1.0e-6f);
	EXPECT_NEAR(outputs[1], 0.4f, 1.0e-6f);
	EXPECT_NEAR(outputs[2], 0.4f, 1.0e-6f);
	EXPECT_NEAR(outputs[3], 0.6f, 1.0e-6f);
}

TEST(MotorMixer, OutputsAreClamped)
{
	MotorMixer mixer(0.0f, 1.0f);
	float outputs[MotorMixer::kMotorCount]{};
	mixer.mix(MixerInput{1.0f, 1.0f, 1.0f, 1.0f}, outputs, MotorMixer::kMotorCount);

	for (float value : outputs)
	{
		EXPECT_TRUE(value >= 0.0f);
		EXPECT_TRUE(value <= 1.0f);
	}
}

TEST(MotorMixer, InsufficientCapacityWritesNothing)
{
	MotorMixer mixer(0.0f, 1.0f);
	float outputs[2]{};
	EXPECT_EQ(mixer.mix(MixerInput{}, outputs, 2), 0);
	EXPECT_EQ(mixer.mix(MixerInput{}, nullptr, 4), 0);
}

HOMEFLY_TEST_MAIN()
