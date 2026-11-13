#include "configuration/flight_config.hpp"
#include "flight_core/prearm_checker.hpp"
#include "homefly_test.hpp"

using namespace homefly;

namespace
{

FlightConfig valid_config()
{
	return FlightConfig{};
}

PreArmInputs base_inputs()
{
	PreArmInputs in{};
	in.imu_initialized = true;
	in.imu_data_valid = true;
	in.throttle = 0.0f;
	in.config = nullptr; // set by caller
	in.calibration_required = false;
	in.has_battery = false;
	in.has_severe_fault = false;
	return in;
}

} // namespace

TEST(PreArmChecker, PassesWithHealthyState)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	PreArmReport r = checker.check(in);
	EXPECT_TRUE(r.all_passed());
	EXPECT_TRUE(r.first_failure() == PreArmCheck::Count);
}

TEST(PreArmChecker, FailsWhenImuNotInitialized)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	in.imu_initialized = false;
	PreArmReport r = checker.check(in);
	EXPECT_FALSE(r.all_passed());
	EXPECT_TRUE(r.first_failure() == PreArmCheck::ImuInitialized);
}

TEST(PreArmChecker, FailsWhenThrottleNotAtMinimum)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	in.throttle = 0.5f;
	PreArmReport r = checker.check(in);
	EXPECT_FALSE(r.all_passed());
	EXPECT_TRUE(r.first_failure() == PreArmCheck::ThrottleAtMinimum);
}

TEST(PreArmChecker, PassesThrottleExactlyAtMax)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	in.throttle = PreArmChecker::kThrottleArmMax;
	EXPECT_TRUE(checker.check(in).all_passed());
}

TEST(PreArmChecker, FailsWhenMotorConfigInvalid)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	cfg.motor_output_min = 0.9f;
	cfg.motor_output_max = 0.1f; // min > max
	PreArmReport r = checker.check(in);
	EXPECT_FALSE(r.all_passed());
	EXPECT_TRUE(r.first_failure() == PreArmCheck::MotorConfigValid);
}

TEST(PreArmChecker, FailsWhenParamsInvalid)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	cfg.control_period_us = 0; // invalid
	PreArmReport r = checker.check(in);
	EXPECT_FALSE(r.all_passed());
	EXPECT_TRUE(r.first_failure() == PreArmCheck::FlightParamsValid);
}

TEST(PreArmChecker, FailsWhenSevereFaultPresent)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	in.has_severe_fault = true;
	PreArmReport r = checker.check(in);
	EXPECT_FALSE(r.all_passed());
	EXPECT_TRUE(r.first_failure() == PreArmCheck::NoSevereFault);
}

TEST(PreArmChecker, CalibrationRequiredButNotOkFails)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	in.calibration_required = true;
	in.calibration_ok = false;
	EXPECT_FALSE(checker.check(in).all_passed());
}

TEST(PreArmChecker, CalibrationRequiredAndOkPasses)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	in.calibration_required = true;
	in.calibration_ok = true;
	EXPECT_TRUE(checker.check(in).all_passed());
}

TEST(PreArmChecker, BatteryOutOfRangeFails)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	in.has_battery = true;
	in.battery_voltage = 3.0f; // below min
	in.battery_min_v = 10.0f;
	in.battery_max_v = 12.6f;
	EXPECT_FALSE(checker.check(in).all_passed());
}

TEST(PreArmChecker, BatteryInRangePasses)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	FlightConfig cfg = valid_config();
	in.config = &cfg;
	in.has_battery = true;
	in.battery_voltage = 11.4f;
	in.battery_min_v = 10.0f;
	in.battery_max_v = 12.6f;
	EXPECT_TRUE(checker.check(in).all_passed());
}

TEST(PreArmChecker, MissingConfigFails)
{
	PreArmChecker checker;
	PreArmInputs in = base_inputs();
	in.config = nullptr;
	EXPECT_FALSE(checker.check(in).all_passed());
}

HOMEFLY_TEST_MAIN()
