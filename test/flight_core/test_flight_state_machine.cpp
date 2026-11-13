#include "flight_core/flight_state_machine.hpp"
#include "homefly_test.hpp"

using namespace homefly;

TEST(FlightStateMachine, StartsDisarmed)
{
	FlightStateMachine fsm;
	EXPECT_TRUE(fsm.mode() == FlightMode::Disarmed);
	EXPECT_TRUE(fsm.failsafe_reason() == FailsafeReason::None);
}

TEST(FlightStateMachine, ArmRequiresValidSensors)
{
	FlightStateMachine fsm;
	EXPECT_TRUE(fsm.update(StateEvent::ArmRequest, false) == FlightMode::Failsafe);
	EXPECT_TRUE(fsm.failsafe_reason() == FailsafeReason::ImuTimeout);
}

TEST(FlightStateMachine, ArmAndDisarm)
{
	FlightStateMachine fsm;
	EXPECT_TRUE(fsm.update(StateEvent::ArmRequest, true) == FlightMode::Stabilize);
	EXPECT_TRUE(fsm.update(StateEvent::None, true) == FlightMode::Stabilize);
	EXPECT_TRUE(fsm.update(StateEvent::DisarmRequest, true) == FlightMode::Disarmed);
}

TEST(FlightStateMachine, SensorFaultIsSticky)
{
	FlightStateMachine fsm;
	fsm.update(StateEvent::ArmRequest, true);
	EXPECT_TRUE(fsm.update(StateEvent::SensorFault, false, true, FailsafeReason::ImuReadFault)
		== FlightMode::Failsafe);
	EXPECT_TRUE(fsm.failsafe_reason() == FailsafeReason::ImuReadFault);
	EXPECT_TRUE(fsm.update(StateEvent::None, true) == FlightMode::Failsafe);
	EXPECT_TRUE(fsm.update(StateEvent::ArmRequest, true) == FlightMode::Failsafe);
}

TEST(FlightStateMachine, RecoveryNeedsExplicitDisarmAndHealthySensors)
{
	FlightStateMachine fsm;
	fsm.update(StateEvent::ArmRequest, true);
	fsm.update(StateEvent::SensorFault, false, true, FailsafeReason::MotorFault);
	EXPECT_TRUE(fsm.update(StateEvent::DisarmRequest, false) == FlightMode::Failsafe);
	EXPECT_TRUE(fsm.update(StateEvent::DisarmRequest, true) == FlightMode::Disarmed);
	EXPECT_TRUE(fsm.failsafe_reason() == FailsafeReason::None);
}

TEST(FlightStateMachine, InvalidSensorsInFlightForceFailsafe)
{
	FlightStateMachine fsm;
	fsm.update(StateEvent::ArmRequest, true);
	EXPECT_TRUE(fsm.update(StateEvent::None, false) == FlightMode::Failsafe);
	EXPECT_TRUE(fsm.failsafe_reason() == FailsafeReason::ImuTimeout);
}

TEST(FlightStateMachine, ArmBlockedWhenPreArmFails)
{
	FlightStateMachine fsm;
	// sensors valid but pre-arm checks failed -> must not arm.
	EXPECT_TRUE(fsm.update(StateEvent::ArmRequest, true, false) == FlightMode::Failsafe);
	EXPECT_TRUE(fsm.failsafe_reason() == FailsafeReason::PreArmFailed);
}

TEST(FlightStateMachine, ArmSucceedsWhenPreArmPasses)
{
	FlightStateMachine fsm;
	EXPECT_TRUE(fsm.update(StateEvent::ArmRequest, true, true) == FlightMode::Stabilize);
}

TEST(FlightStateMachine, ArmBlockedWhenSensorsAndPreArmBothFail)
{
	FlightStateMachine fsm;
	// sensor fault wins the reason attribution over PreArmFailed.
	EXPECT_TRUE(fsm.update(StateEvent::ArmRequest, false, false) == FlightMode::Failsafe);
	EXPECT_TRUE(fsm.failsafe_reason() == FailsafeReason::ImuTimeout);
}

HOMEFLY_TEST_MAIN()
