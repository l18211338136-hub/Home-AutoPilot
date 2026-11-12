#include <cstddef>

#include "communication/telemetry.hpp"
#include "homefly_test.hpp"
#include "simulation/sim_link.hpp"

using namespace homefly;

TEST(TelemetryReporter, SendsOneFramePerReport)
{
	SimLink link;
	TelemetryReporter reporter(link);
	VehicleState state{};
	state.timestamp_us = 42000u;
	state.attitude.roll = 0.1f;
	ControlOutput output{};
	output.motors[0] = 0.25f;

	EXPECT_TRUE(reporter.report(state, output, FlightMode::Stabilize, FailsafeReason::None));
	EXPECT_EQ(link.sent_frames(), static_cast<std::size_t>(1));
	EXPECT_EQ(link.last_length(), sizeof(TelemetryFrame));
}

TEST(TelemetryReporter, RejectsEmptyPayloadLink)
{
	SimLink link;
	TelemetryReporter reporter(link);
	// The loopback link rejects zero length sends, so a frame must be emitted.
	VehicleState state{};
	ControlOutput output{};
	EXPECT_TRUE(reporter.report(state, output, FlightMode::Disarmed, FailsafeReason::None));
	EXPECT_TRUE(link.last_length() > 0u);
}

HOMEFLY_TEST_MAIN()
