#include "communication/telemetry.hpp"

#include <array>
#include <cstring>

namespace homefly
{

TelemetryReporter::TelemetryReporter(
	ILink& link
) noexcept
	: _link(link)
{
}

bool TelemetryReporter::report(
	const VehicleState& state,
	const ControlOutput& output,
	FlightMode mode,
	FailsafeReason reason
) noexcept
{
	TelemetryFrame frame{};
	frame.timestamp_us = state.timestamp_us;
	frame.roll = state.attitude.roll;
	frame.pitch = state.attitude.pitch;
	frame.yaw = state.attitude.yaw;
	frame.mode = static_cast<uint8_t>(mode);
	frame.failsafe_reason = static_cast<uint8_t>(reason);

	for (uint8_t i = 0; i < 4u && i < output.count; ++i)
	{
		frame.motor[i] = output.motors[i];
	}

	std::array<uint8_t, sizeof(TelemetryFrame)> buffer{};
	std::memcpy(buffer.data(), &frame, sizeof(frame));

	return _link.send(buffer.data(), buffer.size());
}

} // namespace homefly
