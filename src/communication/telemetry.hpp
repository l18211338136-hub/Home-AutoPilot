#pragma once

#include <cstdint>

#include "communication/i_link.hpp"
#include "flight_core/types.hpp"

namespace homefly
{

// Provisional internal wire format (little endian host layout). A real
// MAVLink encoder replaces the body of TelemetryReporter::report only.
struct TelemetryFrame
{
	uint32_t magic{0x48464c59u}; // "HFLY"
	uint64_t timestamp_us{0};
	float roll{0.0f};
	float pitch{0.0f};
	float yaw{0.0f};
	float motor[4]{};
	uint8_t mode{0};
	uint8_t failsafe_reason{0};
	uint16_t reserved{0};
};

class TelemetryReporter
{
public:
	explicit TelemetryReporter(
		ILink& link
	) noexcept;

	bool report(
		const VehicleState& state,
		const ControlOutput& output,
		FlightMode mode,
		FailsafeReason reason
	) noexcept;

private:
	ILink& _link;
};

} // namespace homefly
