#pragma once

#include <atomic>
#include <cstdint>

#include "platforms/common/logging/log_sink.hpp"

namespace homefly
{

constexpr uint32_t kFlightDataMagic = 0x48464644u; // "HFFD"
constexpr uint8_t kFlightDataVersion = 1;
constexpr uint32_t kFlightDataRingCapacity = 128;  // power of two

// Structured flight data is a SEPARATE channel from text logging.
// IMU samples, attitude angles and motor outputs must never be printed with
// HF_LOG_*: string formatting at 500 Hz-1 kHz would dominate the CPU budget
// and fill the log ring. This POD is recorded as raw binary instead.
//
// Attitude in rad, gyro in rad/s, accel in m/s^2, motor normalized [0, 1].
// mode maps to flight_core::FlightMode (kept as uint8_t to avoid a dependency
// from platforms/common onto FlightCore).
struct FlightDataRecord
{
	uint32_t magic{kFlightDataMagic};
	uint8_t version{kFlightDataVersion};
	uint8_t mode{0};
	uint16_t reserved{0};
	uint64_t timestamp_us{0};
	float roll{0.0f};
	float pitch{0.0f};
	float yaw{0.0f};
	float gyro[3]{};
	float accel[3]{};
	float motor[4]{};
};

static_assert(sizeof(FlightDataRecord) % 4 == 0, "flight data record must be word aligned");

// Single producer (control loop) / single consumer (idle task) ring.
// push() is allocation free, non blocking and bounded.
class FlightDataRecorder
{
public:
	void reset() noexcept;

	bool push(
		const FlightDataRecord& record
	) noexcept;

	// Writes raw records to a binary sink (SD / Flash / telemetry).
	// Returns the number of records written.
	uint32_t drain_to(
		ILogSink& sink,
		uint32_t max_records = 0
	) noexcept;

	uint32_t dropped() const noexcept;

	uint32_t pending() const noexcept;

private:
	FlightDataRecord _slots[kFlightDataRingCapacity]{};
	std::atomic<uint32_t> _head{0};
	std::atomic<uint32_t> _tail{0};
	std::atomic<uint32_t> _dropped{0};
};

} // namespace homefly
