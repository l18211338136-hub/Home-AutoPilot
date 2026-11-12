#pragma once

#include <cstdint>

#include "platforms/common/logging/flight_data.hpp"
#include "platforms/common/logging/log_sink.hpp"

namespace homefly
{

// ---------------------------------------------------------------------------
// HomeFly Flight Log (blackbox) v1
//
// A self describing binary file, WRITTEN ONLY BY THE LOW PRIORITY TASK:
//
//   [ FlightLogHeader   64 B ]   magic / version / sizes / session / config
//   [ block 0           N*72 B ]  up to kFlightLogBlockRecords records
//   [ block 1           N*72 B ]
//   ...
//   [ FlightLogFooter   32 B ]   counts, dropped frames, failures, checksum
//
// All multi byte fields are native little endian POD. An offline tool can
// parse the file without any firmware headers: header.record_size and
// header.version tell it everything it needs. See docs/flight_log.md.
// ---------------------------------------------------------------------------

constexpr uint32_t kFlightLogMagic = 0x48464C47u;       // "HFLG"
constexpr uint32_t kFlightLogFooterMagic = 0x48464C45u; // "HFLE"
constexpr uint16_t kFlightLogVersion = 1;
constexpr uint32_t kFlightLogBlockRecords = 16u;
constexpr uint32_t kFlightLogBlockBytes =
	kFlightLogBlockRecords * sizeof(FlightDataRecord);

static_assert(kFlightLogBlockBytes <= 0xFFFFu, "block must fit uint16_t length");

// Fixed 64 bytes. uint64_t members are placed first so the layout has no
// implicit padding on either 32 bit MCUs or 64 bit hosts.
struct FlightLogHeader
{
	uint32_t magic{kFlightLogMagic};
	uint16_t version{kFlightLogVersion};
	uint16_t header_size{static_cast<uint16_t>(sizeof(FlightLogHeader))};
	uint64_t session_id{0};
	uint64_t start_time_us{0};
	uint32_t record_size{static_cast<uint32_t>(sizeof(FlightDataRecord))};
	uint32_t control_period_us{0};
	uint32_t config_hash{0};
	char platform[16]{};
	uint32_t reserved[3]{};
};

static_assert(sizeof(FlightLogHeader) == 64u, "flight log header must be 64 bytes");

// Fixed 32 bytes, written by finish(). A missing/invalid footer marks an
// abruptly terminated (crash / power loss) session for the offline tool.
struct FlightLogFooter
{
	uint32_t magic{kFlightLogFooterMagic};
	uint32_t record_count{0};
	uint64_t end_time_us{0};
	uint32_t dropped_records{0};
	uint32_t write_failures{0};
	uint32_t checksum{0};
	uint32_t reserved{0};
};

static_assert(sizeof(FlightLogFooter) == 32u, "flight log footer must be 32 bytes");

struct FlightLogStats
{
	uint32_t records_written{0};
	uint32_t blocks_written{0};
	uint32_t write_failures{0};
	uint32_t dropped_records{0};
	uint32_t checksum{0};
	uint64_t end_time_us{0};
};

// Batches structured records into blocks before handing them to a storage
// sink. It implements ILogSink so FlightDataRecorder::drain_to(*writer) can
// feed it directly -- a failed storage write makes drain_to() keep the record
// for the next attempt, which gives retry semantics for free.
//
// Two buffering stages, no I/O in the control loop:
//   control loop -> FlightDataRecorder SPSC ring (128 slots)
//   low priority -> FlightLogWriter block (16 records) -> ILogSink (file/SD)
class FlightLogWriter final : public ILogSink
{
public:
	// Writes the header. Call once, from the low priority task, before the
	// first pump(). Returns false if the storage rejected the header.
	bool begin(
		ILogSink& storage,
		const FlightLogHeader& header
	) noexcept;

	// Moves up to max_records entries from the ring into blocks and writes
	// full blocks to storage. Returns how many records were consumed.
	uint32_t pump(
		FlightDataRecorder& recorder,
		uint32_t max_records = 0
	) noexcept;

	// Flushes the trailing partial block, writes the footer and flushes the
	// storage. Returns false if any final write failed.
	bool finish(
		uint32_t dropped_records
	) noexcept;

	const FlightLogStats& stats() const noexcept
	{
		return _stats;
	}

	bool active() const noexcept
	{
		return _active;
	}

	// ILogSink: called by FlightDataRecorder::drain_to(), never by firmware
	// modules directly.
	bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept override;

	void flush() noexcept override;

private:
	bool commit_block() noexcept;

	ILogSink* _storage{nullptr};
	FlightLogStats _stats{};
	uint32_t _checksum{2166136261u}; // FNV-1a offset basis
	uint32_t _block_records{0};
	bool _active{false};
	uint8_t _block[kFlightLogBlockBytes]{};
};

uint32_t flight_log_checksum(
	const uint8_t* data,
	uint32_t length,
	uint32_t seed
) noexcept;

} // namespace homefly
