#include "platforms/common/logging/flight_log.hpp"

#include <cstddef>
#include <cstring>

namespace homefly
{

uint32_t flight_log_checksum(
	const uint8_t* data,
	uint32_t length,
	uint32_t seed
) noexcept
{
	uint32_t hash = seed;

	for (uint32_t i = 0; i < length; ++i)
	{
		hash = (hash ^ data[i]) * 16777619u;
	}

	return hash;
}

bool FlightLogWriter::begin(
	ILogSink& storage,
	const FlightLogHeader& header
) noexcept
{
	_storage = &storage;
	_stats = FlightLogStats{};
	_checksum = 2166136261u;
	_block_records = 0;
	_active = false;

	// Never trust the caller for the fields the format depends on.
	FlightLogHeader normalized = header;
	normalized.magic = kFlightLogMagic;
	normalized.version = kFlightLogVersion;
	normalized.header_size = static_cast<uint16_t>(sizeof(FlightLogHeader));
	normalized.record_size = static_cast<uint32_t>(sizeof(FlightDataRecord));

	if (!storage.write(reinterpret_cast<const uint8_t*>(&normalized),
		static_cast<uint16_t>(sizeof(normalized))))
	{
		++_stats.write_failures;
		return false;
	}

	_active = true;
	return true;
}

bool FlightLogWriter::write(
	const uint8_t* data,
	uint16_t length
) noexcept
{
	if (!_active || _storage == nullptr || data == nullptr ||
		length != sizeof(FlightDataRecord))
	{
		return false;
	}

	const uint32_t previous_checksum = _checksum;
	const uint64_t previous_end_time = _stats.end_time_us;
	const uint32_t offset = _block_records * static_cast<uint32_t>(sizeof(FlightDataRecord));
	std::memcpy(_block + offset, data, length);

	_checksum = flight_log_checksum(data, length, _checksum);
	++_block_records;
	++_stats.records_written;

	uint64_t timestamp_us = 0;
	std::memcpy(&timestamp_us, data + offsetof(FlightDataRecord, timestamp_us),
		sizeof(timestamp_us));
	_stats.end_time_us = timestamp_us;

	if (_block_records == kFlightLogBlockRecords && !commit_block())
	{
		// Roll back so FlightDataRecorder::drain_to() keeps this record and
		// the next pump() can retry instead of silently losing a block.
		--_block_records;
		--_stats.records_written;
		_checksum = previous_checksum;
		_stats.end_time_us = previous_end_time;
		++_stats.write_failures;
		return false;
	}

	return true;
}

uint32_t FlightLogWriter::pump(
	FlightDataRecorder& recorder,
	uint32_t max_records
) noexcept
{
	if (!_active)
	{
		return 0;
	}

	return recorder.drain_to(*this, max_records);
}

bool FlightLogWriter::finish(
	uint32_t dropped_records
) noexcept
{
	if (!_active || _storage == nullptr)
	{
		return false;
	}

	bool ok = true;
	_stats.dropped_records = dropped_records;
	_stats.checksum = _checksum;

	if (!commit_block())
	{
		++_stats.write_failures;
		ok = false;
	}

	FlightLogFooter footer{};
	footer.record_count = _stats.records_written;
	footer.end_time_us = _stats.end_time_us;
	footer.dropped_records = _stats.dropped_records;
	footer.write_failures = _stats.write_failures;
	footer.checksum = _checksum;

	if (!_storage->write(reinterpret_cast<const uint8_t*>(&footer),
		static_cast<uint16_t>(sizeof(footer))))
	{
		++_stats.write_failures;
		ok = false;
	}

	_storage->flush();
	_active = false;
	return ok;
}

void FlightLogWriter::flush() noexcept
{
	if (_storage != nullptr)
	{
		_storage->flush();
	}
}

bool FlightLogWriter::commit_block() noexcept
{
	if (_block_records == 0)
	{
		return true;
	}

	const uint16_t bytes = static_cast<uint16_t>(
		_block_records * static_cast<uint32_t>(sizeof(FlightDataRecord)));

	if (!_storage->write(_block, bytes))
	{
		return false;
	}

	++_stats.blocks_written;
	_block_records = 0;
	return true;
}

} // namespace homefly
