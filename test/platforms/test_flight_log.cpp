#include <cstdint>
#include <cstring>

#include "homefly_test.hpp"
#include "platforms/common/logging/flight_log.hpp"

using namespace homefly;

namespace
{

// Captures the whole "file" in memory so the tests can parse the exact bytes
// an offline tool would see.
class CapturingFileSink final : public ILogSink
{
public:
	bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept override
	{
		if (_fault || data == nullptr || length == 0 ||
			static_cast<uint32_t>(_size + length) > sizeof(_buffer))
		{
			return false;
		}

		std::memcpy(_buffer + _size, data, length);
		_size += length;
		++_writes;
		return true;
	}

	void flush() noexcept override
	{
		++_flushes;
	}

	void set_fault(bool fault) noexcept
	{
		_fault = fault;
	}

	uint32_t size() const noexcept
	{
		return _size;
	}

	uint32_t writes() const noexcept
	{
		return _writes;
	}

	uint32_t flushes() const noexcept
	{
		return _flushes;
	}

	const uint8_t* data() const noexcept
	{
		return _buffer;
	}

private:
	uint8_t _buffer[16384]{};
	uint32_t _size{0};
	uint32_t _writes{0};
	uint32_t _flushes{0};
	bool _fault{false};
};

FlightDataRecord make_record(
	uint64_t timestamp_us
)
{
	FlightDataRecord record{};
	record.timestamp_us = timestamp_us;
	record.mode = 1u;
	record.roll = 0.25f;
	record.pitch = -0.125f;
	record.motor[0] = 0.6f;
	return record;
}

FlightLogHeader make_header()
{
	FlightLogHeader header{};
	header.session_id = 0x123456789ABCDEF0ull;
	header.control_period_us = 1000u;
	header.config_hash = 0xDEADBEEFu;
	std::memcpy(header.platform, "test_board", sizeof("test_board"));
	return header;
}

template<typename T>
T read_at(
	const CapturingFileSink& sink,
	uint32_t offset
)
{
	T value{};
	std::memcpy(&value, sink.data() + offset, sizeof(T));
	return value;
}

} // namespace

TEST(FlightLog, HeaderIsSelfDescribing)
{
	CapturingFileSink sink;
	FlightLogWriter writer;

	EXPECT_TRUE(writer.begin(sink, make_header()));
	EXPECT_EQ(sink.size(), static_cast<uint32_t>(sizeof(FlightLogHeader)));
	EXPECT_TRUE(writer.active());

	const FlightLogHeader header = read_at<FlightLogHeader>(sink, 0);
	EXPECT_EQ(header.magic, kFlightLogMagic);
	EXPECT_EQ(header.version, kFlightLogVersion);
	EXPECT_EQ(header.header_size, static_cast<uint16_t>(sizeof(FlightLogHeader)));
	EXPECT_EQ(header.record_size, static_cast<uint32_t>(sizeof(FlightDataRecord)));
	EXPECT_EQ(header.session_id, 0x123456789ABCDEF0ull);
	EXPECT_EQ(header.control_period_us, 1000u);
	EXPECT_EQ(header.config_hash, 0xDEADBEEFu);
	EXPECT_EQ(std::strcmp(header.platform, "test_board"), 0);
}

TEST(FlightLog, BeginForcesFormatFields)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	FlightLogHeader header = make_header();
	header.magic = 0u;
	header.version = 99u;
	header.record_size = 1u;

	EXPECT_TRUE(writer.begin(sink, header));
	const FlightLogHeader stored = read_at<FlightLogHeader>(sink, 0);
	EXPECT_EQ(stored.magic, kFlightLogMagic);
	EXPECT_EQ(stored.version, kFlightLogVersion);
	EXPECT_EQ(stored.record_size, static_cast<uint32_t>(sizeof(FlightDataRecord)));
}

TEST(FlightLog, BatchesRecordsIntoBlocks)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	FlightDataRecorder recorder;
	EXPECT_TRUE(writer.begin(sink, make_header()));

	for (uint32_t i = 0; i < kFlightLogBlockRecords * 2u; ++i)
	{
		EXPECT_TRUE(recorder.push(make_record(1000u + i)));
	}

	EXPECT_EQ(writer.pump(recorder), kFlightLogBlockRecords * 2u);
	EXPECT_EQ(recorder.pending(), 0u);
	EXPECT_EQ(writer.stats().records_written, kFlightLogBlockRecords * 2u);
	EXPECT_EQ(writer.stats().blocks_written, 2u);
	// 1 header write + 2 block writes, NOT 32 individual writes.
	EXPECT_EQ(sink.writes(), 3u);
}

TEST(FlightLog, FinishFlushesPartialBlockAndWritesFooter)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	FlightDataRecorder recorder;
	EXPECT_TRUE(writer.begin(sink, make_header()));

	for (uint32_t i = 0; i < 5u; ++i)
	{
		recorder.push(make_record(1000u + i));
	}

	EXPECT_EQ(writer.pump(recorder), 5u);
	EXPECT_EQ(sink.writes(), 1u); // header only, no full block yet
	EXPECT_TRUE(writer.finish(3u));
	EXPECT_TRUE(!writer.active());
	EXPECT_EQ(sink.writes(), 3u); // header + partial block + footer
	EXPECT_EQ(sink.flushes(), 1u);

	const uint32_t expected_size = static_cast<uint32_t>(
		sizeof(FlightLogHeader) + 5u * sizeof(FlightDataRecord) + sizeof(FlightLogFooter));
	EXPECT_EQ(sink.size(), expected_size);

	const FlightLogFooter footer = read_at<FlightLogFooter>(
		sink, expected_size - static_cast<uint32_t>(sizeof(FlightLogFooter)));
	EXPECT_EQ(footer.magic, kFlightLogFooterMagic);
	EXPECT_EQ(footer.record_count, 5u);
	EXPECT_EQ(footer.dropped_records, 3u);
	EXPECT_EQ(footer.write_failures, 0u);
	EXPECT_EQ(footer.end_time_us, 1004u);
}

TEST(FlightLog, ChecksumCoversEveryRecordByte)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	FlightDataRecorder recorder;
	EXPECT_TRUE(writer.begin(sink, make_header()));

	constexpr uint32_t kCount = 20u;
	for (uint32_t i = 0; i < kCount; ++i)
	{
		recorder.push(make_record(1000u + i));
	}

	writer.pump(recorder);
	EXPECT_TRUE(writer.finish(0u));

	// Independently recompute FNV-1a over the bytes actually written.
	uint32_t expected = 2166136261u;
	uint32_t offset = static_cast<uint32_t>(sizeof(FlightLogHeader));

	for (uint32_t i = 0; i < kCount; ++i)
	{
		expected = flight_log_checksum(sink.data() + offset,
			static_cast<uint32_t>(sizeof(FlightDataRecord)), expected);
		offset += static_cast<uint32_t>(sizeof(FlightDataRecord));
	}

	const FlightLogFooter footer = read_at<FlightLogFooter>(sink, offset);
	EXPECT_EQ(footer.checksum, expected);
	EXPECT_EQ(writer.stats().checksum, expected);
}

TEST(FlightLog, WriteFailureKeepsRecordForRetry)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	FlightDataRecorder recorder;
	EXPECT_TRUE(writer.begin(sink, make_header()));

	for (uint32_t i = 0; i < kFlightLogBlockRecords; ++i)
	{
		recorder.push(make_record(1000u + i));
	}

	sink.set_fault(true);
	EXPECT_EQ(writer.pump(recorder), kFlightLogBlockRecords - 1u);
	// The 16th record triggered a failed commit and must stay in the ring.
	EXPECT_EQ(recorder.pending(), 1u);
	EXPECT_EQ(writer.stats().write_failures, 1u);
	EXPECT_EQ(writer.stats().blocks_written, 0u);

	sink.set_fault(false);
	EXPECT_EQ(writer.pump(recorder), 1u);
	EXPECT_EQ(recorder.pending(), 0u);
	EXPECT_EQ(writer.stats().blocks_written, 1u);
	EXPECT_EQ(writer.stats().records_written, kFlightLogBlockRecords);
}

TEST(FlightLog, HeaderFailureDisablesWriter)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	sink.set_fault(true);

	EXPECT_TRUE(!writer.begin(sink, make_header()));
	EXPECT_TRUE(!writer.active());
	EXPECT_EQ(writer.stats().write_failures, 1u);
	EXPECT_TRUE(!writer.finish(0u));
}

TEST(FlightLog, InactiveWriterRejectsEverything)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	FlightDataRecorder recorder;
	recorder.push(make_record(42u));

	EXPECT_EQ(writer.pump(recorder), 0u);
	EXPECT_TRUE(!writer.write(reinterpret_cast<const uint8_t*>(&recorder), 1u));
	EXPECT_TRUE(!writer.finish(0u));
	EXPECT_TRUE(!writer.active());
}

TEST(FlightLog, RejectsWrongRecordLength)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	EXPECT_TRUE(writer.begin(sink, make_header()));

	FlightDataRecord record = make_record(7u);
	const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&record);
	EXPECT_TRUE(!writer.write(bytes, static_cast<uint16_t>(sizeof(FlightDataRecord) - 1u)));
	EXPECT_TRUE(writer.write(bytes, static_cast<uint16_t>(sizeof(FlightDataRecord))));
	EXPECT_EQ(writer.stats().records_written, 1u);
}

TEST(FlightLog, FooterReportsWriteFailures)
{
	CapturingFileSink sink;
	FlightLogWriter writer;
	FlightDataRecorder recorder;
	EXPECT_TRUE(writer.begin(sink, make_header()));
	recorder.push(make_record(1u));
	writer.pump(recorder);

	// Both the trailing partial block AND the footer write fail.
	sink.set_fault(true);
	EXPECT_TRUE(!writer.finish(0u));
	EXPECT_EQ(writer.stats().write_failures, 2u);
}

HOMEFLY_TEST_MAIN()
