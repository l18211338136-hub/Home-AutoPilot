#include <cstdint>
#include <cstring>

#include "homefly_test.hpp"
#include "platforms/common/logging/flight_data.hpp"

using namespace homefly;

namespace
{

class CapturingSink final : public ILogSink
{
public:
	bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept override
	{
		if (_fault || data == nullptr || length != sizeof(FlightDataRecord) || _count >= kMaxRecords)
		{
			return false;
		}

		std::memcpy(&_records[_count], data, sizeof(FlightDataRecord));
		++_count;
		return true;
	}

	void set_fault(
		bool fault
	) noexcept
	{
		_fault = fault;
	}

	uint32_t count() const noexcept
	{
		return _count;
	}

	const FlightDataRecord& record(
		uint32_t index
	) const noexcept
	{
		return _records[index];
	}

private:
	static constexpr uint32_t kMaxRecords = 256u;
	FlightDataRecord _records[kMaxRecords]{};
	uint32_t _count{0};
	bool _fault{false};
};

FlightDataRecord make_record(
	uint64_t timestamp_us
)
{
	FlightDataRecord record{};
	record.timestamp_us = timestamp_us;
	record.roll = 0.1f;
	record.pitch = -0.2f;
	record.yaw = 0.3f;
	record.gyro[0] = 0.01f;
	record.motor[0] = 0.5f;
	return record;
}

} // namespace

TEST(FlightData, RecordIsBinaryPacked)
{
	FlightDataRecord record{};
	EXPECT_EQ(record.magic, kFlightDataMagic);
	EXPECT_EQ(static_cast<unsigned>(record.version), static_cast<unsigned>(kFlightDataVersion));
	EXPECT_TRUE(sizeof(FlightDataRecord) % 4u == 0u);
}

TEST(FlightData, PreservesOrderAndFields)
{
	FlightDataRecorder recorder;
	CapturingSink sink;

	for (uint32_t i = 0; i < 3u; ++i)
	{
		EXPECT_TRUE(recorder.push(make_record(1000u + i)));
	}

	EXPECT_EQ(recorder.pending(), 3u);
	EXPECT_EQ(recorder.drain_to(sink), 3u);
	EXPECT_EQ(recorder.pending(), 0u);
	EXPECT_EQ(sink.count(), 3u);
	EXPECT_EQ(sink.record(0).timestamp_us, 1000u);
	EXPECT_EQ(sink.record(2).timestamp_us, 1002u);
	EXPECT_NEAR(sink.record(1).roll, 0.1f, 1.0e-6f);
	EXPECT_NEAR(sink.record(1).motor[0], 0.5f, 1.0e-6f);
}

TEST(FlightData, OverflowDropsInsteadOfBlocking)
{
	FlightDataRecorder recorder;

	for (uint32_t i = 0; i < kFlightDataRingCapacity + 10u; ++i)
	{
		recorder.push(make_record(i));
	}

	EXPECT_EQ(recorder.pending(), kFlightDataRingCapacity);
	EXPECT_EQ(recorder.dropped(), 10u);
}

TEST(FlightData, DrainLimit)
{
	FlightDataRecorder recorder;
	CapturingSink sink;

	for (uint32_t i = 0; i < 5u; ++i)
	{
		recorder.push(make_record(i));
	}

	EXPECT_EQ(recorder.drain_to(sink, 2u), 2u);
	EXPECT_EQ(recorder.pending(), 3u);
	EXPECT_EQ(recorder.drain_to(sink), 3u);
	EXPECT_EQ(sink.count(), 5u);
}

TEST(FlightData, SinkFailureKeepsRecordForRetry)
{
	FlightDataRecorder recorder;
	CapturingSink sink;
	sink.set_fault(true);
	recorder.push(make_record(42u));

	EXPECT_EQ(recorder.drain_to(sink), 0u);
	EXPECT_EQ(recorder.pending(), 1u);

	sink.set_fault(false);
	EXPECT_EQ(recorder.drain_to(sink), 1u);
	EXPECT_EQ(sink.count(), 1u);
	EXPECT_EQ(sink.record(0).timestamp_us, 42u);
}

TEST(FlightData, ResetClearsRingAndCounters)
{
	FlightDataRecorder recorder;

	for (uint32_t i = 0; i < kFlightDataRingCapacity + 1u; ++i)
	{
		recorder.push(make_record(i));
	}

	recorder.reset();
	EXPECT_EQ(recorder.pending(), 0u);
	EXPECT_EQ(recorder.dropped(), 0u);
}

HOMEFLY_TEST_MAIN()
