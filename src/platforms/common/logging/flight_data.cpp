#include "platforms/common/logging/flight_data.hpp"

namespace homefly
{

void FlightDataRecorder::reset() noexcept
{
	_head.store(0, std::memory_order_relaxed);
	_tail.store(0, std::memory_order_relaxed);
	_dropped.store(0, std::memory_order_relaxed);
}

bool FlightDataRecorder::push(
	const FlightDataRecord& record
) noexcept
{
	const uint32_t head = _head.load(std::memory_order_relaxed);
	const uint32_t tail = _tail.load(std::memory_order_acquire);

	if (head - tail >= kFlightDataRingCapacity)
	{
		_dropped.fetch_add(1, std::memory_order_relaxed);
		return false;
	}

	_slots[head % kFlightDataRingCapacity] = record;
	_head.store(head + 1, std::memory_order_release);
	return true;
}

uint32_t FlightDataRecorder::drain_to(
	ILogSink& sink,
	uint32_t max_records
) noexcept
{
	uint32_t processed = 0;

	while (max_records == 0 || processed < max_records)
	{
		const uint32_t tail = _tail.load(std::memory_order_relaxed);
		const uint32_t head = _head.load(std::memory_order_acquire);

		if (tail == head)
		{
			break;
		}

		const FlightDataRecord& record = _slots[tail % kFlightDataRingCapacity];
		const uint8_t* data = reinterpret_cast<const uint8_t*>(&record);

		if (!sink.write(data, static_cast<uint16_t>(sizeof(FlightDataRecord))))
		{
			break; // keep the record for the next drain attempt
		}

		_tail.store(tail + 1, std::memory_order_release);
		++processed;
	}

	return processed;
}

uint32_t FlightDataRecorder::dropped() const noexcept
{
	return _dropped.load(std::memory_order_relaxed);
}

uint32_t FlightDataRecorder::pending() const noexcept
{
	const uint32_t head = _head.load(std::memory_order_acquire);
	const uint32_t tail = _tail.load(std::memory_order_acquire);
	return head - tail;
}

} // namespace homefly
