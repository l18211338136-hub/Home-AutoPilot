#include "platforms/common/logging/log.hpp"

#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace homefly
{

const char* log_level_label(
	LogLevel level
) noexcept
{
	switch (level)
	{
	case LogLevel::Debug:
		return "DEBUG";
	case LogLevel::Info:
		return "INFO ";
	case LogLevel::Warn:
		return "WARN ";
	case LogLevel::Error:
		return "ERROR";
	case LogLevel::Fatal:
		return "FATAL";
	default:
		return "?????";
	}
}

const char* log_level_name(
	LogLevel level
) noexcept
{
	switch (level)
	{
	case LogLevel::Debug:
		return "DEBUG";
	case LogLevel::Info:
		return "INFO";
	case LogLevel::Warn:
		return "WARN";
	case LogLevel::Error:
		return "ERROR";
	case LogLevel::Fatal:
		return "FATAL";
	default:
		return "UNKNOWN";
	}
}

namespace
{

struct LogSlot
{
	std::atomic<uint32_t> sequence{0};
	uint16_t length{0};
	char line[kLogLineCapacity]{};
};

// Static storage: no dynamic allocation anywhere in the logging path.
// Vyukov bounded MPMC queue: multiple tasks may log concurrently, the low
// priority drain task consumes. All indices are lock free on 32 bit MCUs.
LogSlot s_slots[kLogRingCapacity];
std::atomic<uint32_t> s_head{0};
std::atomic<uint32_t> s_tail{0};

IClock* s_clock{nullptr};
ILogSink* s_sinks[kLogSinkCapacity]{};
uint8_t s_sink_count{0};

std::atomic<LogLevel> s_min_level{LogLevel::Info};
std::atomic<uint32_t> s_rate_limit_ms{0};
std::atomic<uint32_t> s_rate_last[kLogRateSlots]{};

std::atomic<uint32_t> s_stat_emitted{0};
std::atomic<uint32_t> s_stat_drained{0};
std::atomic<uint32_t> s_stat_dropped_level{0};
std::atomic<uint32_t> s_stat_dropped_rate{0};
std::atomic<uint32_t> s_stat_dropped_full{0};
std::atomic<uint32_t> s_stat_backend_failures{0};
std::atomic<uint32_t> s_stat_bytes_written{0};

// FNV-1a over the tag pointer content. Collisions only make throttling
// slightly more conservative, never unsafe.
uint8_t rate_slot(
	const char* tag,
	LogLevel level
) noexcept
{
	uint32_t hash = 2166136261u;

	if (tag != nullptr)
	{
		for (const char* p = tag; *p != '\0'; ++p)
		{
			hash = (hash ^ static_cast<uint8_t>(*p)) * 16777619u;
		}
	}

	hash ^= static_cast<uint32_t>(level) * 0x9e3779b9u;
	return static_cast<uint8_t>(hash % kLogRateSlots);
}

void reset_stats() noexcept
{
	s_stat_emitted.store(0, std::memory_order_relaxed);
	s_stat_drained.store(0, std::memory_order_relaxed);
	s_stat_dropped_level.store(0, std::memory_order_relaxed);
	s_stat_dropped_rate.store(0, std::memory_order_relaxed);
	s_stat_dropped_full.store(0, std::memory_order_relaxed);
	s_stat_backend_failures.store(0, std::memory_order_relaxed);
	s_stat_bytes_written.store(0, std::memory_order_relaxed);
}

} // namespace

void log_init(
	IClock& clock,
	ILogSink& console,
	const LogConfig& config
) noexcept
{
	// Single threaded startup only. Do not call while producers are running.
	s_head.store(0, std::memory_order_relaxed);
	s_tail.store(0, std::memory_order_relaxed);

	for (uint32_t i = 0; i < kLogRingCapacity; ++i)
	{
		s_slots[i].sequence.store(i, std::memory_order_relaxed);
		s_slots[i].length = 0;
		s_slots[i].line[0] = '\0';
	}

	for (uint8_t i = 0; i < kLogRateSlots; ++i)
	{
		s_rate_last[i].store(0, std::memory_order_relaxed);
	}

	s_sink_count = 0;
	s_sinks[s_sink_count++] = &console;
	s_clock = &clock;
	s_min_level.store(config.min_level, std::memory_order_relaxed);
	s_rate_limit_ms.store(config.rate_limit_ms, std::memory_order_relaxed);
	reset_stats();
}

void log_shutdown() noexcept
{
	// Producers must already be stopped (single-threaded shutdown only).
	// Drain pending records before the sinks disappear, so a caller that
	// forgets the final log_drain() still does not lose the tail of the log.
	log_drain();
	log_flush();
	s_sink_count = 0;
	s_clock = nullptr;
}

bool log_add_sink(
	ILogSink& sink
) noexcept
{
	// Registration happens during startup, before any drain task runs.
	if (s_sink_count >= kLogSinkCapacity)
	{
		return false;
	}

	s_sinks[s_sink_count++] = &sink;
	return true;
}

void log_set_level(
	LogLevel level
) noexcept
{
	s_min_level.store(level, std::memory_order_relaxed);
}

LogLevel log_get_level() noexcept
{
	return s_min_level.load(std::memory_order_relaxed);
}

void log_write(
	LogLevel level,
	const char* tag,
	const char* format,
	...
) noexcept
{
	if (s_clock == nullptr)
	{
		return;
	}

	// 1. Runtime level filter.
	if (static_cast<uint8_t>(level) < static_cast<uint8_t>(s_min_level.load(std::memory_order_relaxed)))
	{
		s_stat_dropped_level.fetch_add(1, std::memory_order_relaxed);
		return;
	}

	const uint32_t now_ms = static_cast<uint32_t>(s_clock->now_us() / 1000u);

	// 2. High frequency log throttling, per (tag, level) bucket.
	const uint32_t interval_ms = s_rate_limit_ms.load(std::memory_order_relaxed);

	if (interval_ms > 0)
	{
		const uint8_t index = rate_slot(tag, level);
		const uint32_t last_ms = s_rate_last[index].load(std::memory_order_relaxed);

		if (last_ms != 0 && (now_ms - last_ms) < interval_ms)
		{
			s_stat_dropped_rate.fetch_add(1, std::memory_order_relaxed);
			return;
		}

		s_rate_last[index].store(now_ms, std::memory_order_relaxed);
	}

	// 3. Reserve a ring slot. Never waits: a full ring drops the record.
	uint32_t position = s_head.load(std::memory_order_relaxed);
	LogSlot* slot = nullptr;

	for (;;)
	{
		slot = &s_slots[position % kLogRingCapacity];
		const uint32_t sequence = slot->sequence.load(std::memory_order_acquire);
		const int32_t difference = static_cast<int32_t>(sequence - position);

		if (difference == 0)
		{
			if (s_head.compare_exchange_weak(position, position + 1, std::memory_order_relaxed))
			{
				break;
			}
		}
		else if (difference < 0)
		{
			s_stat_dropped_full.fetch_add(1, std::memory_order_relaxed);
			return;
		}
		else
		{
			position = s_head.load(std::memory_order_relaxed);
		}
	}

	// 4. Bounded formatting into the slot. vsnprintf is pure CPU work with a
	//    hard upper bound; the slow serial write happens in log_drain().
	char* const line = slot->line;
	const unsigned long timestamp = static_cast<unsigned long>(now_ms);
	const int prefix = std::snprintf(line, kLogLineCapacity, "[%08lu] [%s] [%.12s] ",
		timestamp, log_level_label(level), tag != nullptr ? tag : "NONE");

	uint16_t length = 0;

	if (prefix > 0)
	{
		size_t used = static_cast<size_t>(prefix);

		if (used > kLogLineCapacity - 1)
		{
			used = kLogLineCapacity - 1;
		}

		va_list args;
		va_start(args, format);
		const int body = std::vsnprintf(line + used, kLogLineCapacity - used,
			format != nullptr ? format : "", args);
		va_end(args);

		size_t body_length = body > 0 ? static_cast<size_t>(body) : 0;
		const size_t available = kLogLineCapacity - used - 1;

		if (body_length > available)
		{
			body_length = available;
		}

		length = static_cast<uint16_t>(used + body_length);
	}

	line[length] = '\0';
	slot->length = length;

	// 5. Publish. The consumer may only see the record after this store.
	slot->sequence.store(position + 1, std::memory_order_release);
	s_stat_emitted.fetch_add(1, std::memory_order_relaxed);
}

uint32_t log_drain(
	uint32_t max_records
) noexcept
{
	uint32_t processed = 0;

	while (max_records == 0 || processed < max_records)
	{
		uint32_t position = s_tail.load(std::memory_order_relaxed);
		LogSlot* slot = nullptr;

		for (;;)
		{
			slot = &s_slots[position % kLogRingCapacity];
			const uint32_t sequence = slot->sequence.load(std::memory_order_acquire);
			const int32_t difference = static_cast<int32_t>(sequence - (position + 1));

			if (difference == 0)
			{
				if (s_tail.compare_exchange_weak(position, position + 1, std::memory_order_relaxed))
				{
					break;
				}
			}
			else if (difference < 0)
			{
				return processed; // ring empty
			}
			else
			{
				position = s_tail.load(std::memory_order_relaxed);
			}
		}

		const uint8_t* data = reinterpret_cast<const uint8_t*>(slot->line);

		for (uint8_t i = 0; i < s_sink_count; ++i)
		{
			if (s_sinks[i]->write(data, slot->length))
			{
				s_stat_bytes_written.fetch_add(slot->length, std::memory_order_relaxed);
			}
			else
			{
				s_stat_backend_failures.fetch_add(1, std::memory_order_relaxed);
			}
		}

		// Mark the slot writable again for the next wrap around.
		slot->sequence.store(position + kLogRingCapacity, std::memory_order_release);
		++processed;
		s_stat_drained.fetch_add(1, std::memory_order_relaxed);
	}

	return processed;
}

void log_flush() noexcept
{
	for (uint8_t i = 0; i < s_sink_count; ++i)
	{
		s_sinks[i]->flush();
	}
}

LogStats log_get_stats() noexcept
{
	LogStats stats{};
	stats.emitted = s_stat_emitted.load(std::memory_order_relaxed);
	stats.drained = s_stat_drained.load(std::memory_order_relaxed);
	stats.dropped_level = s_stat_dropped_level.load(std::memory_order_relaxed);
	stats.dropped_rate = s_stat_dropped_rate.load(std::memory_order_relaxed);
	stats.dropped_full = s_stat_dropped_full.load(std::memory_order_relaxed);
	stats.backend_failures = s_stat_backend_failures.load(std::memory_order_relaxed);
	stats.bytes_written = s_stat_bytes_written.load(std::memory_order_relaxed);
	return stats;
}

void log_reset_stats() noexcept
{
	reset_stats();
}

} // namespace homefly
