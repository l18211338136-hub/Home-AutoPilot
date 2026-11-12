#pragma once

#include <cstddef>
#include <cstdint>

#include "hal/i_clock.hpp"
#include "platforms/common/logging/log_level.hpp"
#include "platforms/common/logging/log_sink.hpp"

// Compile time level filter. e.g. -DHF_LOG_COMPILE_MIN_LEVEL=2 strips all
// DEBUG/INFO calls completely (zero cost) from a production firmware build.
#ifndef HF_LOG_COMPILE_MIN_LEVEL
#define HF_LOG_COMPILE_MIN_LEVEL 0
#endif

namespace homefly
{

constexpr uint32_t kLogRingCapacity = 32;      // power of two
constexpr uint16_t kLogLineCapacity = 160;     // including the terminating NUL
constexpr uint8_t kLogSinkCapacity = 4;
constexpr uint8_t kLogRateSlots = 16;

struct LogConfig
{
	LogLevel min_level{LogLevel::Info};

	// Minimum interval between two records with the same (tag, level).
	// 0 disables throttling.
	uint32_t rate_limit_ms{0};
};

struct LogStats
{
	uint32_t emitted{0};
	uint32_t drained{0};
	uint32_t dropped_level{0};
	uint32_t dropped_rate{0};
	uint32_t dropped_full{0};
	uint32_t backend_failures{0};
	uint32_t bytes_written{0};
};

// Call once during single threaded startup, before any HF_LOG_* call.
// The console sink is registered automatically; extra sinks (SD card,
// telemetry) can be added with log_add_sink().
void log_init(
	IClock& clock,
	ILogSink& console,
	const LogConfig& config = LogConfig{}
) noexcept;

void log_shutdown() noexcept;

bool log_add_sink(
	ILogSink& sink
) noexcept;

void log_set_level(
	LogLevel level
) noexcept;

LogLevel log_get_level() noexcept;

// Front end. Bounded CPU work only: level check, throttle, bounded
// formatting, lock free enqueue. No I/O, no allocation, no blocking.
// Safe from the 1 kHz control loop and from any other task.
void log_write(
	LogLevel level,
	const char* tag,
	const char* format,
	...
) noexcept;

// Back end. Call from an idle / low priority task. May block on a sink.
// max_records == 0 drains everything currently queued.
uint32_t log_drain(
	uint32_t max_records = 0
) noexcept;

void log_flush() noexcept;

LogStats log_get_stats() noexcept;

void log_reset_stats() noexcept;

} // namespace homefly

// ---------------------------------------------------------------------------
// Unified API. Every HomeFly module uses these macros, never printf/cout.
// When a level is compiled out, the formatting arguments are not evaluated:
// never put side effects (i++, sensor reads) inside a log argument.
// ---------------------------------------------------------------------------

#if HF_LOG_COMPILE_MIN_LEVEL <= 0
#define HF_LOG_DEBUG(tag, ...) \
	do { ::homefly::log_write(::homefly::LogLevel::Debug, (tag), __VA_ARGS__); } while (false)
#else
#define HF_LOG_DEBUG(tag, ...) do { (void)0; } while (false)
#endif

#if HF_LOG_COMPILE_MIN_LEVEL <= 1
#define HF_LOG_INFO(tag, ...) \
	do { ::homefly::log_write(::homefly::LogLevel::Info, (tag), __VA_ARGS__); } while (false)
#else
#define HF_LOG_INFO(tag, ...) do { (void)0; } while (false)
#endif

#if HF_LOG_COMPILE_MIN_LEVEL <= 2
#define HF_LOG_WARN(tag, ...) \
	do { ::homefly::log_write(::homefly::LogLevel::Warn, (tag), __VA_ARGS__); } while (false)
#else
#define HF_LOG_WARN(tag, ...) do { (void)0; } while (false)
#endif

#if HF_LOG_COMPILE_MIN_LEVEL <= 3
#define HF_LOG_ERROR(tag, ...) \
	do { ::homefly::log_write(::homefly::LogLevel::Error, (tag), __VA_ARGS__); } while (false)
#else
#define HF_LOG_ERROR(tag, ...) do { (void)0; } while (false)
#endif

#define HF_LOG_FATAL(tag, ...) \
	do { ::homefly::log_write(::homefly::LogLevel::Fatal, (tag), __VA_ARGS__); } while (false)
