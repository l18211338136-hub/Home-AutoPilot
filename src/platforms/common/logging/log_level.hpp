#pragma once

#include <cstdint>

namespace homefly
{

enum class LogLevel : uint8_t
{
	Debug = 0,
	Info = 1,
	Warn = 2,
	Error = 3,
	Fatal = 4
};

constexpr uint8_t kLogLevelCount = 5;

// Fixed width (5 chars) label used inside the log line: "DEBUG", "INFO ", ...
const char* log_level_label(
	LogLevel level
) noexcept;

// Unpadded name, useful for configuration parsing and telemetry.
const char* log_level_name(
	LogLevel level
) noexcept;

} // namespace homefly
