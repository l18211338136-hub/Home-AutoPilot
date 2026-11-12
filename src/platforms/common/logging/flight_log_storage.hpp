#pragma once

#include "platforms/common/logging/log_sink.hpp"

namespace homefly
{

// Platform flight log storage, linked exactly once per firmware:
//   src/platforms/<platform>/logging/flight_log_sink.cpp
//
// Same pattern as log_backend_console(): no runtime dispatch, no virtual
// factory, and the common layer stays free of any SDK include. Firmware code
// only sees these four functions; swapping SD card / Flash / file storage
// never touches FlightCore or the application's logging calls.
//
// open() may be called with a host path (Linux Native) or nullptr (MCU with
// a board defined volume). It returns false when storage is unavailable.
bool flight_log_storage_open(
	const char* path
) noexcept;

ILogSink& flight_log_storage() noexcept;

void flight_log_storage_close() noexcept;

} // namespace homefly
