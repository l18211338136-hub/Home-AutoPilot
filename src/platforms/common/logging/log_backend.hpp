#pragma once

#include "platforms/common/logging/log_sink.hpp"

namespace homefly
{

// Exactly one definition per platform, in
//   src/platforms/<platform>/logging/log_backend.cpp
// Phase 1 backend: USB CDC / UART console. File (SD/Flash) and Telemetry
// sinks are added later via log_add_sink() without touching business modules.
ILogSink& log_backend_console() noexcept;

} // namespace homefly
