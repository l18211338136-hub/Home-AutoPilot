#include "platforms/common/logging/log_backend.hpp"

// Phase 1 ESP32-S3 backend: USB-UART console (UART0).
// Add this file to the ESP-IDF component SRCS; ESP_PLATFORM is defined by
// the ESP-IDF build system. On a host build the translation unit is empty,
// so it can never collide with another platform's backend.
//
// Phase 2: replace UART0 with the native USB CDC console / TinyUSB, then add
// an SD card sink and a telemetry sink via homefly::log_add_sink().

#if defined(ESP_PLATFORM)

#include "driver/uart.h"

namespace homefly
{

namespace
{

constexpr uart_port_t kLogUart = UART_NUM_0;

class Esp32ConsoleLogSink final : public ILogSink
{
public:
	bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept override
	{
		if (data == nullptr || length == 0)
		{
			return false;
		}

		// Called from the low priority drain task, never from the control
		// loop, so a blocking UART transmit is acceptable here.
		const int written = uart_write_bytes(kLogUart,
			reinterpret_cast<const char*>(data), length);
		static const char kNewline = '\n';
		uart_write_bytes(kLogUart, &kNewline, 1);

		return written == static_cast<int>(length);
	}
};

Esp32ConsoleLogSink s_console;

} // namespace

ILogSink& log_backend_console() noexcept
{
	return s_console;
}

} // namespace homefly

#endif // ESP_PLATFORM
