#include "platforms/common/logging/log_backend.hpp"

#include <cstdio>

namespace homefly
{

namespace
{

// Phase 1 Console sink. Swapping this for a file or telemetry sink does not
// require any change in a business module: just log_add_sink() at startup.
class ConsoleLogSink final : public ILogSink
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

		const size_t written = std::fwrite(data, 1, length, stdout);
		std::fputc('\n', stdout);
		return written == static_cast<size_t>(length);
	}

	void flush() noexcept override
	{
		std::fflush(stdout);
	}
};

ConsoleLogSink s_console;

} // namespace

ILogSink& log_backend_console() noexcept
{
	return s_console;
}

} // namespace homefly
