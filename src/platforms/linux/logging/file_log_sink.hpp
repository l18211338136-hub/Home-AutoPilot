#pragma once

#include <cstdint>
#include <cstdio>

#include "platforms/common/logging/log_sink.hpp"

namespace homefly
{

// Linux Native flight log storage: a plain binary file.
// Only ever used by the low priority logging task. The control loop never
// sees this type; the common layer only knows ILogSink.
class FileLogSink final : public ILogSink
{
public:
	explicit FileLogSink(
		const char* path
	) noexcept;

	~FileLogSink() override;

	FileLogSink(const FileLogSink&) = delete;
	FileLogSink& operator=(const FileLogSink&) = delete;

	bool open() noexcept;
	bool open(
		const char* path
	) noexcept;
	void close() noexcept;

	bool is_open() const noexcept
	{
		return _file != nullptr;
	}

	uint32_t bytes_written() const noexcept
	{
		return _bytes_written;
	}

	uint32_t write_failures() const noexcept
	{
		return _write_failures;
	}

	bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept override;

	void flush() noexcept override;

private:
	void set_path(
		const char* path
	) noexcept;

	char _path[256]{};
	std::FILE* _file{nullptr};
	uint32_t _bytes_written{0};
	uint32_t _write_failures{0};
};

} // namespace homefly
