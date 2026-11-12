#include "platforms/linux/logging/file_log_sink.hpp"

namespace homefly
{

FileLogSink::FileLogSink(
	const char* path
) noexcept
{
	set_path(path);
}

FileLogSink::~FileLogSink()
{
	close();
}

void FileLogSink::set_path(
	const char* path
) noexcept
{
	_path[0] = '\0';

	if (path == nullptr)
	{
		return;
	}

	uint32_t i = 0;

	for (; i + 1u < sizeof(_path) && path[i] != '\0'; ++i)
	{
		_path[i] = path[i];
	}

	_path[i] = '\0';
}

bool FileLogSink::open(
	const char* path
) noexcept
{
	set_path(path);
	return open();
}

bool FileLogSink::open() noexcept
{
	close();

	if (_path[0] == '\0')
	{
		return false;
	}

	_file = std::fopen(_path, "wb");
	return _file != nullptr;
}

void FileLogSink::close() noexcept
{
	if (_file != nullptr)
	{
		std::fflush(_file);
		std::fclose(_file);
		_file = nullptr;
	}
}

bool FileLogSink::write(
	const uint8_t* data,
	uint16_t length
) noexcept
{
	if (_file == nullptr || data == nullptr || length == 0)
	{
		++_write_failures;
		return false;
	}

	const size_t written = std::fwrite(data, 1u, length, _file);

	if (written != static_cast<size_t>(length))
	{
		++_write_failures;
		return false;
	}

	_bytes_written += length;
	return true;
}

void FileLogSink::flush() noexcept
{
	if (_file != nullptr)
	{
		std::fflush(_file);
	}
}

} // namespace homefly
