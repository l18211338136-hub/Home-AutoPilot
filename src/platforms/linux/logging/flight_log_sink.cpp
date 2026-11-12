#include "platforms/common/logging/flight_log_storage.hpp"

#include "platforms/linux/logging/file_log_sink.hpp"

// Linux Native implementation of the platform flight log storage factory.
// The object is statically allocated; no dynamic memory is involved.
namespace homefly
{

namespace
{

FileLogSink s_storage("");

} // namespace

bool flight_log_storage_open(
	const char* path
) noexcept
{
	return s_storage.open(path != nullptr ? path : "homefly_flight.hfl");
}

ILogSink& flight_log_storage() noexcept
{
	return s_storage;
}

void flight_log_storage_close() noexcept
{
	s_storage.close();
}

} // namespace homefly
