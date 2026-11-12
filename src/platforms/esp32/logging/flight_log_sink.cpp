#include "platforms/common/logging/flight_log_storage.hpp"

// Phase 2 ESP32-S3 flight log storage (SD card / Flash).
//
// The board support package provides four C hooks so this file contains no
// ESP-IDF headers and the common layer stays platform independent:
//   - hf_flight_log_mount()   esp_vfs_fat_sdmmc_mount() + fopen(path, "wb")
//   - hf_flight_log_write()   fwrite() into a FATFS file, or spi_flash_write
//   - hf_flight_log_flush()   fflush()/fsync()
//   - hf_flight_log_close()   fclose() + unmount
//
// All four are called from the LOW PRIORITY logging task only. The control
// loop never reaches this code. Add this file to the ESP-IDF component SRCS;
// on a host build the translation unit is empty and cannot collide with the
// Linux or STM32 implementation.

#if defined(ESP_PLATFORM)

extern "C" bool hf_flight_log_mount(const char* path) noexcept;
extern "C" bool hf_flight_log_write(const uint8_t* data, uint16_t length) noexcept;
extern "C" void hf_flight_log_flush() noexcept;
extern "C" void hf_flight_log_close() noexcept;

namespace homefly
{

namespace
{

class SdFlightLogSink final : public ILogSink
{
public:
	bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept override
	{
		return hf_flight_log_write(data, length);
	}

	void flush() noexcept override
	{
		hf_flight_log_flush();
	}
};

SdFlightLogSink s_storage;
bool s_mounted{false};

} // namespace

bool flight_log_storage_open(
	const char* path
) noexcept
{
	s_mounted = hf_flight_log_mount(path != nullptr ? path : "/sdcard/homefly.hfl");
	return s_mounted;
}

ILogSink& flight_log_storage() noexcept
{
	return s_storage;
}

void flight_log_storage_close() noexcept
{
	if (s_mounted)
	{
		hf_flight_log_close();
		s_mounted = false;
	}
}

} // namespace homefly

#endif // ESP_PLATFORM
