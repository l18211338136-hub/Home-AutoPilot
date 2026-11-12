#include "platforms/common/logging/flight_log_storage.hpp"

// Phase 4 STM32 flight log storage (SD card via FatFs / QSPI Flash).
//
// The board support package provides four C hooks so this file contains no
// HAL/SDK headers and the common layer stays platform independent:
//   - hf_flight_log_mount()   f_mount() + f_open(path, FA_WRITE | FA_CREATE_ALWAYS)
//   - hf_flight_log_write()   f_write()  (prefer a contiguous pre-allocated file)
//   - hf_flight_log_flush()   f_sync()
//   - hf_flight_log_close()   f_close()
//
// All four are called from the LOW PRIORITY logging task only. For best
// results the BSP should use DMA + double buffering; the common writer
// already batches 16 records per call so no per-cycle I/O ever occurs.
// On a host build the translation unit is empty.

#if defined(HOMEFLY_PLATFORM_STM32)

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
	s_mounted = hf_flight_log_mount(path != nullptr ? path : "0:homefly.hfl");
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

#endif // HOMEFLY_PLATFORM_STM32
