#include "platforms/common/logging/log_backend.hpp"

// STM32 backend template. Add this file to the STM32 CMake target and define
// HOMEFLY_PLATFORM_STM32. The board support package must provide the UART
// handle "hf_log_uart" (or switch this sink to ITM/SWO).
//
// On any other build the translation unit is empty, so it can never collide
// with another platform's backend.

#if defined(HOMEFLY_PLATFORM_STM32)

#include <cstdint>

extern "C"
{
#include "stm32f4xx_hal.h"
}

extern UART_HandleTypeDef hf_log_uart;

namespace homefly
{

namespace
{

class Stm32UartLogSink final : public ILogSink
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

		// Only the low priority drain task calls this. A future revision
		// should use HAL_UART_Transmit_IT + a TX complete callback so even
		// the drain task never spins on a full FIFO.
		const HAL_StatusTypeDef status = HAL_UART_Transmit(&hf_log_uart,
			const_cast<uint8_t*>(data), static_cast<uint16_t>(length), 100u);
		static const uint8_t kNewline = '\n';
		HAL_UART_Transmit(&hf_log_uart, const_cast<uint8_t*>(&kNewline), 1u, 100u);

		return status == HAL_OK;
	}
};

Stm32UartLogSink s_console;

} // namespace

ILogSink& log_backend_console() noexcept
{
	return s_console;
}

} // namespace homefly

#endif // HOMEFLY_PLATFORM_STM32
