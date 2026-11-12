#include "drivers/board/drv_board_led.h"

/* Software shadow state. Platform code maps it to a real GPIO. */
static uint8_t s_led_state = 0u;

void drv_board_led_set(uint8_t on)
{
	s_led_state = (on != 0u) ? 1u : 0u;
}

uint8_t drv_board_led_get(void)
{
	return s_led_state;
}
