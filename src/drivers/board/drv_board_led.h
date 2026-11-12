#ifndef DRV_BOARD_LED_H
#define DRV_BOARD_LED_H

#include <stdint.h>

/* C11 board primitive. The board adapter replaces the weak defaults below. */
void drv_board_led_set(uint8_t on);
uint8_t drv_board_led_get(void);

#endif /* DRV_BOARD_LED_H */
