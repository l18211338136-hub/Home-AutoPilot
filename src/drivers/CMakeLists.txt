add_library(homefly_drivers STATIC
	motor/drv_pwm_motor.cpp
	board/drv_board_led.c)

target_link_libraries(homefly_drivers PUBLIC homefly_include homefly_hal)
