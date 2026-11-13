add_library(homefly_flight_core STATIC
	flight_controller.cpp
	flight_state_machine.cpp
	prearm_checker.cpp
	safety_monitor.cpp)

target_link_libraries(homefly_flight_core PUBLIC
	homefly_include
	homefly_hal
	homefly_control
	homefly_configuration)
