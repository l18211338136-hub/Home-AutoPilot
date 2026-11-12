add_executable(homefly_linux_native
	main.cpp
	simulation_source.cpp)

target_link_libraries(homefly_linux_native PRIVATE
	homefly_flight_core
	homefly_control
	homefly_configuration
	homefly_communication
	homefly_drivers
	homefly_platform_linux
	homefly_log_backend
	homefly_logging_common
	homefly_simulation
	homefly_hal
	homefly_include
	homefly_log_filter)
