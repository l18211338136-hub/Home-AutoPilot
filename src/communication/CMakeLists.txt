add_library(homefly_communication STATIC
	telemetry.cpp
	control_protocol.cpp
	command_receiver.cpp)

target_link_libraries(homefly_communication PUBLIC homefly_include homefly_flight_core homefly_hal)
