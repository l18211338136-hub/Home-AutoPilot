# Linux Native console backend (stdout). The same source layout is used by
# the ESP32-S3 and STM32 backends in their vendor build systems.
add_library(homefly_log_backend STATIC
	log_backend.cpp
	file_log_sink.cpp
	flight_log_sink.cpp)

target_link_libraries(homefly_log_backend PUBLIC
	homefly_include
	homefly_logging_common)
