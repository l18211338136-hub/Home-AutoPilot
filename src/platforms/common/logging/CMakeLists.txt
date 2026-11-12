# Platform independent logging front end. No platform API is used here.
add_library(homefly_logging_common STATIC
	log.cpp
	flight_data.cpp
	flight_log.cpp)

target_link_libraries(homefly_logging_common PUBLIC
	homefly_include
	homefly_hal)

# HF_LOG_COMPILE_MIN_LEVEL strips low severity calls at compile time.
# It is opt-in per firmware / application target, NOT attached to the common
# library: unit test targets must always see every log level.
# Example: -DHF_LOG_COMPILE_MIN_LEVEL=2 for a release flight firmware.
add_library(homefly_log_filter INTERFACE)
if(DEFINED HF_LOG_COMPILE_MIN_LEVEL)
	target_compile_definitions(homefly_log_filter INTERFACE
		HF_LOG_COMPILE_MIN_LEVEL=${HF_LOG_COMPILE_MIN_LEVEL})
endif()
