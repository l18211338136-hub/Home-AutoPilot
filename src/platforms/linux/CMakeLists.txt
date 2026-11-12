add_library(homefly_platform_linux STATIC
	linux_clock.cpp
	linux_timer.cpp)

target_link_libraries(homefly_platform_linux PUBLIC homefly_include homefly_hal)

if(WIN32)
	# LinuxTimer raises the Windows scheduler tick for accurate 1 kHz pacing.
	target_link_libraries(homefly_platform_linux PUBLIC winmm)
endif()
