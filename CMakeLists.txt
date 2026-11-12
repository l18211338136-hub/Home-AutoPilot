cmake_minimum_required(VERSION 3.20)

project(homefly
	VERSION 0.1.0
	DESCRIPTION "HomeFly - hardware independent flight control architecture"
	LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(HOMEFLY_PLATFORM "linux" CACHE STRING "Target platform: linux | stm32 | esp32")
set_property(CACHE HOMEFLY_PLATFORM PROPERTY STRINGS linux stm32 esp32)

# std::thread is used by the Linux clock/timer, the Gazebo transport I/O task
# and the native application. MSVC needs no explicit link library, but Linux
# toolchains do (older glibc exposes pthread_create only in libpthread).
find_package(Threads REQUIRED)

if(MSVC)
	add_compile_options(/W4 /permissive- /utf-8)
	# The Linux Native file sink uses ISO C fopen/fwrite (portable to MCU
	# toolchains via the BSP hooks). MSVC's deprecation warning is not
	# actionable here; keep /W4 warning free.
	add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Two include roots, no path hacks inside source files:
#   src/          -> #include "hal/i_imu.hpp", "control/pid_controller.hpp"
#   project root  -> #include "simulation/sim_plant.hpp", "test/homefly_test.hpp"
add_library(homefly_include INTERFACE)
target_include_directories(homefly_include INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/src
	${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src/hal)
add_subdirectory(src/control)
add_subdirectory(src/configuration)
add_subdirectory(src/flight_core)
add_subdirectory(src/communication)
add_subdirectory(src/drivers)
add_subdirectory(src/platforms/common/logging)

if(HOMEFLY_PLATFORM STREQUAL "linux")
	add_subdirectory(src/platforms/linux)
	add_subdirectory(src/platforms/linux/logging)
	add_subdirectory(simulation)
	add_subdirectory(src/applications/linux_native)

	include(CTest)
	if(BUILD_TESTING)
		add_subdirectory(test)
	endif()
else()
	message(STATUS "HOMEFLY_PLATFORM=${HOMEFLY_PLATFORM}: MCU firmware uses the vendor build system")
	message(STATUS "  ESP32-S3: idf.py -C boards/esp32_s3 build")
	message(STATUS "  STM32   : cmake -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake")
endif()
