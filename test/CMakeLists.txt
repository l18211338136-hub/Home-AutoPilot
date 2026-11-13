# GoogleTest is used when available. The bundled header-only harness in
# homefly_test.hpp keeps CTest usable on offline machines without adding a
# third-party dependency (SKILL.md rule 9).
find_package(GTest QUIET)

if(GTest_FOUND)
	message(STATUS "HomeFly: using system GoogleTest")
else()
	message(STATUS "HomeFly: GoogleTest not found, using bundled minimal harness")
endif()

function(homefly_add_test name)
	add_executable(${name} ${ARGN})
	target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
	target_link_libraries(${name} PRIVATE
		homefly_include
		homefly_hal
		homefly_control
		homefly_configuration
		homefly_flight_core
		homefly_communication
		homefly_drivers
		homefly_simulation
		homefly_logging_common)
	if(GTest_FOUND)
		target_link_libraries(${name} PRIVATE GTest::gtest_main)
		target_compile_definitions(${name} PRIVATE HOMEFLY_USE_GTEST)
	endif()
	add_test(NAME ${name} COMMAND ${name})
endfunction()

homefly_add_test(test_pid_controller control/test_pid_controller.cpp)
homefly_add_test(test_attitude_estimator control/test_attitude_estimator.cpp)
homefly_add_test(test_motor_mixer control/test_motor_mixer.cpp)
homefly_add_test(test_flight_state_machine flight_core/test_flight_state_machine.cpp)
homefly_add_test(test_flight_controller flight_core/test_flight_controller.cpp)
homefly_add_test(test_prearm_checker flight_core/test_prearm_checker.cpp)
homefly_add_test(test_sim_plant simulation/test_sim_plant.cpp)
homefly_add_test(test_gazebo_protocol simulation/test_gazebo_protocol.cpp)
homefly_add_test(test_gazebo_transport simulation/test_gazebo_transport.cpp)
homefly_add_test(test_pwm_motor drivers/test_pwm_motor.cpp)
homefly_add_test(test_telemetry communication/test_telemetry.cpp)
homefly_add_test(test_control_protocol communication/test_control_protocol.cpp)
homefly_add_test(test_command_receiver communication/test_command_receiver.cpp)
homefly_add_test(test_wifi_integration communication/test_wifi_integration.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/applications/linux_native/udp_link.cpp)
homefly_add_test(test_logging platforms/test_logging.cpp)
homefly_add_test(test_flight_data platforms/test_flight_data.cpp)
homefly_add_test(test_flight_log platforms/test_flight_log.cpp)
