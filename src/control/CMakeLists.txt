add_library(homefly_control STATIC
	pid_controller.cpp
	attitude_estimator.cpp
	motor_mixer.cpp)

target_link_libraries(homefly_control PUBLIC homefly_include homefly_hal)
