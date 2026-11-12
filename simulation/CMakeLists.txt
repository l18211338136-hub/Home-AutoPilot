add_library(homefly_simulation STATIC
	sim_clock.cpp
	sim_plant.cpp
	sim_imu.cpp
	sim_motor.cpp
	sim_link.cpp
	gazebo/gazebo_protocol.cpp
	gazebo/gazebo_transport.cpp
	gazebo/gazebo_hal.cpp
	gazebo/udp_socket.cpp)

target_link_libraries(homefly_simulation PUBLIC
	homefly_include homefly_hal homefly_control Threads::Threads)

if(WIN32)
	target_link_libraries(homefly_simulation PUBLIC ws2_32)
endif()
