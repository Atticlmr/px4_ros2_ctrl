cmake_minimum_required(VERSION 3.10)
project(px4_ros2_ctrl)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)
find_package(ONNXRuntime REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(px4_output_adapter SHARED src/lib/px4_output_adapter.cpp)
ament_target_dependencies(px4_output_adapter rclcpp px4_msgs)
target_include_directories(px4_output_adapter PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)


# lib/listeners/sensor_combined_listener
add_executable(sensor_combined_listener src/lib/listeners/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

# lib/listeners/vehicle_gps_position_listener
add_executable(vehicle_gps_position_listener src/lib/listeners/vehicle_gps_position_listener.cpp)
ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)
install(TARGETS vehicle_gps_position_listener DESTINATION lib/${PROJECT_NAME})

# lib/advertisers/debug_vect_advertiser
add_executable(debug_vect_advertiser src/lib/advertisers/debug_vect_advertiser.cpp)
ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)
install(TARGETS debug_vect_advertiser DESTINATION lib/${PROJECT_NAME})

# lib/offboard/offboard_control
add_executable(offboard_control src/lib/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# lib/offboard/offboard_control_srv
add_executable(offboard_control_srv src/lib/offboard/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv rclcpp px4_msgs)
install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})

# fsm/fsm_node
add_executable(fsm_node src/fsm/fsm_node.cpp)
ament_target_dependencies(fsm_node rclcpp px4_msgs std_srvs)
target_link_libraries(fsm_node px4_output_adapter)
install(TARGETS fsm_node DESTINATION lib/${PROJECT_NAME})

# position controller

add_executable(position_controller src/controller/position_controller.cpp)
ament_target_dependencies(position_controller rclcpp px4_msgs)
install(TARGETS position_controller DESTINATION lib/${PROJECT_NAME})

add_executable(mocap_udp_bridge src/localization/mocap_udp_bridge.cpp)
target_compile_features(mocap_udp_bridge PRIVATE cxx_std_17)
ament_target_dependencies(mocap_udp_bridge rclcpp px4_msgs Eigen3)
target_link_libraries(mocap_udp_bridge Threads::Threads)
install(TARGETS mocap_udp_bridge DESTINATION lib/${PROJECT_NAME})

add_executable(rl_thrust_controller src/controller/rl_thrust_controller.cpp)
target_compile_features(rl_thrust_controller PRIVATE cxx_std_17)
ament_target_dependencies(rl_thrust_controller rclcpp px4_msgs)
target_link_libraries(rl_thrust_controller ONNXRuntime::onnxruntime)
set_target_properties(rl_thrust_controller PROPERTIES
	BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime/lib"
	INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime/lib"
)
install(TARGETS rl_thrust_controller DESTINATION lib/${PROJECT_NAME})

# calibration/thrust_calibration_node
add_executable(thrust_calibration_node src/calibration/thrust_calibration_node.cpp)
ament_target_dependencies(thrust_calibration_node rclcpp px4_msgs geometry_msgs std_srvs)
install(TARGETS thrust_calibration_node DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs std_srvs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms px4_output_adapter)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(TARGETS px4_output_adapter
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install scripts.
install(PROGRAMS
	scripts/body_rate_nmpc_controller.py
	scripts/fit_thrust_calibration.py
	scripts/generate_body_rate_nmpc_solver.py
	scripts/test_body_rate_nmpc_solver.py
	DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/generated")
	install(DIRECTORY generated DESTINATION share/${PROJECT_NAME}/)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
	install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)
endif()

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()


ament_package()
