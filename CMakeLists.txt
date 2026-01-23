cmake_minimum_required(VERSION 3.8)
project(unitree_ros2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)


# Install .msg
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsState.msg"
  "msg/HighCmd.msg"
  "msg/IMU.msg"
  "msg/BmsCmd.msg"
  "msg/HighState.msg"
  "msg/LED.msg"
  "msg/Cartesian.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/MotorState.msg"
  "msg/MotorCmd.msg"
  DEPENDENCIES
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


#
# Linking Unitree's SDK's
#
link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/amd64)
SET(LEGGED_LIB ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/amd64/libunitree_legged_sdk.a)


#
# Include SDK's header files
#
include_directories(include ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include)
include_directories(${OpenCV_INCLUDE_DIRS})


#
# Create executables
#
add_executable(legged_controller src/legged_controller.cpp)
#
# Set dependacies
#
ament_export_dependencies(rosidl_default_runtime) # msg export
ament_target_dependencies(legged_controller rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)



# Install includes before packages
install(
  DIRECTORY include 
  ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include 
  DESTINATION include
)

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#
# Install executables to package
#
install(TARGETS 
  legged_controller
  DESTINATION lib/${PROJECT_NAME})


#
# Install SDK's
#
target_link_libraries(legged_controller
  ${LEGGED_LIB}
  "${cpp_typesupport_target}"
  pthread
  rt
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
