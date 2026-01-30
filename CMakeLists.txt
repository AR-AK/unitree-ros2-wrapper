cmake_minimum_required(VERSION 3.10)
project(unitree_ros2_cpp)

# -----------------------------------------------------------------------
# Policy Settings (Silence Warnings)
# -----------------------------------------------------------------------
# Silence CMP0148 (FindPythonInterp/Libs) warnings common in Humble/Rolling
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# -----------------------------------------------------------------------
# Architecture Detection (ARM64 vs AMD64)
# -----------------------------------------------------------------------
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*" OR "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "amd64.*")
  set(UNITREE_ARCH amd64)
  message(STATUS "Unitree SDK: Detected AMD64 architecture")
elseif("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*" OR "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm64.*")
  set(UNITREE_ARCH arm64)
  message(STATUS "Unitree SDK: Detected ARM64 architecture")
else()
  # Fallback: If not detected, try to guess or warn. 
  # Note: Some build environments report just 'unknown'. 
  # We default to arm64 if on a robot-like environment, or error out.
  message(WARNING "Unitree SDK: Unknown architecture '${CMAKE_SYSTEM_PROCESSOR}'. Defaulting to amd64 for build safety.")
  set(UNITREE_ARCH amd64)
endif()

# -----------------------------------------------------------------------
# Compiler Options (Modern C++)
# -----------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enforce C++17 to support std::function/std::bind/std::thread
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------
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

# -----------------------------------------------------------------------
# Link Unitree SDK
# -----------------------------------------------------------------------
set(LEGGED_LIB_PATH ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${UNITREE_ARCH})
set(LEGGED_LIB ${LEGGED_LIB_PATH}/libunitree_legged_sdk.a)

# Verify library exists before linking
if(NOT EXISTS "${LEGGED_LIB}")
    message(FATAL_ERROR "Unitree SDK Library not found at: ${LEGGED_LIB}. Please ensure submodules are initialized and arch is correct.")
endif()

link_directories(${LEGGED_LIB_PATH})

# Include SDK header files
include_directories(include ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include)

# -----------------------------------------------------------------------
# Executables
# -----------------------------------------------------------------------
add_executable(legged_controller src/legged_controller.cpp)

# Set dependencies
ament_export_dependencies(rosidl_default_runtime) 
ament_target_dependencies(legged_controller rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

# Link SDK and System Libs
target_link_libraries(legged_controller
  ${LEGGED_LIB}
  "${cpp_typesupport_target}"
  pthread
  rt
)

# -----------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------

# Fixed: Only install the SDK includes. Removed the reference to the non-existent root 'include' folder.
install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
  legged_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()