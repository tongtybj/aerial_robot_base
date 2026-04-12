cmake_minimum_required(VERSION 3.8)
project(aerial_robot_nav)

# --- C++17 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- common dependent packages ---
set(ROS_DEPENDENCIES
  aerial_robot_estimation
  aerial_robot_msgs
  geodesy
  geographic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  spinal
  std_msgs
  visualization_msgs
  )

### flight navigation
add_library (flight_navigation
  src/flight_navigation.cpp
  src/util/joy_parser.cpp
  src/trajectory/reference_base.cpp
  src/trajectory/trajectory_reference/polynomial.cpp
  src/trajectory/trajectory_reference/polynomial_trajectory.cpp
  src/trajectory/trajectory_reference/sampled_trajectory.cpp
  src/trajectory/base/parameter_base.cpp
  src/trajectory/math/math.cpp
  src/trajectory/types/command.cpp
  src/trajectory/types/quad_state.cpp
  src/trajectory/types/quadrotor.cpp
  src/trajectory/utils/logger.cpp
  src/trajectory/utils/timer.cpp)
ament_target_dependencies(flight_navigation
  ${ROS_DEPENDENCIES}
)
target_include_directories(flight_navigation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

# --- installation ------------------------------
install(
  TARGETS
    flight_navigation

  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY include/
  DESTINATION include
)

# --- export ------------------------------
ament_export_libraries(
  flight_navigation
  )
ament_export_include_directories(include)
ament_export_dependencies(
  aerial_robot_estimation
  aerial_robot_msgs
  geodesy
  geometry_msgs
  nav_msgs
  sensor_msgs
  spinal
  std_srvs
  visualization_msgs
)



ament_package()
