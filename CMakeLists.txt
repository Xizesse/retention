cmake_minimum_required(VERSION 3.5)
project(retention)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)


# add the executables
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(talkerRange src/talkerRange.cpp)
ament_target_dependencies(talkerRange rclcpp std_msgs)

add_executable(talkerState src/talkerState.cpp)
ament_target_dependencies(talkerState rclcpp std_msgs)

add_executable(talkerDrone src/talkerDrone.cpp)
ament_target_dependencies(talkerDrone rclcpp std_msgs)

add_executable(listenerBoth src/listenerBoth.cpp)
ament_target_dependencies(listenerBoth rclcpp std_msgs sensor_msgs drone_interfaces)

add_executable(DroneStateRange src/DroneStateRange.cpp)
ament_target_dependencies(DroneStateRange rclcpp std_msgs sensor_msgs drone_interfaces)

add_executable(converte src/converte.cpp)
ament_target_dependencies(converte rclcpp std_msgs sensor_msgs drone_interfaces)

install(TARGETS
  listener
  talkerRange
  talkerState
  listenerBoth
  talker
  talkerDrone
  DroneStateRange
  converte
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()