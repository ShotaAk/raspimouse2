cmake_minimum_required(VERSION 3.5)
project(raspimouse_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(raspimouse_msgs REQUIRED)

include_directories(include)

add_library(joystick_component SHARED src/joystick_component.cpp)
target_compile_definitions(joystick_component PRIVATE "RASPIMOUSE_EXAMPLES_BUILDING_DLL")
ament_target_dependencies(joystick_component
  rclcpp
  # rclcpp_lifecycle
  rclcpp_components
  std_msgs
  sensor_msgs
  # std_srvs
  # geometry_msgs
  # nav_msgs
  # tf2
  # tf2_ros
  # raspimouse_msgs
  )
rclcpp_components_register_nodes(joystick_component "joystick::JoystickComponent")

add_executable(joystick_example src/joystick_example.cpp)
target_link_libraries(joystick_example joystick_component)
ament_target_dependencies(joystick_example
  rclcpp
#   rclcpp_lifecycle
#   class_loader
#   std_msgs
#   std_srvs
#   geometry_msgs
#   nav_msgs
#   tf2
#   tf2_ros
#   raspimouse_msgs
  )

install(TARGETS
  joystick_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS
  joystick_example
  DESTINATION lib/${PROJECT_NAME})

# ament_export_dependencies(rosidl_default_runtime)
ament_package()
