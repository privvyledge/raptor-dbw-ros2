cmake_minimum_required(VERSION 3.5)
project(raptor_dbw_joystick)

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
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

#ament_auto_add_executable(${PROJECT_NAME}_node
#  src/raptor_dbw_joystick_node.cpp
#  src/raptor_dbw_joystick.cpp
#)

# Composition setup
add_library(raptor_dbw_joystick_component SHARED src/raptor_dbw_joystick.cpp)
#ament_target_dependencies(raptor_dbw_joystick_component ...)

rclcpp_components_register_node(raptor_dbw_joystick_component
  PLUGIN "RaptorDbwJoystick::RaptorDbwJoystickComponent"
  EXECUTABLE raptor_dbw_joystick_node
)

ament_export_targets(raptor_dbw_joystick_component)
install(TARGETS raptor_dbw_joystick_component
        EXPORT export_raptor_dbw_joystick_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
