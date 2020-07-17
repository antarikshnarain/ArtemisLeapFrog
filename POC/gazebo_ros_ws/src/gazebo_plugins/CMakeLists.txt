cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# gazebo_ros_model_lift
add_library(gazebo_ros_model_lift SHARED
  src/gazebo_ros_model_lift.cpp
  src/utilities/PIDController.cpp
)
target_include_directories(gazebo_ros_model_lift PUBLIC include)
ament_target_dependencies(gazebo_ros_model_lift
  "gazebo_dev"
  "gazebo_msgs"
  "gazebo_ros"
  "geometry_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_model_lift)

# gazebo_ros_linear_actuator
add_library(gazebo_ros_actuator_move SHARED
  src/gazebo_ros_actuator_move.cpp
)
target_include_directories(gazebo_ros_actuator_move PUBLIC include)
ament_target_dependencies(gazebo_ros_actuator_move
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_actuator_move)

# gazebo_ros_ackermann_drive
add_library(gazebo_ros_ackermann_drive SHARED
  src/gazebo_ros_ackermann_drive.cpp
)
target_include_directories(gazebo_ros_ackermann_drive PUBLIC include)
ament_target_dependencies(gazebo_ros_ackermann_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_ackermann_drive)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)


# TODO: Add testing direct with tests for each plugin
 #if(BUILD_TESTING)
 #  add_subdirectory(test)
 #  find_package(ament_lint_auto REQUIRED)
 #  ament_lint_auto_find_test_dependencies()
 #endif()

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_model_lift
    gazebo_ros_actuator_move
    gazebo_ros_ackermann_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
