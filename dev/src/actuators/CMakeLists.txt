cmake_minimum_required(VERSION 3.5)
project(actuators)

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
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Messages and Services
set(msg_files
  "msg/ActuatorJCP300Info.msg"
  "msg/ActuatorJCP300Telemetry.msg"
)
set(srv_files
  "srv/ActuatorJCP300HealthCheck.srv"
  "srv/ActuatorJCP300Params.srv"
  "srv/ActuatorJCP300Status.srv"
  "srv/ActuatorJCP300Thrust.srv"
  "srv/ActuatorColdGasFireThruster.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

find_library(wpi wiringPi)
find_library(shm rt)
find_library(cpt crypt)

# Executable Engine
add_executable(engine
 src/engine.cpp
 src/JetCatP300.cpp
 src/Serial.cpp
)
ament_target_dependencies(engine
  "rclcpp"
)

target_include_directories(engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(engine "${wpi}" "${shm}" "${cpt}")

install(TARGETS engine
  DESTINATION lib/${PROJECT_NAME})
rosidl_target_interfaces(engine
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Executable ACS
add_executable(acs
 src/acs.cpp
)
ament_target_dependencies(acs
  "rclcpp"
)

target_include_directories(acs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(acs "${wpi}" "${shm}" "${cpt}")

install(TARGETS acs
  DESTINATION lib/${PROJECT_NAME})
rosidl_target_interfaces(acs
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
