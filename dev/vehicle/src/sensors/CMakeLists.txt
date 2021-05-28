cmake_minimum_required(VERSION 3.5)
project(sensors)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic -lwiringPi -lm -lpthread -lcrypt -lrt)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(wiringPi REQUIRED)

# set(custom_directories
#   hardware
# )

# include_directories(
#   include ${custom_directories}
#   include ../hardware/include/
# )
# Messages and Services
set(msg_files
  "msg/SensorImu.msg"
  "msg/SensorLaser.msg"
  "msg/SensorLinearActuator.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Export message runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Executables
find_library(mserial Serial)
find_library(mi2c I2C)
find_library(si2c i2c)
find_library(pthreads pthread)

## Executable IMU
add_executable(imu 
  src/imu.cpp
  src/MPU6050.cpp
)
ament_target_dependencies(imu
  "rclcpp"
)
# add_library(
#   ${wpi}
# )
target_include_directories(imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu "${si2c}" "${mi2c}" "${pthreads}")

install(TARGETS imu
  DESTINATION lib/${PROJECT_NAME}
)
rosidl_target_interfaces(imu
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Executable Laser
add_executable(laser 
  src/laser.cpp
)
ament_target_dependencies(laser
  "rclcpp"
)
target_include_directories(laser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(laser "${mserial}" "${pthreads}")

install(TARGETS laser
  DESTINATION lib/${PROJECT_NAME}
)
rosidl_target_interfaces(laser
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

## Executable Linear Acuator
add_executable(linearActuator 
  src/linearactuator2.cpp
  #src/ADS1015.cpp
)
ament_target_dependencies(linearActuator
  "rclcpp"
)
target_include_directories(linearActuator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#target_link_libraries(linearActuator "${si2c}" "${mi2c}" "${pthreads}")
target_link_libraries(linearActuator "${mserial}" "${pthreads}")

install(TARGETS linearActuator
  DESTINATION lib/${PROJECT_NAME}
)
rosidl_target_interfaces(linearActuator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
