cmake_minimum_required(VERSION 3.5)
project(communication)

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
set(srv_files
  "srv/CommRFSend.srv"
  "srv/CommRFRecv.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

# Export message runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Executable RFD900x
add_executable(rfd900x
 src/rfd900x.cpp
 src/Serial.cpp
)
ament_target_dependencies(rfd900x
  "rclcpp"
)

# find libraries -> if any dependencies
## find_library(name lib_name)

target_include_directories(rfd900x PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link target libraries
## target_link_libraries(rfd900x names...)

install(TARGETS rfd900x
  DESTINATION lib/${PROJECT_NAME}
)
rosidl_target_interfaces(rfd900x
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
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
