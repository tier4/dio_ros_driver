cmake_minimum_required(VERSION 3.5)
project(dio_ros_driver)

# Compile as C++14, supported in ROS Foxy and newer.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)  
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find depended packages
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_auto_find_build_dependencies()

# generate original message.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DIOPort.msg"
  DEPENDENCIES builtin_interfaces
)

# build dio_ros_driver
include_directories(
  include
)

set(SOURCE_FILES src/dio_ros_driver_node.cpp
                 src/dio_ros_driver.cpp
                 src/dio_accessor_base.cpp
                 src/din_accessor.cpp
                 src/dout_accessor.cpp
                 src/dio_diagnostic_updater.cpp
                 src/dio_accessor_diagnosis.cpp)

ament_auto_add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})

find_package(Boost REQUIRED
  COMPONENTS system
)

# use gpiod as link library.
target_link_libraries(${PROJECT_NAME}_node
  gpiod
)

# find header file
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

# install node.
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
