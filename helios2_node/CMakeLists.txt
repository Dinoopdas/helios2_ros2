cmake_minimum_required(VERSION 3.15)
project(helios2_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_definitions(-DGENICAM_USER_ACCEPTS_ANY_COMPILER)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(arena_sdk REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

add_executable(helios2_node src/helios2_node.cpp)

target_include_directories(helios2_node PRIVATE
  ${arena_sdk_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(helios2_node
  rclcpp
  sensor_msgs
  std_msgs
  arena_sdk
)

target_link_libraries(helios2_node
  ${arena_sdk_LIBRARIES}
  ${OpenCV_LIBS}
  opencv_calib3d
)

install(TARGETS helios2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# -------- Probe tool --------
add_executable(helios2_probe src/helios2_probe.cpp)
target_include_directories(helios2_probe PRIVATE
  ${arena_sdk_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(helios2_probe rclcpp arena_sdk)
target_link_libraries(helios2_probe
  ${arena_sdk_LIBRARIES}
  ${OpenCV_LIBS}
  opencv_calib3d
)
install(TARGETS helios2_probe DESTINATION lib/${PROJECT_NAME})

ament_package()

