cmake_minimum_required(VERSION 3.5)
project(aruco_det)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)

# compile the main node
add_executable(aruco_det src/aruco_det.cpp)

# link dependencies
target_link_libraries(aruco_det
  ${OpenCV_LIBS}
  yaml-cpp
)

# declare ROS 2 dependencies
ament_target_dependencies(aruco_det
  rclcpp
  sensor_msgs
  geometry_msgs
  OpenCV
  cv_bridge
  image_transport
)

install(TARGETS aruco_det
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
