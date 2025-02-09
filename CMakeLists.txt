cmake_minimum_required(VERSION 3.8)
project(stereo_odometry_ros)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(stereo_odometry_node src/stereo_odometry_node.cpp
                                    src/camera.cpp
                                    src/frame.cpp
                                    src/stereo_odometry.cpp)

add_executable(image_publisher_node src/image_publisher_node.cpp)

ament_target_dependencies(stereo_odometry_node rclcpp
                                                std_msgs 
                                                nav_msgs
                                                tf2_geometry_msgs  
                                                sensor_msgs 
                                                image_transport 
                                                cv_bridge 
                                                message_filters 
                                                OpenCV)

ament_target_dependencies(image_publisher_node rclcpp
                                                std_msgs 
                                                sensor_msgs 
                                                image_transport 
                                                cv_bridge 
                                                OpenCV)

install(TARGETS 
  stereo_odometry_node
  image_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
