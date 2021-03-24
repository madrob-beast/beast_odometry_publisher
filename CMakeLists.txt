cmake_minimum_required(VERSION 2.8.3)
project(beast_odometry_publisher)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs beast_msgs tf tf2_ros)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(odometry_publisher src/odometry_node.cpp src/odometry.cpp)
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})
