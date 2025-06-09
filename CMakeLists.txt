cmake_minimum_required(VERSION 2.8.3)
project(openpose_3d_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  openpose_ros_msgs
  message_filters
)

find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES openpose_3d_ros
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/openpose_3d.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(openpose_xtion src/openpose_xtion.cpp)
add_dependencies(openpose_xtion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(openpose_xtion
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )


add_executable(cap_depth src/cap_depth.cpp)
add_dependencies(cap_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cap_depth
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )
