cmake_minimum_required(VERSION 3.0.2)
project(aruco_pose_nodelet)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        pluginlib
        sensor_msgs
        image_transport
        cv_bridge
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        snailbot_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES aruco_pose_nodelet
        CATKIN_DEPENDS roscpp nodelet pluginlib sensor_msgs tf2_ros tf2_geometry_msgs tf2_sensor_msgs geometry_msgs
        snailbot_msgs
)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(aruco_pose_nodelet src/aruco_pose_nodelet.cpp)
#add_dependencies(aruco_pose_nodelet snailbot_msgs_generate_messages_cpp)
target_link_libraries(aruco_pose_nodelet
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

install(TARGETS aruco_pose_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES aruco_pose_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
