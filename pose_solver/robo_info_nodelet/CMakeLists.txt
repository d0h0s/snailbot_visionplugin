cmake_minimum_required(VERSION 3.0.2)
project(robo_info_nodelet)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        pluginlib
        sensor_msgs
        image_transport
        cv_bridge
        tf2_ros
        tf2_geometry_msgs
        dynamic_reconfigure
        snailbot_msgs
)
generate_dynamic_reconfigure_options(
        cfg/RoboInfo.cfg
)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES robo_info_nodelet
        CATKIN_DEPENDS roscpp nodelet pluginlib sensor_msgs image_transport cv_bridge tf2_ros tf2_geometry_msgs
        snailbot_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${dynamic_reconfigure_PACKAGE_PATH}
)

add_library(robo_info_nodelet src/robo_info_nodelet.cpp)
add_dependencies(robo_info_nodelet ${PROJECT_NAME}_gencfg snailbot_msgs_generate_messages_cpp)
target_link_libraries(robo_info_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS robo_info_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES robo_info_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
