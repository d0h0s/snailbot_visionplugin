cmake_minimum_required(VERSION 3.0.2)
project(aruco_image_preprocessing_nodelet)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        pluginlib
        sensor_msgs
        image_transport
        cv_bridge
        dynamic_reconfigure
)
generate_dynamic_reconfigure_options(
        cfg/ArucoPreprocessing.cfg
)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES aruco_image_preprocessing_nodelet
        CATKIN_DEPENDS roscpp nodelet pluginlib sensor_msgs image_transport cv_bridge
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${dynamic_reconfigure_PACKAGE_PATH}
)

add_library(aruco_image_preprocessing_nodelet src/aruco_image_preprocessing_nodelet.cpp)
add_dependencies(aruco_image_preprocessing_nodelet ${PROJECT_NAME}_gencfg)
target_link_libraries(aruco_image_preprocessing_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS aruco_image_preprocessing_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES aruco_image_preprocessing_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
