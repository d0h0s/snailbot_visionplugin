cmake_minimum_required(VERSION 3.0.2)
project(stl_surface_projection)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        nav_msgs
        visualization_msgs
        shape_msgs
        tf
        geometric_shapes
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES stl_surface_projection
        CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs shape_msgs tf
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

add_executable(stl_surface_projection_node src/stl_surface_projection_node.cpp src/stl_surface_projector.cpp)

target_link_libraries(stl_surface_projection_node
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)
