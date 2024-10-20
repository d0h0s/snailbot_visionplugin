cmake_minimum_required(VERSION 3.10)
project(snailbot_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        actionlib
        actionlib_msgs
)

add_message_files(
        FILES
#        ActuatorState.msg
)

add_service_files(
        FILES

)

#add_message_files(
#        DIRECTORY msg/detection
#        FILES
#        PriorityArray.msg
#        TargetDetection.msg
#        TargetDetectionArray.msg
#        TrackData.msg
#)
add_action_files(
        FILES
        ConnectedRelation.action
)
# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        actionlib
        actionlib_msgs
)
