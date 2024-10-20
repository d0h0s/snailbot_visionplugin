//
// Created by xianghong on 7/21/24.
//

#pragma once

#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <robo_info_nodelet/RoboInfoConfig.h>

#include <actionlib/server/simple_action_server.h>
#include <snailbot_msgs/ConnectedRelationAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
namespace robo_info
{

class CheckConnectionAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<snailbot_msgs::ConnectedRelationAction> as_;
  std::string action_name_;
  snailbot_msgs::ConnectedRelationFeedback feedback_;
  snailbot_msgs::ConnectedRelationResult result_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener;
  std::queue<snailbot_msgs::ConnectedRelationGoalConstPtr> goal_queue_;  // 存储任务的队列
  bool is_processing_;  // 标记当前是否正在处理任务
public:
  CheckConnectionAction(std::string name);
  ~CheckConnectionAction() ;

  void goalCB()
  {
    goal_queue_.push(as_.acceptNewGoal());
    if (!is_processing_)
    {
      processNextGoal();
    }
  }
  void processNextGoal();
  // 处理抢占请求 (在这种情况下，将不使用抢占)
  void preemptCB()
  {
    ROS_INFO("%s: Preempted is ignored, tasks will be queued", action_name_.c_str());
  }
};

class RoboInfoNodelet : public nodelet::Nodelet
{
public:
  RoboInfoNodelet();
  ~RoboInfoNodelet() override;
  void onInit() override;
private:
  std::shared_ptr<CheckConnectionAction> check_connection_action_;
};

} // namespace robo_info

