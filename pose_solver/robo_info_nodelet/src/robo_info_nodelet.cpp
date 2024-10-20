//
// Created by xianghong on 9/19/24.
//

#include "robo_info_nodelet/robo_info_nodelet.h"

namespace robo_info
{
RoboInfoNodelet::RoboInfoNodelet(){}
RoboInfoNodelet::~RoboInfoNodelet() {}
CheckConnectionAction::CheckConnectionAction(std::string name) :
  as_(nh_, name, false),
  action_name_(name), tf_listener(tf_buffer_),is_processing_(false)
{
  as_.registerGoalCallback(boost::bind(&CheckConnectionAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&CheckConnectionAction::preemptCB, this));
  as_.start();
}
CheckConnectionAction::~CheckConnectionAction() {}

void RoboInfoNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& nh_private = getPrivateNodeHandle();
  check_connection_action_ = std::make_shared<CheckConnectionAction>("check_connection_action");
}

void CheckConnectionAction::processNextGoal()
{
  while (!goal_queue_.empty())
  {
    ROS_INFO_STREAM("Processing next goal. Queue size: " << goal_queue_.size());
    snailbot_msgs::ConnectedRelationGoalConstPtr goal = goal_queue_.front();
    goal_queue_.pop();
    is_processing_ = true;

    feedback_.connecting_object_id = 255;
    geometry_msgs::TransformStamped target_transform_stamped;
    try {
      std::string target_frame = "robot_" + std::to_string(goal->robot_id);
      target_transform_stamped = tf_buffer_.lookupTransform( "camera_rgb_frame", target_frame,ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Could not get transform: %s", ex.what());
    }
    for(int i = 1; i <= goal->robots_num; i++)
    {
      if(i == goal->robot_id)
        continue;
      std::string target_frame = "robot_" + std::to_string(i);
      geometry_msgs::TransformStamped transform_stamped;
      try
      {
        transform_stamped = tf_buffer_.lookupTransform("camera_rgb_frame",target_frame, ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
      }
      ROS_INFO_STREAM("Target Frame: " << target_frame);
      ROS_INFO_STREAM("Target Translation: ["
                      << target_transform_stamped.transform.translation.x << ", "
                      << target_transform_stamped.transform.translation.y << ", "
                      << target_transform_stamped.transform.translation.z << "]");
      ROS_INFO_STREAM("Transform Frame Translation: ["
                      << transform_stamped.transform.translation.x << ", "
                      << transform_stamped.transform.translation.y << ", "
                      << transform_stamped.transform.translation.z << "]");
      // 计算两者之间的距离
      double dx = target_transform_stamped.transform.translation.x - transform_stamped.transform.translation.x;
      double dy = target_transform_stamped.transform.translation.y - transform_stamped.transform.translation.y;
      double dz = target_transform_stamped.transform.translation.z - transform_stamped.transform.translation.z;
      double distance = sqrt(dx * dx + dy * dy + dz * dz);
      int robot_id = goal->robot_id;
      ROS_INFO_STREAM("robot_id:"<< robot_id <<" i:"<<i<<" dx:"<<dx<<" dy:"<<dy<<" dz:"<<dz<<" distance:"<<distance);
      double connecting_distance = 0.1059;
      ROS_INFO_STREAM("fabs(distance - connecting_distance): " << fabs(distance - connecting_distance));
      if (fabs(distance - connecting_distance) > 0.015) // 距离误差允许范围为0.01米
        continue; // 距离不符合要求，进入下一个循环

      // 判断Z轴线是否穿过中心点
      // Z轴方向是四元数变换后的向量，使用rotation表示
      tf2::Quaternion quat;
      tf2::fromMsg(target_transform_stamped.transform.rotation, quat);
      tf2::Matrix3x3 rotation_matrix(quat);
      tf2::Vector3 z_axis_in_target = rotation_matrix * tf2::Vector3(0.0, 0.0, 1.0); // Z轴在目标坐标系中的方向

      tf2::Vector3 target_to_center(dx, dy, dz);// 计算目标到transform_stamped中心点的向量
      z_axis_in_target.normalize();
      target_to_center.normalize();
      // 判断Z轴向量是否穿过中心点，可以通过向量点积的符号判断
      double dot_product = z_axis_in_target.dot(target_to_center);
      ROS_INFO_STREAM("Dot product: " << fabs(dot_product));
      if (fabs(dot_product - 1.0) < 0.015) // 误差允许范围为0.01
      {
        feedback_.connecting_object_id = i;
        break;
      }
      else
        // Z轴未穿过中心点，进入下一个循环
        continue;
    }
    ROS_INFO_STREAM("Connecting object id: " << static_cast<int>(feedback_.connecting_object_id));
    as_.publishFeedback(feedback_);
    result_.success = true;
    as_.setSucceeded(result_);
    is_processing_ = false;
  }
}

}  // namespace robo_info
PLUGINLIB_EXPORT_CLASS(robo_info::RoboInfoNodelet, nodelet::Nodelet)
