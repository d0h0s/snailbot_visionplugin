//
// Created by xianghong on 9/19/24.
//
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <map>
#include <vector>
#include <unordered_map>
#include <actionlib/client/simple_action_client.h>
#include <snailbot_msgs/ConnectedRelationAction.h>

namespace aruco_pose {

class ArucoPoseNodelet : public nodelet::Nodelet {
public:
  ArucoPoseNodelet();
  ~ArucoPoseNodelet() override;

  void onInit() override;
  void sendGoal(int robot_id, int robots_num);  // 发送目标
private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void drawImage(cv::Vec3d rvec, cv::Vec3d tvec, cv::Mat image, int id, std::vector<cv::Point2f> corner);

  // Helper functions
  void loadCameraParams(ros::NodeHandle& nh);
  std::vector<std::vector<cv::Point2f>> detectAndSmoothMarkers(cv::Mat& image, std::vector<int>& ids);
  std::vector<cv::Point2f> smoothMarkerCorners(int marker_id, const std::vector<cv::Point2f>& corners);
  void processMarkerPose(int marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec, std::vector<geometry_msgs::TransformStamped>& sphere_transforms, cv::Mat& image, const std::vector<cv::Point2f>& smoothed_corners);
  bool convertPoseToTransform(int marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec, geometry_msgs::TransformStamped& transformStamped);
  void publishSphereCenter(const std::vector<geometry_msgs::TransformStamped>& sphere_transforms, cv::Mat& image);
  void publishSphereSTL(const geometry_msgs::TransformStamped& sphere_center_transform);
  geometry_msgs::TransformStamped computeAverageTransform(const std::vector<geometry_msgs::TransformStamped>& transforms);
  void publishDebugImage(const std_msgs::Header& header, const cv::Mat& image);
  void getArucoPosesList(const XmlRpc::XmlRpcValue& aruco_poses_list);

  void doneCallback(const actionlib::SimpleClientGoalState& state, const snailbot_msgs::ConnectedRelationResultConstPtr& result){}
  void feedbackCallback(const snailbot_msgs::ConnectedRelationFeedbackConstPtr& feedback);

  actionlib::SimpleActionClient<snailbot_msgs::ConnectedRelationAction> ac_;
  ros::Subscriber image_sub_;
  ros::Publisher aruco_debug_pub_, marker_pub_;
  tf2_ros::TransformBroadcaster br_;
  cv::Mat camera_matrix_, dist_coeffs_;
  double real_marker_width_ = 0., alpha_ = 0., robot_radius_ = 0.;
  std::string robot_name_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener;
  std::map<int, std::vector<class WeightedAverageFilter2D>> marker_filters;
  std::unordered_map<int, geometry_msgs::TransformStamped> aruco_poses_list_{};
  int robots_num_ = 0;
  bool is_connecting_ = false;
};

// 加权平均滤波器
class WeightedAverageFilter2D {
public:
  WeightedAverageFilter2D(double alpha = 0.8) : alpha_(alpha), initialized_(false) {}

  cv::Point2f update(const cv::Point2f& pt) {
    if (!initialized_) {
      smoothed_pt_ = pt;
      initialized_ = true;
    } else {
      smoothed_pt_.x = alpha_ * smoothed_pt_.x + (1.0 - alpha_) * pt.x;
      smoothed_pt_.y = alpha_ * smoothed_pt_.y + (1.0 - alpha_) * pt.y;
    }
    return smoothed_pt_;
  }

private:
  double alpha_;
  bool initialized_;
  cv::Point2f smoothed_pt_;
};

} // namespace aruco_pose
