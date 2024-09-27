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
#include <aruco_image_preprocessing_nodelet/ArucoPreprocessingConfig.h>

namespace aruco_image_preprocessing
{

class ArucoImagePreprocessingNodelet : public nodelet::Nodelet
{
public:
  ArucoImagePreprocessingNodelet() {}
  ~ArucoImagePreprocessingNodelet() {}

private:
  virtual void onInit();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void configCallback(aruco_image_preprocessing_nodelet::ArucoPreprocessingConfig &config, uint32_t level);

  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;

  double alpha_;  // �Աȶȵ�������
  int median_blur_size_;  // ��ֵ�˲��˴�С
  int block_size_;  // ����Ӧ��ֵ���Ŀ��С
  double C_;  // ����Ӧ��ֵ������

  boost::shared_ptr<dynamic_reconfigure::Server<aruco_image_preprocessing_nodelet::ArucoPreprocessingConfig>> dr_srv_;
};

} // namespace aruco_pose

