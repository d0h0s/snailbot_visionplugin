//
// Created by xianghong on 9/19/24.
//

#include "aruco_image_preprocessing_nodelet/aruco_image_preprocessing_nodelet.h"

namespace aruco_image_preprocessing
{

void ArucoImagePreprocessingNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& nh_private = getPrivateNodeHandle();

  // ����ͼ����
  std::string image_topic;
  nh_private.param("image_topic", image_topic, std::string("/camera/rgb/image_raw"));
  image_sub_ = nh.subscribe(image_topic, 10, &ArucoImagePreprocessingNodelet::imageCallback, this);

  // ����Ԥ������ͼ��
  image_pub_ = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_preprocessed", 10);
  // ��̬���η�����
  dr_srv_ = boost::make_shared<dynamic_reconfigure::Server<aruco_image_preprocessing_nodelet::ArucoPreprocessingConfig>>(nh_private);
  dynamic_reconfigure::Server<aruco_image_preprocessing_nodelet::ArucoPreprocessingConfig>::CallbackType cb;
  cb = boost::bind(&ArucoImagePreprocessingNodelet::configCallback, this, _1, _2);
  dr_srv_->setCallback(cb);
  // ��ȡ����
  nh_private.param("alpha", alpha_, 1.5);  // Ĭ�϶Աȶȵ�������
  nh_private.param("median_blur_size", median_blur_size_, 5);  // ��ֵ�˲���С
  nh_private.param("block_size", block_size_, 11);  // ����Ӧ��ֵ���Ŀ��С
  nh_private.param("C", C_, 2.0);  // ����Ӧ��ֵ���ĳ���
}

void ArucoImagePreprocessingNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // �� ROS ͼ��ת��Ϊ OpenCV ��ʽ
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // 1. �ҶȻ�
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    // 2. ֱ��ͼ���⻯
    cv::Mat equalized_image;
    cv::equalizeHist(gray_image, equalized_image);

    // 3. ����Ӧ��ֵ��
    cv::Mat threshold_image;
    cv::adaptiveThreshold(equalized_image, threshold_image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, block_size_, C_);

    // 4. ��ֵ�˲�ȥ��
    cv::Mat filtered_image;
    cv::medianBlur(threshold_image, filtered_image, median_blur_size_);

    // ����������ͼ��
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;  // Ԥ������ͼ���ǵ�ͨ��
    out_msg.image = filtered_image;
    image_pub_.publish(out_msg.toImageMsg());
  }
  catch (const cv::Exception& e)
  {
    ROS_ERROR("OpenCV exception: %s", e.what());
  }
}

void ArucoImagePreprocessingNodelet::configCallback(aruco_image_preprocessing_nodelet::ArucoPreprocessingConfig &config, uint32_t level)
{
  alpha_ = config.alpha;
  median_blur_size_ = config.median_blur_size;
  block_size_ = config.block_size;
  C_ = config.C;
}

}  // namespace aruco_image_preprocessing
PLUGINLIB_EXPORT_CLASS(aruco_image_preprocessing::ArucoImagePreprocessingNodelet, nodelet::Nodelet)
