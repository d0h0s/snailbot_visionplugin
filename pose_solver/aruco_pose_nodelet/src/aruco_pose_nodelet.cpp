//
// Created by xianghong on 9/19/24.
//
#include "aruco_pose/aruco_pose_nodelet.h"

namespace aruco_pose
{
ArucoPoseNodelet::ArucoPoseNodelet() : tf_listener(tf_buffer_) {}

ArucoPoseNodelet::~ArucoPoseNodelet() {}

void ArucoPoseNodelet::onInit() {
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& nh_private = getPrivateNodeHandle();

  // 从参数服务器中获取相机参数
  loadCameraParams(nh);
  XmlRpc::XmlRpcValue aruco_poses_list;
  nh.getParam("aruco_poses", aruco_poses_list);
  getArucoPosesList(aruco_poses_list);
  std::string image_topic;
  nh_private.param("image_topic", image_topic, std::string("/camera/rgb/image_raw"));
  nh_private.param("real_marker_width", real_marker_width_, 16.0);
  nh_private.param("alpha", alpha_, 0.8);
  nh_private.param("robot_radius", robot_radius_, 60.0);

  image_sub_ = nh.subscribe(image_topic, 10, &ArucoPoseNodelet::imageCallback, this);

  // 发布带标记的图像和marker
  aruco_debug_pub_ = nh.advertise<sensor_msgs::Image>("/camera/rgb/aruco_debug", 10);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("aruco_markers", 1);
}

void ArucoPoseNodelet::getArucoPosesList(const XmlRpc::XmlRpcValue& aruco_poses_list)
{
  for (const auto& aruco_pose : aruco_poses_list)
  {
    ROS_ASSERT(aruco_pose.second.hasMember("id") and aruco_pose.second.hasMember("translation") and
               aruco_pose.second.hasMember("rotation"));
    ROS_ASSERT(aruco_pose.second["id"].getType() == XmlRpc::XmlRpcValue::TypeInt and
               aruco_pose.second["translation"].getType() == XmlRpc::XmlRpcValue::TypeArray and
               aruco_pose.second["rotation"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    geometry_msgs::TransformStamped pose;
    pose.transform.translation = tf2::toMsg(tf2::Vector3(aruco_pose.second["translation"][0], aruco_pose.second["translation"][1], aruco_pose.second["translation"][2]));
    pose.transform.rotation = tf2::toMsg(tf2::Quaternion(aruco_pose.second["rotation"][0], aruco_pose.second["rotation"][1], aruco_pose.second["rotation"][2], aruco_pose.second["rotation"][3]));
    pose.header.frame_id = "sphere_center";
    pose.child_frame_id = "target_" + std::to_string(static_cast<int>(aruco_pose.second["id"]));
    aruco_poses_list_.insert(std::make_pair(aruco_pose.second["id"], pose));
  }
}

// 加载相机参数
void ArucoPoseNodelet::loadCameraParams(ros::NodeHandle& nh)
{
  std::vector<double> camera_matrix_data, dist_coeffs_data;
  nh.getParam("/camera_matrix/data", camera_matrix_data);
  nh.getParam("/distortion_coefficients/data", dist_coeffs_data);

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  dist_coeffs_ = cv::Mat(1, 5, CV_64F);

  for (int i = 0; i < 9; i++)
    camera_matrix_.at<double>(i / 3, i % 3) = camera_matrix_data[i];
  for (int i = 0; i < 5; i++)
    dist_coeffs_.at<double>(i) = dist_coeffs_data[i];
}

// 处理图像的回调函数
void ArucoPoseNodelet::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // 检测Aruco标记
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> smoothed_corners_all = detectAndSmoothMarkers(image, ids);

    if (!ids.empty())
    {
      ROS_INFO("Detected %d Aruco markers", (int)ids.size());

      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(smoothed_corners_all, (float)real_marker_width_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      std::vector<geometry_msgs::Point> points_in_camera_frame;
      for (size_t i = 0; i < ids.size(); i++)
        processMarkerPose(ids[i], rvecs[i], tvecs[i], points_in_camera_frame, image, smoothed_corners_all[i]);

      // 计算并发布球心的位置
      if (!points_in_camera_frame.empty())
        publishSphereCenter(points_in_camera_frame, image);

    }
    // 发布带标记的调试图像
    publishDebugImage(msg->header, image);
  } catch (const cv::Exception &e){
    ROS_ERROR("OpenCV exception: %s", e.what());
  }
}

// 检测并平滑Aruco标记
std::vector<std::vector<cv::Point2f>> ArucoPoseNodelet::detectAndSmoothMarkers(cv::Mat& image, std::vector<int>& ids)
{
  cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  std::vector<std::vector<cv::Point2f>> corners;

  // 检测Aruco标记
  cv::aruco::detectMarkers(image, aruco_dict, corners, ids);

  std::vector<std::vector<cv::Point2f>> smoothed_corners_all;
  smoothed_corners_all.reserve(ids.size());

  for (size_t i = 0; i < ids.size(); i++)
  {
    if (corners[i].size() != 4)
    {
      ROS_WARN("Invalid number of corners for marker %d", ids[i]);
      continue;
    }

    // 平滑每个角点
    std::vector<cv::Point2f> smoothed_corners = smoothMarkerCorners(ids[i], corners[i]);
    smoothed_corners_all.push_back(smoothed_corners);
  }
  return smoothed_corners_all;
}

// 平滑每个标记的角点
std::vector<cv::Point2f> ArucoPoseNodelet::smoothMarkerCorners(int marker_id, const std::vector<cv::Point2f>& corners)
{
  if (marker_filters.find(marker_id) == marker_filters.end())
    marker_filters[marker_id] = std::vector<WeightedAverageFilter2D>(4, WeightedAverageFilter2D(alpha_));

  std::vector<cv::Point2f> smoothed_corners(4);
  for (size_t j = 0; j < 4; j++)
    smoothed_corners[j] = marker_filters[marker_id][j].update(corners[j]);
  return smoothed_corners;
}

// 处理每个标记的姿态并发布TF
void ArucoPoseNodelet::processMarkerPose(int marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                         std::vector<geometry_msgs::Point>& points_in_camera_frame,
                                         cv::Mat& image, const std::vector<cv::Point2f>& smoothed_corners)
{
  geometry_msgs::TransformStamped transformStamped;
  if (!convertPoseToTransform(marker_id, rvec, tvec, transformStamped)) return;

  br_.sendTransform(transformStamped);
  drawImage(rvec, tvec, image, marker_id, smoothed_corners);

  // 将球心坐标转换到相机坐标系
  geometry_msgs::PointStamped marker_point, point_in_camera_frame;
  marker_point.point.x = 0.0;
  marker_point.point.y = 0.0;
  marker_point.point.z = -robot_radius_ / 1000.0;  // mm 转换为 m
  marker_point.header.frame_id = transformStamped.child_frame_id;

  try {
    tf2::doTransform(marker_point, point_in_camera_frame, transformStamped);
    points_in_camera_frame.push_back(point_in_camera_frame.point);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Transformation failed: %s", ex.what());
  }
}

// 将标记的位姿转换为Transform
bool ArucoPoseNodelet::convertPoseToTransform(int marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                              geometry_msgs::TransformStamped& transformStamped)
{
  cv::Mat rotation_matrix;
  cv::Rodrigues(rvec, rotation_matrix);
  tf2::Matrix3x3 tf_rot(rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

  tf2::Quaternion quaternion;
  tf_rot.getRotation(quaternion);

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "camera_rgb_frame";
  transformStamped.child_frame_id = "target_" + std::to_string(marker_id);
  transformStamped.transform.translation.x = tvec[0] / 1000.0;
  transformStamped.transform.translation.y = tvec[1] / 1000.0;
  transformStamped.transform.translation.z = tvec[2] / 1000.0;
  transformStamped.transform.rotation.x = quaternion.x();
  transformStamped.transform.rotation.y = quaternion.y();
  transformStamped.transform.rotation.z = quaternion.z();
  transformStamped.transform.rotation.w = quaternion.w();

  return true;
}

// 发布球心的TF和2D投影
void ArucoPoseNodelet::publishSphereCenter(const std::vector<geometry_msgs::Point>& points_in_camera_frame, cv::Mat& image)
{
  geometry_msgs::Point average_point = computeAveragePoint(points_in_camera_frame);
  publishSphereSTL(average_point);
  // 发布球心TF
  geometry_msgs::TransformStamped sphere_center_transform;
  sphere_center_transform.header.stamp = ros::Time::now();
  sphere_center_transform.header.frame_id = "camera_rgb_frame";
  sphere_center_transform.child_frame_id = "sphere_center";
  sphere_center_transform.transform.translation.x = average_point.x;
  sphere_center_transform.transform.translation.y = average_point.y;
  sphere_center_transform.transform.translation.z = average_point.z;
  sphere_center_transform.transform.rotation.w = 1.0;  // 单位四元数
  br_.sendTransform(sphere_center_transform);

  // 在图像上绘制球心
  std::vector<cv::Point2f> projected_point;
  cv::projectPoints(std::vector<cv::Point3f>{cv::Point3f(average_point.x, average_point.y, average_point.z)},
                    cv::Mat::zeros(3, 1, CV_64F),  // 旋转向量
                    cv::Mat::zeros(3, 1, CV_64F),  // 平移向量
                    camera_matrix_, dist_coeffs_, projected_point);

  if (!projected_point.empty())
  {
    cv::circle(image, projected_point[0], 5, cv::Scalar(0, 255, 0), -1);  // 绿色表示球心
  }
}

void ArucoPoseNodelet::publishSphereSTL(const geometry_msgs::Point& sphere_center)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_rgb_frame";
  marker.header.stamp = ros::Time::now();

  // 定义Marker类型为MESH_RESOURCE，用于加载STL文件
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;

  // 设置球体的位置
  marker.pose.position.x = sphere_center.x;
  marker.pose.position.y = sphere_center.y;
  marker.pose.position.z = sphere_center.z;

  // 设置旋转，单位四元数
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // 设置球体的尺寸（缩放比例），可以根据需要调整
  marker.scale.x = 0.001; // 根据半径设置缩放
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;

  // 设置颜色
  marker.color.r = 1.0; // 红色
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0; // 不透明

  // 设置STL文件路径（可以是官方资源）
  marker.mesh_resource = "package://aruco_pose_nodelet/meshes/MODEL3.STL";

  // 发布Marker
  marker_pub_.publish(marker);
}

// 计算球心的平均位置
geometry_msgs::Point ArucoPoseNodelet::computeAveragePoint(const std::vector<geometry_msgs::Point>& points)
{
  geometry_msgs::Point avg_point;
  avg_point.x = avg_point.y = avg_point.z = 0.0;

  for (const auto& point : points)
  {
    avg_point.x += point.x;
    avg_point.y += point.y;
    avg_point.z += point.z;
  }
  size_t count = points.size();
  avg_point.x /= count;
  avg_point.y /= count;
  avg_point.z /= count;
  return avg_point;
}

// 发布调试图像
void ArucoPoseNodelet::publishDebugImage(const std_msgs::Header& header, const cv::Mat& image)
{
  cv_bridge::CvImage debug_image_msg;
  debug_image_msg.header = header;
  debug_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  debug_image_msg.image = image;
  aruco_debug_pub_.publish(debug_image_msg.toImageMsg());
}

void ArucoPoseNodelet::drawImage(cv::Vec3d rvec, cv::Vec3d tvec, cv::Mat image, int id, std::vector<cv::Point2f> corner)
{
  // 绘制检测到的Aruco标记的角点和框
  for (size_t j = 0; j < 4; j++) {
    cv::line(image, corner[j], corner[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
    cv::circle(image, corner[j], 4, cv::Scalar(0, 0, 255), -1);
  }
  // 使用 projectPoints 将姿态轴投影到图像上s
  std::vector<cv::Point2f> projected_points;
  std::vector<cv::Point3f> axis_points;
  float axis_length = 30.0;  // 增大到 0.3 米

  axis_points.emplace_back(0, 0, 0); // 坐标原点
  axis_points.emplace_back(axis_length, 0, 0); // X轴
  axis_points.emplace_back(0, axis_length, 0); // Y轴
  axis_points.emplace_back(0, 0, axis_length); // Z轴

  cv::projectPoints(axis_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);

  // 绘制轴线
  cv::line(image, projected_points[0], projected_points[1], cv::Scalar(0, 0, 255), 2); // X轴，红色
  cv::line(image, projected_points[0], projected_points[2], cv::Scalar(0, 255, 0), 2); // Y轴，绿色
  cv::line(image, projected_points[0], projected_points[3], cv::Scalar(255, 0, 0), 2); // Z轴，蓝色

  // 打印调试信息：输出投影的坐标轴点
  ROS_INFO("Projected points for axes: X (%.2f, %.2f), Y (%.2f, %.2f), Z (%.2f, %.2f)",
           projected_points[1].x, projected_points[1].y,
           projected_points[2].x, projected_points[2].y,
           projected_points[3].x, projected_points[3].y);

  // 显示标记的id
  cv::putText(image, std::to_string(id), corner[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
}

}  // namespace aruco_pose

PLUGINLIB_EXPORT_CLASS(aruco_pose::ArucoPoseNodelet, nodelet::Nodelet)
