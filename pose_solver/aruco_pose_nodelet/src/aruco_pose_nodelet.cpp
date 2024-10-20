//
// Created by xianghong on 9/19/24.
//
#include "aruco_pose/aruco_pose_nodelet.h"

namespace aruco_pose
{
ArucoPoseNodelet::ArucoPoseNodelet() : ac_("check_connection_action", true), tf_listener(tf_buffer_)
{
  if (!ac_.waitForServer(ros::Duration(10.0)))
    ROS_ERROR("Action server not available after waiting for 10 seconds.");
  else
    ROS_INFO("Connected to action server.");
}

ArucoPoseNodelet::~ArucoPoseNodelet() {}

void ArucoPoseNodelet::onInit() {
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& nh_private = getPrivateNodeHandle();
  ros::NodeHandle aruco_poses_nh(nh, "aruco_poses");
  // 从参数服务器中获取相机参数
  loadCameraParams(nh);
  nh_private.getParam("robot_name", robot_name_);
  XmlRpc::XmlRpcValue aruco_poses_list;
  aruco_poses_nh.getParam(robot_name_, aruco_poses_list);
  getArucoPosesList(aruco_poses_list);
  std::string image_topic;
  nh_private.param("image_topic", image_topic, std::string("/camera/rgb/image_raw"));
  nh_private.param("real_marker_width", real_marker_width_, 16.0);
  nh_private.param("alpha", alpha_, 0.8);
  nh_private.param("robot_radius", robot_radius_, 60.0);
  nh_private.param("robots_num", robots_num_, 3);
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
    pose.header.frame_id = robot_name_;
    pose.child_frame_id = "target_" + std::to_string(static_cast<int>(aruco_pose.second["id"]));
    aruco_poses_list_.insert(std::make_pair(static_cast<int>(aruco_pose.second["id"]), pose));
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

      std::vector<geometry_msgs::TransformStamped> sphere_transforms;
      for (size_t i = 0; i < ids.size(); i++)
        processMarkerPose(ids[i], rvecs[i], tvecs[i], sphere_transforms, image, smoothed_corners_all[i]);

      // 计算并发布球心的位姿
      if (!sphere_transforms.empty())
        publishSphereCenter(sphere_transforms, image);
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
                                         std::vector<geometry_msgs::TransformStamped>& sphere_transforms,
                                         cv::Mat& image, const std::vector<cv::Point2f>& smoothed_corners)
{
  geometry_msgs::TransformStamped aruco_transform;
  if (!convertPoseToTransform(marker_id, rvec, tvec, aruco_transform)) return;
  br_.sendTransform(aruco_transform);
  // 绘制Aruco标记框
  drawImage(rvec, tvec, image, marker_id, smoothed_corners);
  // 从参数服务器中获取 center2target（Aruco 码相对于球心的变换）
  auto it = aruco_poses_list_.find(marker_id);
  if (it == aruco_poses_list_.end()) {
    ROS_WARN("No center2target pose found for marker ID %d", marker_id);
    return;
  }

  // 获取 center2target (Aruco 码相对于球心的位姿)
  geometry_msgs::TransformStamped center2target = it->second;

  // 将 center2target 转换为 target2center (球心相对于Aruco 码的位姿，求逆变换)
  geometry_msgs::TransformStamped target2center;
  tf2::Transform center2target_tf;
  tf2::fromMsg(center2target.transform, center2target_tf);
  tf2::Transform target2center_tf = center2target_tf.inverse();

  // 将逆变换结果存储回 target2center
  target2center.transform = tf2::toMsg(target2center_tf);
  target2center.header.frame_id = aruco_transform.child_frame_id;
  target2center.child_frame_id = robot_name_;
  // 将球心的位姿从Aruco码坐标系转换到相机坐标系
  geometry_msgs::TransformStamped center_in_camera;
  center_in_camera.header.frame_id = "camera_rgb_frame";
  center_in_camera.child_frame_id = robot_name_;

  try {
    tf2::doTransform(target2center, center_in_camera, aruco_transform);
    sphere_transforms.push_back(center_in_camera);
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
void ArucoPoseNodelet::publishSphereCenter(const std::vector<geometry_msgs::TransformStamped>& sphere_transforms, cv::Mat& image)
{
  // 平均每个标记的平移和旋转
  geometry_msgs::TransformStamped avg_transform = computeAverageTransform(sphere_transforms);
  br_.sendTransform(avg_transform);
  publishSphereSTL(avg_transform);
  // 在图像上绘制球心
  std::vector<cv::Point2f> projected_point;
  cv::projectPoints(std::vector<cv::Point3f>{cv::Point3f(avg_transform.transform.translation.x,
                                                         avg_transform.transform.translation.y,
                                                         avg_transform.transform.translation.z)},
                    cv::Mat::zeros(3, 1, CV_64F),  // 旋转向量
                    cv::Mat::zeros(3, 1, CV_64F),  // 平移向量
                    camera_matrix_, dist_coeffs_, projected_point);

  if (!projected_point.empty())
  {
    cv::circle(image, projected_point[0], 5, cv::Scalar(0, 255, 0), -1);  // 绿色表示球心
  }
}

// 计算位姿的平均值
geometry_msgs::TransformStamped ArucoPoseNodelet::computeAverageTransform(const std::vector<geometry_msgs::TransformStamped>& transforms)
{
  geometry_msgs::TransformStamped avg_transform;
  avg_transform.transform.translation.x = 0.0;
  avg_transform.transform.translation.y = 0.0;
  avg_transform.transform.translation.z = 0.0;
  avg_transform.header.stamp = ros::Time::now();
  avg_transform.header.frame_id = "camera_rgb_frame";
  avg_transform.child_frame_id = robot_name_;
  for (const auto& transform : transforms)
  {
    avg_transform.transform.translation.x += transform.transform.translation.x;
    avg_transform.transform.translation.y += transform.transform.translation.y;
    avg_transform.transform.translation.z += transform.transform.translation.z;
  }

  size_t count = transforms.size();
  avg_transform.transform.translation.x /= count;
  avg_transform.transform.translation.y /= count;
  avg_transform.transform.translation.z /= count;

  // 计算姿态部分的平均值（四元数）
  tf2::Quaternion avg_quaternion(0, 0, 0, 0);  // 初始化累加器为零四元数
  for (const auto& transform : transforms) {
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    avg_quaternion += q;  // 对四元数进行累加
  }
  avg_quaternion = avg_quaternion * (1.0 / transforms.size());
  avg_quaternion.normalize();  // 归一化，确保结果为单位四元数

  avg_transform.transform.rotation.x = avg_quaternion.x();
  avg_transform.transform.rotation.y = avg_quaternion.y();
  avg_transform.transform.rotation.z = avg_quaternion.z();
  avg_transform.transform.rotation.w = avg_quaternion.w();

  return avg_transform;
}

// 发布球心的STL文件并显示其姿态
void ArucoPoseNodelet::publishSphereSTL(const geometry_msgs::TransformStamped& sphere_center_transform)
{
  visualization_msgs::Marker marker;
  marker.ns = robot_name_;
  marker.header.frame_id = sphere_center_transform.header.frame_id;
  marker.header.stamp = ros::Time::now();

  // 定义Marker类型为MESH_RESOURCE，用于加载STL文件
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;

  // 设置球体的位置和姿态
  marker.pose.position.x = sphere_center_transform.transform.translation.x;
  marker.pose.position.y = sphere_center_transform.transform.translation.y;
  marker.pose.position.z = sphere_center_transform.transform.translation.z;
  marker.pose.orientation = sphere_center_transform.transform.rotation;

  // 设置球体的尺寸（缩放比例）
  marker.scale.x = 0.001; // 根据半径设置缩放
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;

  size_t underscore_pos = robot_name_.find_last_of('_');
  std::string number_str = robot_name_.substr(underscore_pos + 1);
  int robot_id = std::stoi(number_str);
  sendGoal(robot_id, robots_num_);
  ROS_INFO_STREAM("is_connecting_: " << is_connecting_);
  // 设置颜色
  if (!is_connecting_)
  {
    marker.color.r = 1.0; // 红色
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // 不透明
  }
  else
  {
    marker.color.r = 0.0; // 绿色
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // 不透明
  }

  // 设置STL文件路径
  marker.mesh_resource = "package://aruco_pose_nodelet/meshes/MODEL3.STL";
  marker.lifetime = ros::Duration(0.0);
  // 发布Marker
  marker_pub_.publish(marker);
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
  // 使用 projectPoints 将姿态轴投影到图像上
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

void ArucoPoseNodelet::sendGoal(int robot_id, int robots_num)
{
  snailbot_msgs::ConnectedRelationGoal goal;
  goal.robot_id = robot_id;  // 设置机器人ID
  goal.robots_num = robots_num;  // 设置机器人的总数

  // 发送目标
  ac_.sendGoal(goal,
               boost::bind(&ArucoPoseNodelet::doneCallback, this, _1, _2),  // 使用boost::bind将成员函数作为回调
               actionlib::SimpleActionClient<snailbot_msgs::ConnectedRelationAction>::SimpleActiveCallback(),
               boost::bind(&ArucoPoseNodelet::feedbackCallback, this, _1));
}

void ArucoPoseNodelet::feedbackCallback(const snailbot_msgs::ConnectedRelationFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM(robot_name_ <<" Feedback received: " << static_cast<int>(feedback->connecting_object_id));
  is_connecting_ = feedback->connecting_object_id != 255;
}

}  // namespace aruco_pose
PLUGINLIB_EXPORT_CLASS(aruco_pose::ArucoPoseNodelet, nodelet::Nodelet)
