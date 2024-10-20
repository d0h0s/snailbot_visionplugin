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
  // �Ӳ����������л�ȡ�������
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

  // ��������ǵ�ͼ���marker
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

// �����������
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

// ����ͼ��Ļص�����
void ArucoPoseNodelet::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // ���Aruco���
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

      // ���㲢�������ĵ�λ��
      if (!sphere_transforms.empty())
        publishSphereCenter(sphere_transforms, image);
    }

    // ��������ǵĵ���ͼ��
    publishDebugImage(msg->header, image);
  } catch (const cv::Exception &e){
    ROS_ERROR("OpenCV exception: %s", e.what());
  }
}

// ��Ⲣƽ��Aruco���
std::vector<std::vector<cv::Point2f>> ArucoPoseNodelet::detectAndSmoothMarkers(cv::Mat& image, std::vector<int>& ids)
{
  cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  std::vector<std::vector<cv::Point2f>> corners;

  // ���Aruco���
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

    // ƽ��ÿ���ǵ�
    std::vector<cv::Point2f> smoothed_corners = smoothMarkerCorners(ids[i], corners[i]);
    smoothed_corners_all.push_back(smoothed_corners);
  }
  return smoothed_corners_all;
}

// ƽ��ÿ����ǵĽǵ�
std::vector<cv::Point2f> ArucoPoseNodelet::smoothMarkerCorners(int marker_id, const std::vector<cv::Point2f>& corners)
{
  if (marker_filters.find(marker_id) == marker_filters.end())
    marker_filters[marker_id] = std::vector<WeightedAverageFilter2D>(4, WeightedAverageFilter2D(alpha_));

  std::vector<cv::Point2f> smoothed_corners(4);
  for (size_t j = 0; j < 4; j++)
    smoothed_corners[j] = marker_filters[marker_id][j].update(corners[j]);
  return smoothed_corners;
}

// ����ÿ����ǵ���̬������TF
void ArucoPoseNodelet::processMarkerPose(int marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                         std::vector<geometry_msgs::TransformStamped>& sphere_transforms,
                                         cv::Mat& image, const std::vector<cv::Point2f>& smoothed_corners)
{
  geometry_msgs::TransformStamped aruco_transform;
  if (!convertPoseToTransform(marker_id, rvec, tvec, aruco_transform)) return;
  br_.sendTransform(aruco_transform);
  // ����Aruco��ǿ�
  drawImage(rvec, tvec, image, marker_id, smoothed_corners);
  // �Ӳ����������л�ȡ center2target��Aruco ����������ĵı任��
  auto it = aruco_poses_list_.find(marker_id);
  if (it == aruco_poses_list_.end()) {
    ROS_WARN("No center2target pose found for marker ID %d", marker_id);
    return;
  }

  // ��ȡ center2target (Aruco ����������ĵ�λ��)
  geometry_msgs::TransformStamped center2target = it->second;

  // �� center2target ת��Ϊ target2center (���������Aruco ���λ�ˣ�����任)
  geometry_msgs::TransformStamped target2center;
  tf2::Transform center2target_tf;
  tf2::fromMsg(center2target.transform, center2target_tf);
  tf2::Transform target2center_tf = center2target_tf.inverse();

  // ����任����洢�� target2center
  target2center.transform = tf2::toMsg(target2center_tf);
  target2center.header.frame_id = aruco_transform.child_frame_id;
  target2center.child_frame_id = robot_name_;
  // �����ĵ�λ�˴�Aruco������ϵת�����������ϵ
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

// ����ǵ�λ��ת��ΪTransform
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

// �������ĵ�TF��2DͶӰ
void ArucoPoseNodelet::publishSphereCenter(const std::vector<geometry_msgs::TransformStamped>& sphere_transforms, cv::Mat& image)
{
  // ƽ��ÿ����ǵ�ƽ�ƺ���ת
  geometry_msgs::TransformStamped avg_transform = computeAverageTransform(sphere_transforms);
  br_.sendTransform(avg_transform);
  publishSphereSTL(avg_transform);
  // ��ͼ���ϻ�������
  std::vector<cv::Point2f> projected_point;
  cv::projectPoints(std::vector<cv::Point3f>{cv::Point3f(avg_transform.transform.translation.x,
                                                         avg_transform.transform.translation.y,
                                                         avg_transform.transform.translation.z)},
                    cv::Mat::zeros(3, 1, CV_64F),  // ��ת����
                    cv::Mat::zeros(3, 1, CV_64F),  // ƽ������
                    camera_matrix_, dist_coeffs_, projected_point);

  if (!projected_point.empty())
  {
    cv::circle(image, projected_point[0], 5, cv::Scalar(0, 255, 0), -1);  // ��ɫ��ʾ����
  }
}

// ����λ�˵�ƽ��ֵ
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

  // ������̬���ֵ�ƽ��ֵ����Ԫ����
  tf2::Quaternion avg_quaternion(0, 0, 0, 0);  // ��ʼ���ۼ���Ϊ����Ԫ��
  for (const auto& transform : transforms) {
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    avg_quaternion += q;  // ����Ԫ�������ۼ�
  }
  avg_quaternion = avg_quaternion * (1.0 / transforms.size());
  avg_quaternion.normalize();  // ��һ����ȷ�����Ϊ��λ��Ԫ��

  avg_transform.transform.rotation.x = avg_quaternion.x();
  avg_transform.transform.rotation.y = avg_quaternion.y();
  avg_transform.transform.rotation.z = avg_quaternion.z();
  avg_transform.transform.rotation.w = avg_quaternion.w();

  return avg_transform;
}

// �������ĵ�STL�ļ�����ʾ����̬
void ArucoPoseNodelet::publishSphereSTL(const geometry_msgs::TransformStamped& sphere_center_transform)
{
  visualization_msgs::Marker marker;
  marker.ns = robot_name_;
  marker.header.frame_id = sphere_center_transform.header.frame_id;
  marker.header.stamp = ros::Time::now();

  // ����Marker����ΪMESH_RESOURCE�����ڼ���STL�ļ�
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;

  // ���������λ�ú���̬
  marker.pose.position.x = sphere_center_transform.transform.translation.x;
  marker.pose.position.y = sphere_center_transform.transform.translation.y;
  marker.pose.position.z = sphere_center_transform.transform.translation.z;
  marker.pose.orientation = sphere_center_transform.transform.rotation;

  // ��������ĳߴ磨���ű�����
  marker.scale.x = 0.001; // ���ݰ뾶��������
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;

  size_t underscore_pos = robot_name_.find_last_of('_');
  std::string number_str = robot_name_.substr(underscore_pos + 1);
  int robot_id = std::stoi(number_str);
  sendGoal(robot_id, robots_num_);
  ROS_INFO_STREAM("is_connecting_: " << is_connecting_);
  // ������ɫ
  if (!is_connecting_)
  {
    marker.color.r = 1.0; // ��ɫ
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // ��͸��
  }
  else
  {
    marker.color.r = 0.0; // ��ɫ
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // ��͸��
  }

  // ����STL�ļ�·��
  marker.mesh_resource = "package://aruco_pose_nodelet/meshes/MODEL3.STL";
  marker.lifetime = ros::Duration(0.0);
  // ����Marker
  marker_pub_.publish(marker);
}

// ��������ͼ��
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
  // ���Ƽ�⵽��Aruco��ǵĽǵ�Ϳ�
  for (size_t j = 0; j < 4; j++) {
    cv::line(image, corner[j], corner[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
    cv::circle(image, corner[j], 4, cv::Scalar(0, 0, 255), -1);
  }
  // ʹ�� projectPoints ����̬��ͶӰ��ͼ����
  std::vector<cv::Point2f> projected_points;
  std::vector<cv::Point3f> axis_points;
  float axis_length = 30.0;  // ���� 0.3 ��

  axis_points.emplace_back(0, 0, 0); // ����ԭ��
  axis_points.emplace_back(axis_length, 0, 0); // X��
  axis_points.emplace_back(0, axis_length, 0); // Y��
  axis_points.emplace_back(0, 0, axis_length); // Z��

  cv::projectPoints(axis_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);

  // ��������
  cv::line(image, projected_points[0], projected_points[1], cv::Scalar(0, 0, 255), 2); // X�ᣬ��ɫ
  cv::line(image, projected_points[0], projected_points[2], cv::Scalar(0, 255, 0), 2); // Y�ᣬ��ɫ
  cv::line(image, projected_points[0], projected_points[3], cv::Scalar(255, 0, 0), 2); // Z�ᣬ��ɫ

  // ��ӡ������Ϣ�����ͶӰ���������
  ROS_INFO("Projected points for axes: X (%.2f, %.2f), Y (%.2f, %.2f), Z (%.2f, %.2f)",
           projected_points[1].x, projected_points[1].y,
           projected_points[2].x, projected_points[2].y,
           projected_points[3].x, projected_points[3].y);

  // ��ʾ��ǵ�id
  cv::putText(image, std::to_string(id), corner[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
}

void ArucoPoseNodelet::sendGoal(int robot_id, int robots_num)
{
  snailbot_msgs::ConnectedRelationGoal goal;
  goal.robot_id = robot_id;  // ���û�����ID
  goal.robots_num = robots_num;  // ���û����˵�����

  // ����Ŀ��
  ac_.sendGoal(goal,
               boost::bind(&ArucoPoseNodelet::doneCallback, this, _1, _2),  // ʹ��boost::bind����Ա������Ϊ�ص�
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
