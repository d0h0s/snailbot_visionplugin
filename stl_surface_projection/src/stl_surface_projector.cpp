//
// Created by xianghong on 10/22/24.
//
#include "stl_surface_projection/stl_surface_projector.h"

STLSurfaceProjector::STLSurfaceProjector(ros::NodeHandle& nh)
{
  projected_path_pub_ = nh.advertise<nav_msgs::Path>("projected_path", 10);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  is_mesh_loaded_ = false;

  nh.getParam("/stl_surface_projection_node/model_frames", model_frames_);
  ROS_INFO_STREAM("Model frames: " << model_frames_.size() << " frames loaded");
  for (const auto& frame_id : model_frames_)
  {
    tf::StampedTransform transform;
    try
    {
      tf_listener_.waitForTransform("world", frame_id, ros::Time(0), ros::Duration(1.0));
      tf_listener_.lookupTransform("world", frame_id, ros::Time(0), transform);
      model_transforms_.push_back(transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("Failed to get transform from world to %s: %s", frame_id.c_str(), ex.what());
    }
  }
}
void STLSurfaceProjector::createGridMesh(double width, double height, int rows, int cols, shape_msgs::Mesh& grid_mesh)
{
  // Calculate step size for vertices
  double step_x = width / (cols - 1);
  double step_y = height / (rows - 1);

  // Create vertices
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      geometry_msgs::Point vertex;
      vertex.x = j * step_x - (width / 2);  // Center the grid on the x-axis
      vertex.y = i * step_y - (height / 2); // Center the grid on the y-axis
      vertex.z = 0;  // Flat grid on the z = 0 plane
      grid_mesh.vertices.push_back(vertex);
    }
  }

  // Create triangles (two triangles per grid cell)
  for (int i = 0; i < rows - 1; ++i)
  {
    for (int j = 0; j < cols - 1; ++j)
    {
      int idx0 = i * cols + j;
      int idx1 = idx0 + 1;
      int idx2 = idx0 + cols;
      int idx3 = idx2 + 1;

      // First triangle (v0, v2, v1)
      shape_msgs::MeshTriangle tri1;
      tri1.vertex_indices[0] = idx0;
      tri1.vertex_indices[1] = idx2;
      tri1.vertex_indices[2] = idx1;
      grid_mesh.triangles.push_back(tri1);

      // Second triangle (v1, v2, v3)
      shape_msgs::MeshTriangle tri2;
      tri2.vertex_indices[0] = idx1;
      tri2.vertex_indices[1] = idx2;
      tri2.vertex_indices[2] = idx3;
      grid_mesh.triangles.push_back(tri2);
    }
  }
}

void STLSurfaceProjector::scaleMesh(shape_msgs::Mesh& mesh_msg, double scale_factor)
{
  for (auto& vertex : mesh_msg.vertices)
  {
    vertex.x *= scale_factor;
    vertex.y *= scale_factor;
    vertex.z *= scale_factor;
  }
}
void STLSurfaceProjector::loadSTLModel(const std::string& stl_path)
{
  shapes::Mesh* mesh = shapes::createMeshFromResource(stl_path);
  if (!mesh)
  {
    ROS_ERROR("Failed to load STL file from %s", stl_path.c_str());
    return;
  }

  shape_msgs::Mesh mesh_msg;
  shapes::ShapeMsg mesh_msg_tmp;
  shapes::constructMsgFromShape(mesh, mesh_msg_tmp);
  mesh_msg = boost::get<shape_msgs::Mesh>(mesh_msg_tmp);
  scaleMesh(mesh_msg, 0.001);
  merged_mesh_.vertices.clear();
  merged_mesh_.triangles.clear();

  for (size_t i = 0; i < model_transforms_.size(); ++i)
  {
    shape_msgs::Mesh transformed_mesh = mesh_msg;
    applyTransformToMesh(transformed_mesh, model_transforms_[i]);

    size_t vertex_offset = merged_mesh_.vertices.size();
    merged_mesh_.vertices.insert(merged_mesh_.vertices.end(),
                                 transformed_mesh.vertices.begin(),
                                 transformed_mesh.vertices.end());

    for (const auto& triangle : transformed_mesh.triangles)
    {
      shape_msgs::MeshTriangle new_triangle;
      for (int j = 0; j < 3; ++j)
      {
        new_triangle.vertex_indices[j] = triangle.vertex_indices[j] + vertex_offset;
      }
      merged_mesh_.triangles.push_back(new_triangle);
    }
    // 发布每个带有位姿的 STL 模型的 Marker
    publishMeshMarker(stl_path, model_transforms_[i], i);
  }

    shape_msgs::Mesh grid_mesh;
    createGridMesh(10.0, 10.0, 10, 10, grid_mesh);

    size_t grid_vertex_offset = merged_mesh_.vertices.size();
    merged_mesh_.vertices.insert(merged_mesh_.vertices.end(),
                                 grid_mesh.vertices.begin(),
                                 grid_mesh.vertices.end());

    for (const auto& triangle : grid_mesh.triangles)
    {
        shape_msgs::MeshTriangle new_triangle;
        for (int j = 0; j < 3; ++j)
        {
            new_triangle.vertex_indices[j] = triangle.vertex_indices[j] + grid_vertex_offset;
        }
        merged_mesh_.triangles.push_back(new_triangle);
    }
    publishGridMeshMarker(grid_mesh, model_transforms_.size());
    is_mesh_loaded_ = true;
    ROS_INFO("STL models and grid plane loaded and merged successfully");
}
void STLSurfaceProjector::publishGridMeshMarker(const shape_msgs::Mesh& grid_mesh, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // Assume the grid is in world frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "grid_mesh";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // Use triangle list for visualization
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 0.8f;

    // Add triangles from grid mesh
    for (const auto& triangle : grid_mesh.triangles)
    {
        geometry_msgs::Point p0, p1, p2;

        p0 = grid_mesh.vertices[triangle.vertex_indices[0]];
        p1 = grid_mesh.vertices[triangle.vertex_indices[1]];
        p2 = grid_mesh.vertices[triangle.vertex_indices[2]];

        // Add the vertices for this triangle
        marker.points.push_back(p0);
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    marker.lifetime = ros::Duration(0.0);  // Keep the marker displayed indefinitely
    marker_pub_.publish(marker);  // Publish the grid marker
}

void STLSurfaceProjector::publishMeshMarker(const std::string& stl_path, const tf::StampedTransform& transform, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";  // 发布到同一参考系
  marker.header.stamp = ros::Time::now();
  marker.ns = "stl_mesh";
  marker.id = id;  // 唯一标识符
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;

  // 设置 mesh 资源路径
  marker.mesh_resource = stl_path;  // 确保路径以 "file://" 或 "package://" 开头
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  tf::Quaternion q = transform.getRotation();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  // 设置缩放比例（根据 STL 文件的单位可能需要调整）
  marker.scale.x = 0.001;  // 默认1.0倍缩放，如果 STL 文件单位为毫米则调整为0.001
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;

  // 设置颜色
  marker.color.r = 0.0f;
  marker.color.g = 0.5f;
  marker.color.b = 0.5f;
  marker.color.a = 1.0f;  // 完全不透明
  marker.lifetime = ros::Duration(0.0);
  ROS_INFO_STREAM("Publishing mesh marker for " << stl_path << " with id " << id << "...");
  marker_pub_.publish(marker);  // 发布 Marker
}

void STLSurfaceProjector::applyTransformToMesh(shape_msgs::Mesh& mesh, const tf::StampedTransform& transform)
{
  const tf::Matrix3x3& rotation = transform.getBasis();
  tf::Vector3 translation = transform.getOrigin();

  for (auto& vertex : mesh.vertices)
  {
    tf::Vector3 point(vertex.x, vertex.y, vertex.z);
    tf::Vector3 transformed_point = rotation * point + translation;
    vertex.x = transformed_point.x();
    vertex.y = transformed_point.y();
    vertex.z = transformed_point.z();
  }
}

void STLSurfaceProjector::projectPathOntoSurface(const nav_msgs::Path& path_msg, nav_msgs::Path& projected_path)
{
    if (!is_mesh_loaded_)
    {
        ROS_ERROR("STL models not loaded. Cannot project path.");
        return;
    }

    projected_path.header = path_msg.header;
    projected_path.poses.clear();

    for (const auto& pose_stamped : path_msg.poses)
    {
        Eigen::Vector3d point(pose_stamped.pose.position.x,
                              pose_stamped.pose.position.y,
                              pose_stamped.pose.position.z);
        Eigen::Vector3d nearest_point;
        double min_distance = std::numeric_limits<double>::infinity();

        // 遍历合并后的 STL 模型的三角形，将路径点投影到最近的三角形上
        for (auto & triangle : merged_mesh_.triangles)
        {
            Eigen::Vector3d v0, v1, v2;
            v0 << merged_mesh_.vertices[triangle.vertex_indices[0]].x,
                  merged_mesh_.vertices[triangle.vertex_indices[0]].y,
                  merged_mesh_.vertices[triangle.vertex_indices[0]].z;
            v1 << merged_mesh_.vertices[triangle.vertex_indices[1]].x,
                  merged_mesh_.vertices[triangle.vertex_indices[1]].y,
                  merged_mesh_.vertices[triangle.vertex_indices[1]].z;
            v2 << merged_mesh_.vertices[triangle.vertex_indices[2]].x,
                  merged_mesh_.vertices[triangle.vertex_indices[2]].y,
                  merged_mesh_.vertices[triangle.vertex_indices[2]].z;

            // 调用 projectPointOntoTriangle 函数
            Eigen::Vector3d projected_point = projectPointOntoTriangle(point, v0, v1, v2);
            double distance = (projected_point - point).norm();

            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_point = projected_point;
            }
        }

        // 将投影点作为新的路径点
        geometry_msgs::PoseStamped projected_pose;
        projected_pose.header = path_msg.header;
        projected_pose.pose.position.x = nearest_point.x();
        projected_pose.pose.position.y = nearest_point.y();
        projected_pose.pose.position.z = nearest_point.z();
        projected_pose.pose.orientation = pose_stamped.pose.orientation;

        projected_path.poses.push_back(projected_pose);
    }

    // 发布投影后的路径
    projected_path_pub_.publish(projected_path);
    ROS_INFO_STREAM("Projected path published with " << projected_path.poses.size() << " poses");
}

Eigen::Vector3d STLSurfaceProjector::projectPointOntoTriangle(const Eigen::Vector3d& point,
                                                              const Eigen::Vector3d& v0,
                                                              const Eigen::Vector3d& v1,
                                                              const Eigen::Vector3d& v2)
{
    // Step 1: 计算三角形的法线向量
    Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0).normalized();

    // Step 2: 计算点到三角形平面的距离，并沿法线方向将其投影到平面上
    double distance_to_plane = (point - v0).dot(normal);
    Eigen::Vector3d projected_point = point - distance_to_plane * normal;

    // Step 3: 使用重心坐标法检查投影点是否在三角形内部
    Eigen::Vector3d v0v1 = v1 - v0;
    Eigen::Vector3d v0v2 = v2 - v0;
    Eigen::Vector3d v0p = projected_point - v0;

    double d00 = v0v1.dot(v0v1);
    double d01 = v0v1.dot(v0v2);
    double d11 = v0v2.dot(v0v2);
    double d20 = v0p.dot(v0v1);
    double d21 = v0p.dot(v0v2);

    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;

    // Step 4: 如果投影点在三角形内部，则返回该点；否则，找到最近的边界点
    if (u >= 0 && v >= 0 && w >= 0)
    {
        return projected_point;  // 投影点在三角形内
    }
    else
    {
        // Step 5: 如果投影点在三角形外部，找到最近的边界上的点
        Eigen::Vector3d closest_point_on_edge;

        // 最近点在边 (v0, v1) 上
        Eigen::Vector3d closest_on_edge_v0v1 = closestPointOnSegment(projected_point, v0, v1);
        double dist_to_v0v1 = (closest_on_edge_v0v1 - projected_point).squaredNorm();

        // 最近点在边 (v1, v2) 上
        Eigen::Vector3d closest_on_edge_v1v2 = closestPointOnSegment(projected_point, v1, v2);
        double dist_to_v1v2 = (closest_on_edge_v1v2 - projected_point).squaredNorm();

        // 最近点在边 (v2, v0) 上
        Eigen::Vector3d closest_on_edge_v2v0 = closestPointOnSegment(projected_point, v2, v0);
        double dist_to_v2v0 = (closest_on_edge_v2v0 - projected_point).squaredNorm();

        // 找到最近的边
        if (dist_to_v0v1 < dist_to_v1v2 && dist_to_v0v1 < dist_to_v2v0)
        {
            closest_point_on_edge = closest_on_edge_v0v1;
        }
        else if (dist_to_v1v2 < dist_to_v2v0)
        {
            closest_point_on_edge = closest_on_edge_v1v2;
        }
        else
        {
            closest_point_on_edge = closest_on_edge_v2v0;
        }

        return closest_point_on_edge;
    }
}

Eigen::Vector3d STLSurfaceProjector::closestPointOnSegment(const Eigen::Vector3d& point,
                                                           const Eigen::Vector3d& seg_start,
                                                           const Eigen::Vector3d& seg_end)
{
    Eigen::Vector3d segment = seg_end - seg_start;
    double t = (point - seg_start).dot(segment) / segment.squaredNorm();
    t = clamp(t, 0.0, 1.0); // 保证 t 在 [0, 1] 范围内
    return seg_start + t * segment;
}
