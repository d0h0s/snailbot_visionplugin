//
// Created by xianghong on 10/22/24.
//

#pragma
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class STLSurfaceProjector
{
public:
  STLSurfaceProjector(ros::NodeHandle& nh);
  void loadSTLModel(const std::string& stl_path);
  void projectPathOntoSurface(const nav_msgs::Path& path_msg, nav_msgs::Path& projected_path);

private:
  ros::Publisher projected_path_pub_,marker_pub_;
  shape_msgs::Mesh merged_mesh_;
  std::vector<tf::StampedTransform> model_transforms_;
  tf::TransformListener tf_listener_;
  bool is_mesh_loaded_;
  std::vector<std::string> model_frames_;
  Eigen::Vector3d projectPointOntoTriangle(const Eigen::Vector3d& point,
                                           const Eigen::Vector3d& v0,
                                           const Eigen::Vector3d& v1,
                                           const Eigen::Vector3d& v2);
  void applyTransformToMesh(shape_msgs::Mesh& mesh, const tf::StampedTransform& transform);
  void publishMeshMarker(const std::string& stl_path,const tf::StampedTransform& transform, int id);
  void publishGridMeshMarker(const shape_msgs::Mesh& grid_mesh, int id);
  void scaleMesh(shape_msgs::Mesh& mesh_msg, double scale_factor);
  Eigen::Vector3d closestPointOnSegment(const Eigen::Vector3d& point,
                                        const Eigen::Vector3d& seg_start,
                                        const Eigen::Vector3d& seg_end);
  void createGridMesh(double width, double height, int rows, int cols, shape_msgs::Mesh& grid_mesh);
  double clamp(double val, double min_val, double max_val)
  {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
  }
};