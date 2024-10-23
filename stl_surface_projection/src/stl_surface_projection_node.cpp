//
// Created by xianghong on 10/22/24.
//

#include "stl_surface_projection/stl_surface_projector.h"

std::string STL_FILE_PATH;
STLSurfaceProjector* PROJECTOR = nullptr;

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{

  if (PROJECTOR)
  {
    nav_msgs::Path projected_path;
    PROJECTOR->projectPathOntoSurface(*msg, projected_path);
    ROS_INFO_STREAM("Projected path has " << projected_path.poses.size() << " points");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stl_surface_projection_node");
  ros::NodeHandle nh;
  nh.param<std::string>("/stl_surface_projection_node/stl_file_path", STL_FILE_PATH, "package://your_package/meshes/robot_model.stl");

  PROJECTOR = new STLSurfaceProjector(nh);
  PROJECTOR->loadSTLModel(STL_FILE_PATH);

  ros::Subscriber path_sub = nh.subscribe("/trajopt/init_traj", 1, pathCallback);

  ros::spin();
  return 0;
}