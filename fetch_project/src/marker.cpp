#include <rviz_visual_tools/rviz_visual_tools.h>

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link","/rviz_visual_markers"));

// Create pose
Eigen::Affine3d pose;
pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

// Publish arrow vector of pose
ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
visual_tools_->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);

// Don't forget to trigger the publisher!
visual_tools_->trigger();
