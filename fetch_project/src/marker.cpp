// // ROS
// #include <ros/ros.h>
//
// // For visualizing things in rviz
// #include <rviz_visual_tools/rviz_visual_tools.h>
//
// // C++
// #include <string>
// #include <vector>
//
// namespace rvt = rviz_visual_tools;
//
// namespace rviz_visual_tools
// {
// class RvizVisualToolsDemo
// {
// private:
//   // A shared node handle
//   ros::NodeHandle nh_;
//
//   // For visualizing things in rviz
//   rvt::RvizVisualToolsPtr visual_tools_;
//
//   std::string name_;
//
// public:
//   /**
//    * \brief Constructor
//    */
//   RvizVisualToolsDemo() : name_("rviz_demo")
//   {
//     visual_tools_.reset(new rvt::RvizVisualTools("gripper_link", "/cone_marker"));
//     visual_tools_->loadMarkerPub();  // create publisher before waiting
//
//     ROS_INFO("Sleeping 5 seconds before running demo");
//     ros::Duration(5.0).sleep();
//
//     // Clear messages
//     visual_tools_->deleteAllMarkers();
//     visual_tools_->enableBatchPublishing();
//
//
//     // Create pose
//     Eigen::Isometry3d pose_cone = Eigen::Isometry3d::Identity();
//     pose_cone = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
//     pose_cone.translation() = Eigen::Vector3d( 0.0, 0.0, 0.0 ); // translate x,y,z
//
//     //Publish Cone marker of pose
//     ROS_INFO_STREAM_NAMED("test", "Publishing Cone");
//     visual_tools_->publishCone(pose_cone,M_PI, rvt::RAND, 0.05);
//     visual_tools_->trigger();
//   }
//
//
// };  // end class
// }  // namespace rviz_visual_tools
//
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "visual_tools_demo");
//   ROS_INFO_STREAM("Visual Tools Demo");
//
//
//
//   // Allow the action server to recieve and send ros messages
//   ros::AsyncSpinner spinner(1);
//   spinner.start();
//   while (ros::ok())
//   {
//   rviz_visual_tools::RvizVisualToolsDemo demo;
// }
//   ROS_INFO_STREAM("Shutting down.");
//   return 0;
// }
// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// C++
#include <string>
#include <vector>

namespace rvt = rviz_visual_tools;

int main ( int argc, char** argv)
{
  ros::init(argc, argv, "cone_marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  rvt::RvizVisualToolsPtr visual_tools_;
  std::string name_;

  visual_tools_.reset(new rvt::RvizVisualTools("gripper_link", "/cone_marker"));
  visual_tools_->loadMarkerPub();  // create publisher before waiting
  ROS_INFO("Sleeping 5 seconds before running demo");
  ros::Duration(5.0).sleep();
  // Clear messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  // Create pose
  Eigen::Isometry3d pose_cone = Eigen::Isometry3d::Identity();
  pose_cone = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
  pose_cone.translation() = Eigen::Vector3d( 0.0, 0.0, 0.0 ); // translate x,y,z

  //Publish Cone marker of pose
  ROS_INFO_STREAM_NAMED("test", "Publishing Cone");
  // while(ros::ok())
  while (ros::ok())
  {
  ros::Duration(.30).sleep();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();
  visual_tools_->publishCone(pose_cone,M_PI/3, rvt::TRANSLUCENT, 0.3);
  visual_tools_->trigger();
  }
}
