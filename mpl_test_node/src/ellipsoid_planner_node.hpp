#pragma once

#include <experimental/filesystem>

#include "ros/ros.h"
#include "std_msgs/Float32.h"

namespace fs = std::experimental::filesystem;

class EllipsoidWrapper {
public:
  EllipsoidWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  EllipsoidWrapper()
      : EllipsoidWrapper(ros::NodeHandle(), ros::NodeHandle("~")) {}

  virtual ~EllipsoidWrapper();

private:
  void performPlanningCallback(const std_msgs::Float32ConstPtr &max_speed);

  bool loadParameters();
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber perform_planning_sub_;
  ros::Publisher completed_planning_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher ellipsoid_pub_;
  ros::Publisher start_goal_pub_;
  ros::Publisher traj_pub_;

  std::string data_dir_;

  std::vector<std::string> getDirectories(const std::string &s);
};
