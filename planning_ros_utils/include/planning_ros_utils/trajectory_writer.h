#pragma once

#include <mpl_basis/trajectory.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <fstream>
#include <iomanip>
//#include <quadrotor_common/trajectory.h>


namespace trajectory_writer {

  void saveTrajectorytoCSV(
      const std::string& csv_filename,
			const planning_ros_msgs::Trajectory& msg, double dt);

  struct StreamWithFilename {
        std::ofstream filestream;
        std::string filename;
  };

  //quadrotor_common::Trajectory QuadTrajectoryExtractor(const planning_ros_msgs::Trajectory& msg, double dt);
} // namespace trajectory_writer
