#include <planning_ros_utils/trajectory_writer.h>

namespace trajectory_writer {

void saveTrajectorytoCSV(const std::string& csv_filename,
                         const planning_ros_msgs::Trajectory& msg, double dt) {
  printf("Saving trajectory to CSV. \n");
  StreamWithFilename reference_trajectory_file;
  reference_trajectory_file.filename = csv_filename;
  printf("Trajectory filename: %s\n",
         reference_trajectory_file.filename.c_str());

  reference_trajectory_file.filestream.open(
      reference_trajectory_file.filename,
      std::ios_base::trunc | std::ios_base::in);
  // write header
  // clang-format off
  reference_trajectory_file.filestream << "time_from_start" << ","
                                       << "pos_x" << ","
                                       << "pos_y" << ","
                                       << "pos_z" << ","
                                       << "vel_x" << ","
                                       << "vel_y" << ","
                                       << "vel_z" << ","
                                       << "acc_x" << ","
                                       << "acc_y" << ","
                                       << "acc_z" << ","
                                       << "jerk_x" << ","
                                       << "jerk_y" << ","
                                       << "jerk_z" << ","
                                       << "snap_x" << ","
                                       << "snap_y" << ","
                                       << "snap_z" << ","
                                       << "q_w" << ","
                                       << "q_x" << ","
                                       << "q_y" << ","
                                       << "q_z" << ","
                                       << "omega_x" << ","
                                       << "omega_y" << ","
                                       << "omega_z" << ","
                                       << "angular_acc_x" << ","
                                       << "angular_acc_y" << ","
                                       << "angular_acc_z" << ","
                                       << "angular_jerk_x" << ","
                                       << "angular_jerk_y" << ","
                                       << "angular_jerk_z" << ","
                                       << "angular_snap_x" << ","
                                       << "angular_snap_y" << ","
                                       << "angular_snap_z" << ","
                                       << "heading" << ","
                                       << "heading_rate" << ","
                                       << "heading_acc" << "\n";

  // clang-format on
  reference_trajectory_file.filestream.close();
  reference_trajectory_file.filestream.open(
      reference_trajectory_file.filename,
      std::ios_base::app | std::ios_base::in);

  const auto traj = toTrajectory3D(msg);
  int N = std::ceil(traj.getTotalTime() / dt);
  const auto ws = traj.sample(N);

  const auto t0 = ros::Time::now();
  for (unsigned int i = 0; i < ws.size(); i++) {
    // save current odometry & time to disk
    // clang-format off
      reference_trajectory_file.filestream
          << std::fixed
          << std::setprecision(8) << (ros::Duration(ws[i].t)).toSec() << ","
          << std::setprecision(8) << ws[i].pos(0) << ","
          << std::setprecision(8) << ws[i].pos(1) << ","
          << std::setprecision(8) << ws[i].pos(2) << ","
          << std::setprecision(8) << ws[i].vel(0) << ","
          << std::setprecision(8) << ws[i].vel(1) << ","
          << std::setprecision(8) << ws[i].vel(2) << ","
          << std::setprecision(8) << ws[i].acc(0) << ","
          << std::setprecision(8) << ws[i].acc(1) << ","
          << std::setprecision(8) << ws[i].acc(2) << ","
          << std::setprecision(8) << ws[i].jrk(0) << ","
          << std::setprecision(8) << ws[i].jrk(1) << ","
          << std::setprecision(8) << ws[i].jrk(2) << ","
          << std::setprecision(8) << 0. << ","
          << std::setprecision(8) << 0. << ","
          << std::setprecision(8) << 0. << ","
          << std::setprecision(8) << 1.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << 0.0 << ","
          << std::setprecision(8) << ws[i].yaw << ","
          << std::setprecision(8) << ws[i].yaw_dot << ","
          << std::setprecision(8) << 0.0 << "\n";
    // clang-format on
  }
  reference_trajectory_file.filestream.close();
  printf("Saved trajectory to file.\n");
}

}  // namespace trajectory_writer
