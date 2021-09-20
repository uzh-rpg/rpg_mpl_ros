#include "planning_ros_utils/trajectory_writer.h"
#include <decomp_ros_utils/data_ros_utils.h>
#include <mpl_external_planner/ellipsoid_planner/ellipsoid_planner.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>
#include "agile_autonomy_utils/generate_reference.h"
#include "ellipsoid_planner_node.hpp"
#include "quadrotor_common/trajectory.h"
#include <experimental/filesystem>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// open3d_conversions
#include "example-utils.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include "tinyply.h"

#include "bag_reader.hpp"

void parse_pointcloud(const std::string pointcloud_filename,
                      open3d::geometry::PointCloud *o3d_pointcloud,
                      std::vector<Eigen::Vector3d> *points_vec) {
  std::ifstream csvFile;
  csvFile.open(pointcloud_filename.c_str());

  if (!csvFile.is_open()) {
    std::cout << "Path Wrong!!!!" << std::endl;
    std::cout << pointcloud_filename << std::endl;
    exit(EXIT_FAILURE);
  }

  open3d::geometry::PointCloud pointcloud;

  ////////////////////////////////////
  //  std::cout <<
  //  "..............................................................."
  //               ".........\n";
  std::cout << "Now Reading: " << pointcloud_filename << std::endl;

  std::unique_ptr<std::istream> file_stream;
  std::vector<uint8_t> byte_buffer;
  //  Eigen::MatrixXd points_eigen;
  //  std::vector<Eigen::Vector3d> points_vec;

  try {
    // For most files < 1gb, pre-loading the entire file upfront and wrapping
    //    it
    // into a stream is a net win for parsing speed, about 40% faster.
    bool preload_into_memory = true;
    if (preload_into_memory) {
      byte_buffer = read_file_binary(pointcloud_filename);
      file_stream.reset(
          new memory_stream((char *)byte_buffer.data(), byte_buffer.size()));
    } else {
      file_stream.reset(
          new std::ifstream(pointcloud_filename, std::ios::binary));
    }

    if (!file_stream || file_stream->fail())
      throw std::runtime_error("file_stream failed to open " +
                               pointcloud_filename);

    file_stream->seekg(0, std::ios::end);
    const float size_mb = file_stream->tellg() * float(1e-6);
    file_stream->seekg(0, std::ios::beg);

    PlyFile file;
    file.parse_header(*file_stream);

    // Because most people have their own mesh types, tinyply treats parsed
    //    data
    // as structured/typed byte buffers. See examples below on how to marry
    //        your
    // own application-specific data structures with this one.
    std::shared_ptr<PlyData> vertices, normals, colors, texcoords, faces,
        tripstrip;

    // The header information can be used to programmatically extract
    //    properties
    // on elements known to exist in the header prior to reading the data.
    // For brevity of this sample, properties like vertex position are
    // hard-coded:
    try {
      vertices =
          file.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      normals =
          file.request_properties_from_element("vertex", {"nx", "ny", "nz"});
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors = file.request_properties_from_element(
          "vertex", {"red", "green", "blue", "alpha"});
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors =
          file.request_properties_from_element("vertex", {"r", "g", "b", "a"});
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      texcoords = file.request_properties_from_element("vertex", {"u", "v"});
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    // Providing a list size hint (the last argument) is a 2x performance
    // improvement. If you have arbitrary ply files, it is best to leave this
    //    0.
    try {
      faces =
          file.request_properties_from_element("face", {"vertex_indices"}, 3);
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    // Tristrips must always be read with a 0 list size hint (unless you know
    // exactly how many elements are specifically in the file, which is
    // unlikely);
    try {
      tripstrip = file.request_properties_from_element("tristrips",
                                                       {"vertex_indices"}, 0);
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    manual_timer read_timer;

    read_timer.start();
    file.read(*file_stream);
    read_timer.stop();

    const float parsing_time = read_timer.get() / 1000.f;
    std::cout << "\tparsing " << size_mb << "mb in " << parsing_time
              << " seconds [" << (size_mb / parsing_time) << " MBps]"
              << std::endl;

    const size_t numVerticesBytes = vertices->buffer.size_bytes();
    std::vector<float3> verts(vertices->count);
    std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

    int idx = 0;
    for (auto point_tinyply : verts) {
      //      if (idx == 0) {
      //        points_eigen =
      //        Eigen::Vector3d(static_cast<double>(point_tinyply.x),
      // static_cast<double>(point_tinyply.y),
      // static_cast<double>(point_tinyply.z));
      //      } else {
      //        points_eigen.conservativeResize(points_eigen.rows(),
      //                                        points_eigen.cols() + 1);
      //        points_eigen.col(points_eigen.cols() - 1) =
      //            Eigen::Vector3d(static_cast<double>(point_tinyply.x),
      //                            static_cast<double>(point_tinyply.y),
      //                            static_cast<double>(point_tinyply.z));
      //      }
      points_vec->push_back(
          Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                          static_cast<double>(point_tinyply.y),
                          static_cast<double>(point_tinyply.z)));
      idx += 1;

      //      if (idx > 100) {
      //        break;
      //      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
  }

  //  kd_tree_.SetMatrixData(points_eigen);

  *o3d_pointcloud = open3d::geometry::PointCloud(*points_vec);

  std::cout << "Completed pointcloud parsing!" << std::endl;
}

EllipsoidWrapper::EllipsoidWrapper(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  if (!loadParameters()) {
    ROS_ERROR("[%s] Failed to load all parameters",
              ros::this_node::getName().c_str());
    ros::shutdown();
  }
  perform_planning_sub_ = nh_.subscribe(
      "perform_planning", 1, &EllipsoidWrapper::performPlanningCallback, this);

  completed_planning_pub_ =
      pnh_.advertise<std_msgs::Bool>("completed_planning", 1);
  cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  ellipsoid_pub_ =
      pnh_.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoids", 1);
  start_goal_pub_ =
      pnh_.advertise<sensor_msgs::PointCloud>("start_and_goal", 1);
  traj_pub_ = pnh_.advertise<planning_ros_msgs::Trajectory>("trajectory", 1);
}

EllipsoidWrapper::~EllipsoidWrapper() {}

void EllipsoidWrapper::performPlanningCallback(
    const std_msgs::Float32ConstPtr &max_speed) {
  printf("Performing planning for directory %s\n", data_dir_.c_str());
  ros::Duration(2.0).sleep();
  std::vector<std::string> directories = getDirectories(data_dir_);

  if (directories.size() > 1) {
    ROS_WARN(
        "Currently only supports single rollout folders! Delete pre-existing "
        "rollout folders!");
    // we only use the last directory
    ROS_INFO("num of directories: %lu", directories.size());
    directories.erase(directories.begin(), directories.end() - 1);
    ROS_INFO("num of directories: %lu", directories.size());
  }
  quadrotor_common::Trajectory reference_trajectory;

  for (auto const &directory : directories) {
    int idx = 0;
    //    first_state_in_traj = true;
    ros::Duration start_time = ros::Duration(0.0);
    // iterate over generated csv file, generate label for each row
    std::string odometry_filename = directory + "/odometry.csv";

    std::ifstream odometry_file;
    odometry_file.open(odometry_filename.c_str());

    if (!odometry_file.is_open()) {
      std::cout << "Did not find the following file!!" << std::endl;
      std::cout << odometry_filename << std::endl;
      std::cout << "Will skip folder" << std::endl;
      continue;
    }

    std::string reference_trajectory_filename =
        directory + "/reference_trajectory.csv";
    std::string pc_filename = directory + "/pointcloud-unity.ply";

    loadReferenceTrajectory(&reference_trajectory,
                            reference_trajectory_filename, true);

    ros::Time t0 = ros::Time::now();

    open3d::geometry::PointCloud o3d_pc;
    std::vector<Eigen::Vector3d> points_eigen;
    parse_pointcloud(pc_filename, &o3d_pc, &points_eigen);
    ROS_INFO("Loaded pointcloud contains %lu points.", o3d_pc.points_.size());

    sensor_msgs::PointCloud2 ros_pc2;
    open3d_conversions::open3dToRos(o3d_pc, ros_pc2, "world");
    sensor_msgs::PointCloud map;
    map.header.frame_id = "world";
    map.header.stamp = ros::Time::now();
    bool success = sensor_msgs::convertPointCloud2ToPointCloud(ros_pc2, map);
    if (!success) {
      printf("PointCloud loading failed");
    }

    ros_pc2.header.stamp = ros::Time::now();
    cloud_pub_.publish(ros_pc2);

    ROS_INFO("Pointcloud2 contains %d points.", ros_pc2.height * ros_pc2.width);
    ROS_INFO("Pointcloud contains %lu points.", map.points.size());

    double robot_radius;
    robot_radius = 0.3;
    Vec3f origin, dim;
    Eigen::Vector3d padding = 10.0 * Eigen::Vector3d::Ones();
    origin(0) =
        reference_trajectory.points.front().position.x() - 0.5 * padding.x();
    origin(1) =
        reference_trajectory.points.front().position.y() - 0.5 * padding.y();
    origin(2) =
        reference_trajectory.points.front().position.z() - 0.5 * padding.z();
    dim(0) = reference_trajectory.points.back().position.x() -
             reference_trajectory.points.front().position.x() + padding.x();
    dim(1) = reference_trajectory.points.back().position.y() -
             reference_trajectory.points.front().position.y() + padding.y();
    dim(2) = reference_trajectory.points.back().position.z() -
             reference_trajectory.points.front().position.z() + padding.z();

    ROS_INFO("Takes %f sec to set up map!", (ros::Time::now() - t0).toSec());
    t0 = ros::Time::now();

    // Initialize planner
    double dt, v_max, a_max, w, epsilon;
    double u_max_z, u_max;
    int max_num, num;
    bool use_3d;
    dt = 0.2;
    epsilon = 2.0;
    v_max = max_speed->data; // 12.0;
    a_max = 10.0;
    u_max = 60.0;
    u_max_z = 1.0;
    w = 10000.0;
    num = 2;
    max_num = 500000;
    use_3d = false;

    ROS_INFO("Initiating planner...");
    std::unique_ptr<MPL::EllipsoidPlanner> planner_;
    planner_.reset(new MPL::EllipsoidPlanner(true));
    ROS_INFO("robot_radius: %.2f", robot_radius);
    cloud_to_vec(map);
    std::cout << "Origin: " << origin.transpose() << std::endl;
    std::cout << "Dimension: " << dim.transpose() << std::endl;
    //  planner_->setMap(points_eigen, robot_radius, origin,
    //                   dim);          // Set collision checking function
    planner_->setMap(cloud_to_vec(map), robot_radius, origin,
                     dim); // Set collision checking function
    ROS_INFO("epsilon: %.2f", epsilon);
    planner_->setEpsilon(epsilon); // Set greedy param (default equal to 1)
    planner_->setVmax(v_max);      // Set max velocity
    planner_->setAmax(a_max);      // Set max acceleration
    planner_->setDt(dt);           // Set dt for each primitive
    planner_->setW(w);             // Set time weight for each primitive
    planner_->setMaxNum(
        max_num); // Set maximum allowed expansion, -1 means no limitation
    planner_->setTol(7.0, 100.0,
                     100.0); // Tolerance for goal region as pos, vel, acc

    // Set start and goal
    ROS_INFO("Setting start and goal...");
    double start_x, start_y, start_z;
    start_x = reference_trajectory.points.front().position.x();
    start_y = reference_trajectory.points.front().position.y();
    start_z = reference_trajectory.points.front().position.z();
    double start_vx, start_vy, start_vz;
    start_vx = reference_trajectory.points.front().velocity.x();
    start_vy = reference_trajectory.points.front().velocity.y();
    start_vz = reference_trajectory.points.front().velocity.z();
    double goal_x, goal_y, goal_z;
    goal_x = reference_trajectory.points.back().position.x();
    goal_y = reference_trajectory.points.back().position.y();
    goal_z = reference_trajectory.points.back().position.z();

    // magic parameters are copied from original launch file
    bool use_acc, use_jrk;
    use_acc = true;
    use_jrk = false;

    Waypoint3D start;
    start.pos = Vec3f(start_x, start_y, start_z);
    start.vel = Vec3f(start_vx, start_vy, start_vz);
    start.acc = Vec3f(0, 0, 0);
    start.jrk = Vec3f(0, 0, 0);
    start.use_pos = true;
    start.use_vel = true;
    start.use_acc = use_acc;
    start.use_jrk = use_jrk;
    start.use_yaw = false;

    Waypoint3D goal(start.control);
    goal.pos = Vec3f(goal_x, goal_y, goal_z);
    goal.vel = Vec3f(10, 0, 0);
    goal.acc = Vec3f(0, 0, 0);
    goal.jrk = Vec3f(0, 0, 0);

    // Publish location of start and goal
    sensor_msgs::PointCloud sg_cloud;
    sg_cloud.header.frame_id = "world";
    geometry_msgs::Point32 pt1, pt2;
    pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
    pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
    sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
    start_goal_pub_.publish(sg_cloud);

    // Read prior traj
    ROS_INFO("Reading prior trajectory...");
    std::string traj_file_name, traj_topic_name;

    // Set input control
    vec_E<VecDf> U;
    const decimal_t du = u_max / num;
    if (use_3d) {
      decimal_t du_z = u_max_z / num;
      for (decimal_t dx = -u_max; dx <= u_max; dx += du)
        for (decimal_t dy = -u_max; dy <= u_max; dy += du)
          for (decimal_t dz = -u_max_z; dz <= u_max_z;
               dz += du_z) // here we reduce the z control
            U.push_back(Vec3f(dx, dy, dz));
    } else {
      for (decimal_t dx = -u_max; dx <= u_max; dx += du)
        for (decimal_t dy = -u_max; dy <= u_max; dy += du)
          U.push_back(Vec3f(dx, dy, 0));
    }
    planner_->setU(U); // Set discretization with 1 and efforts
    // planner_->setMode(num, use_3d, start); // Set discretization with 1 and
    // efforts
    // Planning thread!

    t0 = ros::Time::now();
    ROS_INFO("Initiate planning...");
    bool valid = planner_->plan(start, goal);

    // Publish expanded nodes
    //  sensor_msgs::PointCloud ps = vec_to_cloud(planner_->getCloseSet());
    //  ps.header.frame_id = "map";
    //  ps_pub.publish(ps);

    if (!valid) {
      ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
               (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());
      std_msgs::Bool false_msg;
      false_msg.data = false;
      completed_planning_pub_.publish(false_msg);
    } else {
      ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
               (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());

      // Publish trajectory
      auto traj = planner_->getTraj();
      planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
      traj_msg.header.frame_id = "world";
      traj_pub_.publish(traj_msg);
      std::string global_traj_fname = directory + "/ellipsoid_trajectory.csv";
      trajectory_writer::saveTrajectorytoCSV(global_traj_fname, traj_msg, 0.02);

      printf("================== Traj -- total J(VEL): %f, J(ACC): %F, J(JRK): "
             "%f, "
             "total time: %f\n",
             traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::SNP),
             traj.getTotalTime());

      vec_E<Ellipsoid3D> Es = sample_ellipsoids(
          traj, Vec3f(robot_radius, robot_radius, 0.5 * robot_radius), 50);
      decomp_ros_msgs::EllipsoidArray es_msg =
          DecompROS::ellipsoid_array_to_ros(Es);
      es_msg.header.frame_id = "world";
      ellipsoid_pub_.publish(es_msg);

      max_attitude(traj, 1000);

      std_msgs::Bool true_msg;
      true_msg.data = true;
      completed_planning_pub_.publish(true_msg);
    }
  }
}

std::vector<std::string>
EllipsoidWrapper::getDirectories(const std::string &s) {
  std::vector<std::string> r;
  for (auto p = fs::recursive_directory_iterator(s);
       p != fs::recursive_directory_iterator(); ++p) {
    if (p->status().type() == fs::file_type::directory && p.depth() == 0) {
      fs::path p1 = p->path();
      p1 += "/trajectories/trajectories_bf_00000000_muted.csv";
      if (!fs::exists(p1)) {
        r.push_back(p->path().string());
        ROS_INFO("Added Folder: %s", p->path().string().c_str());
      }
    }
  }
  return r;
}

bool EllipsoidWrapper::loadParameters() {
  if (!pnh_.getParam("data_dir", data_dir_))
    return false;

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ellipsoid_planner");
  EllipsoidWrapper ellipsoid_wrapper;

  ros::spin();

  return 0;
}
