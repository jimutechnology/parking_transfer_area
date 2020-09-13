#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <car_scanner/Wheel.h>
#include <car_scanner/WheelArray.h>
#include <car_scanner/WheelCmd.h>
#include <car_scanner/ErrNo.h>
#include <car_scanner/InfoOut.h>

#include "utilities/point.h"
#include "utilities/point_set.h"


namespace obstacle_detector
{

class ObstacleExtractor
{
public:
  ObstacleExtractor(int laser_id, ros::NodeHandle& nh, ros::NodeHandle& nh_local, char *first_input, char *second_input, char *image_path);
  ~ObstacleExtractor();
  void checkScan();
  void wheel_scan();

private:
  bool LoadParams();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr scan_msg);
  void initialize() { 
    LoadParams(); 
    // classifier.load(p_model_path_);
  }
  void groupPoints(double p_max_group_distance_);
    
  void detectWheels(PointSet& tmp1, PointSet& tmp2);
  void publishTires();

  // the living SVM model path
  // Classifier classifier;

  // Wheels wheels;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  // publishers and subscribers
  ros::Subscriber scan_sub_;
  ros::Subscriber wheel_cmd_sub_;
  ros::Subscriber pcl_sub_;
  ros::Publisher wheels_pub_;
  ros::Publisher warning_pub;
  ros::Publisher error_pub;

  ros::ServiceServer params_srv_;
  ros::Time scan_stamp;
  std::string base_frame_id_;
  std::string wheel_array_id_;
  std::string p_frame_id_;
  tf::TransformListener tf_listener_;

  // messages
  car_scanner::InfoOut error_out;
  car_scanner::InfoOut warning_out;
  car_scanner::ErrNo err_no;

  std::list<Point> input_points_;
  std::vector<Point> mix_points_;
  std::list<Point> pre_input_points;
  std::list<Point> tire_points_l;
  std::list<Point> tire_points_r;
  std::list<PointSet> point_sets;

  // Parameters
  string p_model_path;
  string p_model_project;
  string p_project_path;
  double p_distance_proportion_;
  double p_max_orientation;
  double p_max_wheels_distance;
  double p_min_wheels_distance;
  double p_max_group_dis;
  double p_scan_timeout;
  int    p_size_mode;
  double p_range_min;
  double p_range_max;
  int    p_noise_threshold;
  double p_noise_dis;
  bool   p_b_lidar_upsidedown;


  double distance_left;
  double distance_right;
  double distance_front;
  double min_angle;
  double max_angle;
  int laser_id;
  unsigned publish_count;


  bool running;
  bool Convert_Image;
  bool good_position;
  char *ros_id;

  char *first_input_;
  char *second_input_;
  char *image_path_;

  int result1;
  int result2;
};

} // namespace obstacle_detector
