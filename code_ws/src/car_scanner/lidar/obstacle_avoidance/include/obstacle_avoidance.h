#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <robot_msgs/CommandCtrl.h>
#include <robot_msgs/RobotState.h>
#include <robot_msgs/Link.h>
#include <robot_msgs/LinkStatus.h>
#include <robot_msgs/ErrNo.h>
#include <robot_msgs/InfoOut.h>
#include <robot_msgs/OdometryData.h>
#include <robot_msgs/TurnCmd.h>
#include <robot_msgs/ObstacleAvoidingCmd.h>
#include <robot_msgs/CommandReply.h>
//#include <robot_msgs/FormationData.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include "../../utilities/point.h"
#include "../../utilities/point_set.h"
#include "../../utilities/lidar_data_processor.h"
//#include "region.h"
#include "sensor.h"
#include <mutex>

using namespace geometry_msgs;
using namespace std;

class ObstacleAvoidance
{
public:
	ObstacleAvoidance(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
	~ObstacleAvoidance();
	void run();
	void checkScan();

private:
	void load_parameters();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr);
    void referenceStateCallback(const robot_msgs::RobotState::ConstPtr);
    void linkDataCallback(const robot_msgs::LinkStatus::ConstPtr);
    void odometryDataCallback(const robot_msgs::OdometryData::ConstPtr);
    void turnCmdDataCallback(const robot_msgs::TurnCmd::ConstPtr);
    void obstacleAvoidingCmdCallback(const robot_msgs::ObstacleAvoidingCmd::ConstPtr);
    
    
    // parameters
    double p_robot_width;
    double p_robot_length;
    double p_distance_proportion;
    
    double p_front_distance;
    double p_back_distance;
    double p_left_distance;
    double p_right_distance;

    double p_robot_front_distance;
    double p_robot_back_distance;
    double p_robot_left_distance;
    double p_robot_right_distance;

    double p_car_front_distance;
    double p_car_back_distance;
    double p_car_left_distance;
    double p_car_right_distance;

    double p_inflation_layer;
    double p_security_distance_x;
    double p_security_distance_y;
    double p_security_distance;
    double p_buffer_distance;
    double p_noise_filter_num;
    double p_noise_filter_dis;
    double p_range_min;
    double p_range_max;
    bool   p_debug;
    bool   p_close_error;
    vector<double> p_transformation_1;
    vector<double> p_transformation_2;

    double p_scan_timeout;
    double p_obstacle_error;
    double p_obstacle_warning;

    //lock
    std::mutex security_area_mutex;

    // data
    vector<vector<utility::Point>> input_points;
    vector<bool> data_valid;
    std::vector<PointSet> point_sets;
    std::vector<utility::Point> tmp_col_points1;
    std::vector<utility::Point> tmp_col_points2;
    Region security_area;
    string frame_id;
    utility::Point p1;
    utility::Point p2;
    utility::Point p3;
    
	// sensors
	Sensor laser_1;
	Sensor laser_2;

	// ros messages 
	robot_msgs::CommandCtrl command_ctrl;
	robot_msgs::Link link_data;
	robot_msgs::InfoOut error_out;
	robot_msgs::InfoOut warning_out;
    robot_msgs::ErrNo err_no;
    robot_msgs::LinkStatus link_stat;
    robot_msgs::OdometryData odometry_data;
    robot_msgs::ObstacleAvoidingCmd obstacle_avoiding_cmd;
    robot_msgs::CommandReply cmd_reply_data;

	// robot_msgs::FormationData formation_data;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker col_points;
	sensor_msgs::LaserScan f_scan;

	ros::Time scan_stamp_1;
	ros::Time scan_stamp_2;
	ros::Time obstacle_stamp;

	// node handle
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	// publishers and subscribers
	ros::Subscriber scan_sub;
	ros::Subscriber localization_sub;
	ros::Subscriber referenceState_sub;
	ros::Subscriber linkData_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber turnCmd_sub;
    ros::Subscriber obstacleAvoiding_sub;
	ros::Publisher  command_pub;
	ros::Publisher  marker_pub;
	ros::Publisher  error_pub;
	ros::Publisher  warning_pub;
	ros::Publisher  f_scan_pub;
    ros::Publisher  m_scan_pub;
	ros::Publisher  collision_pub;
    ros::Publisher  command_reply_pub;
	// ros::Publisher  formation_pub;

	// result
	bool pre_result;
	bool curr_result;
    bool obstacle_error_pub;
	bool obstacle_warn_pub;

	// robot state
	double ref_x;
	double ref_y;
	double ref_yaw;
	double ref_vx;
	double ref_vy;
	double ref_w;
	double ref_dec;

    double odometry_data_x;
    double odometry_data_y;
    double odometry_data_yaw;

    double turn_radian;
    string ros_id;
    string identity;
    
    geometry_msgs::Pose2D master_pose;
    geometry_msgs::Pose2D slave_pose;

    // the relative direction between robots, 1 means same direction, -1 means inverse
    int pair_direction;
    // rule
	map<string, vector<double>> rule;  // Rule for obstacle avoidance under different modes
	string robot_status;
	string pre_robot_status;

    // tool
    LidarDataProcessor lp;
    int reference_count;
    unsigned marker_count;
    unsigned pre_marker;
	

	
};