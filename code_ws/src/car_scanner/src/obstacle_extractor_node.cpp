#include "obstacle_detector/obstacle_extractor.h"

using namespace obstacle_detector;

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheel_extractor", ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");
  ros::Rate r(1);
  try {
    ROS_INFO("[Obstacle Extractor]: Initializing wheel_detection node");
    // ObstacleExtractor wheel_extractor_1(1, nh, nh_local, argv[1], argv[2], argv[3]);
    ObstacleExtractor wheel_extractor_2(2, nh, nh_local, argv[1], argv[2], argv[3]);
    while(ros::ok())
    {
      //od.checkScan();
      // wheel_extractor_1.wheel_scan();
      wheel_extractor_2.wheel_scan(); 
      ros::spinOnce();
      r.sleep();
    }
  }
  catch (const char* s) {
    ROS_FATAL_STREAM("[Obstacle Extractor]: "  << s);
  }
  catch (...) {
    ROS_FATAL_STREAM("[Obstacle Extractor]: Unexpected error");
  }

  return 0;
}
