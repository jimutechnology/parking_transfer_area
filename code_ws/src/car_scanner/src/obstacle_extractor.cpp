#include "../include/obstacle_detector/obstacle_extractor.h"
//#include "../include/obstacle_detector/utilities/laser_converter.h"
#include "../include/obstacle_detector/utilities/math_utilities.h"
#include "../include/obstacle_detector/utilities/classifier.h"
#include "../include/obstacle_detector/utilities/image_generator.h"
#include <stdlib.h>
#include "ros/ros.h"
#include <math.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp> 
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream> 
#include <string>
using namespace std;
using namespace car_scanner;
using namespace obstacle_detector;

ObstacleExtractor::ObstacleExtractor(int laser_id, ros::NodeHandle& nh, ros::NodeHandle& nh_local, 
char *first_input, char *second_input, char *image_path) : nh_(nh), nh_local_(nh_local)
 {
  if (first_input != NULL && second_input != NULL)
    {
      first_input_ = first_input;
      second_input_ = second_input;
    } 
  if (image_path != NULL)
    {
      image_path_ = image_path;
    }   
  this->laser_id = laser_id;
  running = false;
  good_position = false;
  distance_left = 1;
  distance_right = 1;
  distance_front = 2;
  Convert_Image = false;
  initialize();
  wheels_pub_ = nh_.advertise<car_scanner::WheelArray>("wheel_info", 10);
  warning_pub = nh_.advertise<car_scanner::InfoOut>("warning_info", 10);
  error_pub = nh_.advertise<car_scanner::InfoOut>("error_info", 10);
  scan_sub_ = nh_.subscribe("scan", 10, &ObstacleExtractor::scanCallback, this);
}

ObstacleExtractor::~ObstacleExtractor() {
  // nh_local_.deleteParam("active");
  // nh_local_.deleteParam("use_scan");
  // nh_local_.deleteParam("distance_proportion");
  // nh_local_.deleteParam("frame_id");
}

bool ObstacleExtractor::LoadParams() {
  
  ros::param::param<std::string>("/obstacle_extractor/SVM_model_path", p_model_path_, "/home/jimu/code/parking_transfer_area/code_ws/src/car_scanner/conf/wheel_classifier.xml");
  ros::param::param("/obstacle_extractor/max_group_dis", p_max_group_dis, 0.1);
  ros::param::param("/obstacle_extractor/max_tolerant_orientaion", p_max_orientation, 0.3);
  ros::param::param("/obstacle_extractor/max_wheels_distance", p_max_wheels_distance, 1.8);
  ros::param::param("/obstacle_extractor/min_wheels_distance", p_min_wheels_distance), 1.25;
  ros::param::param("/obstacle_extractor/size_mode", p_size_mode, 0);
  ros::param::param("/scan_timeout", p_scan_timeout, 0.5);
  ros::param::param("/distance_proportion", p_distance_proportion_, 0.00628);
  ros::param::param("/obstacle_extractor/range_min", p_range_min, 0.05);
  ros::param::param("/obstacle_extractor/range_max", p_range_max, 4.0);
  ros::param::param("/obstacle_extractor/noise_threshold", p_noise_threshold, 10);
  ros::param::param("/obstacle_extractor/noise_dis", p_noise_dis, 0.2);
  ros::param::param<std::string>("/lidar_" + to_string(laser_id) + "/frame_id", p_frame_id_, "laser_1");
  wheel_array_id_ = p_frame_id_;

  ros::param::param("/obstacle_extractor/distance_left", distance_left, 2.5);
  ros::param::param("/obstacle_extractor/distance_right", distance_right, 2.5);
  ros::param::param("/obstacle_extractor/distance_front", distance_front, 3.0);
  ros::param::param("/obstacle_extractor/min_angle", min_angle, -1.57);
  ros::param::param("/obstacle_extractor/max_angle", max_angle, 1.57);

  return true;
}


void ObstacleExtractor::scanCallback(const sensor_msgs::LaserScan::ConstPtr scan_msg) {
  base_frame_id_ = scan_msg->header.frame_id;
  if (base_frame_id_ == p_frame_id_)
  {
    scan_stamp = scan_msg->header.stamp;
    double phi = scan_msg->angle_min;
    for (const float r : scan_msg->ranges) 
    {
      if (r >= p_range_min && r <= p_range_max)
        input_points_.push_back(Point::fromPoolarCoords(r, phi));
      phi += scan_msg->angle_increment;
    }
    // filter the noise
    typedef std::list<Point>::iterator PointIterator;
    typedef std::list<PointSet>::iterator PointSetIterator;
    groupPoints(p_noise_dis);
    input_points_.clear();
    for (PointSetIterator point_set = point_sets.begin(); point_set != point_sets.end(); ++point_set)
    {
      if (point_set->num_points > p_noise_threshold)
      {
          for (PointIterator point = point_set->group.begin(); point != point_set->group.end(); ++point)
          {
            input_points_.push_back(*point);
          }
      }
    }
    point_sets.clear();

    if (running == true)
    {
          if (input_points_.size() != 0)
          {  
          publishTires();
          input_points_.clear();
          running = false;
          }
          else
          {
            //cout << "the input_points_ is empty" << endl; 
          }
     }    
    input_points_.clear();
  }
}

// receive the wheel_scan topic command
void ObstacleExtractor::wheel_scan() {
  // wheel_array_id_ = wheel_cmd->header.frame_id;
  // laser_id = wheel_cmd->laser_id;
  // determine which lidar to be used to detect wheel
  // p_frame_id_ = "laser_" + to_string(laser_id);


  running = true;
  //cout << "call back running is "<< running << endl;
}




void ObstacleExtractor::detectWheels(PointSet& tmp1, PointSet& tmp2)
{
  typedef std::list<Point>::iterator PointIterator;
  typedef std::list<PointSet>::iterator PointSetIterator;

  // cluster points which are close to each other
 
  groupPoints(p_max_group_dis);
 
  PointSet tmp;
  // the SVM model path
  string model_path = p_model_path_;
  Classifier classifier(model_path);
  // select the two point sets which most likely are tires
  for (PointSetIterator point_set = point_sets.begin(); point_set != point_sets.end(); ++point_set)
  {
   point_set->get_feature_points();
   if (!Convert_Image)
   {
    // cout << "go to predict" << endl;
    classifier.predict(*point_set);
    // cout << "the result is " << point_set -> class_result << endl;
    // cout << "the confidence is " << point_set -> confidence << endl;
    // cout << "the top_point is " << point_set->top_point << endl;
    // cout << "the bottom_point is " << point_set->bottom_point << endl;
   }
   else
   {
    point_set->class_result = 1;
   }
   // core algorithm
   
   cout << "raw: " << endl;
   for (PointIterator i = point_set->group.begin(); i != point_set->group.end(); ++i)
       cout << i->x << ", " << i->y << "; ";
   cout << endl;
   cout << point_set->max_x << ", " << point_set->min_x << ", " << point_set->max_y << ", " << point_set->min_y << ", " << point_set->num_points << ", " << point_set->class_result << endl;
   
   if (point_set->max_x < distance_front && point_set->min_x > 0 && point_set->max_y < distance_left && 
    point_set->min_y > -distance_right && point_set->num_points > 10 && point_set->class_result == 1 &&
    point_set->min_y * point_set->max_y > 0)
   {
      // cout << "OK " << endl;
      // first filter the tmp1 from point_sets pool through x coordinate of short edge feature point
      // set the nearest 2 cluster as tmp1 and tmp2 while tmp2 < tmp1
      if (abs(point_set->short_fp.x) < abs(tmp1.short_fp.x))
      {
          tmp1.copy(*point_set);
          if (abs(tmp1.short_fp.x) < abs(tmp2.short_fp.x))
          {
            tmp.copy(tmp2);
            tmp2.copy(tmp1);
            tmp1.copy(tmp);
          }
      }
      // second compare the tmp1 with tmp2 to update tmp2
    }
    // cout << "finish a point_set" << endl;
  }
  // cout << "the tmp1 confidence is " << tmp1.confidence << endl;
  // cout << "the tmp2 confidence is " << tmp2.confidence << endl;
  // cout << "the tmp1 num_points is " << tmp1.num_points << endl;
  // cout << "the tmp2 num_points is " << tmp2.num_points << endl;
  // cout << "the tmp1 top_point is " << tmp1.top_point << endl;
  // cout << "the tmp1 bottom_point is " << tmp1.bottom_point << endl;
  // cout << "the tmp2 top_point is " << tmp2.top_point << endl;
  // cout << "the tmp2 bottom_point is " << tmp2.bottom_point << endl;

  if (Convert_Image)
  {
    ImageGenerator Ig;
    Ig.generate(tmp1, tmp2, image_path_, first_input_, second_input_);
  }

  point_sets.clear();
}


void ObstacleExtractor::publishTires()
{
  std::vector<PointSet> MF;
  std::vector<Point> first_points;
  std::vector<Point> second_points;
  std::vector<Point> third_points;
 
  PointSet tmp1;
  PointSet tmp2;
  car_scanner::WheelArray wheelarray;
  detectWheels(tmp1, tmp2);
  // there is no wheel in detection area
  if (tmp2.num_points == 0)
  {
    warning_out.error_code = err_no.LID_IDF_NO_WHEEL;
    warning_out.message = "No wheel found";
    warning_pub.publish(warning_out);
    wheelarray.ready = false;
  }
  // judge whether there are two complete wheels
  // If without two wheels, publish empty message
  if (tmp1.num_points < 10 || tmp2.num_points < 10 || !tmp1.class_result || !tmp2.class_result)
  {
    warning_out.error_code = err_no.LID_IDF_NO_OUTLINE;
    warning_out.message = "No complete wheels!";
    warning_pub.publish(warning_out);
    wheelarray.ready = false;
    cout << tmp1.num_points << " " << tmp2.num_points << " " << wheel_array_id_ << endl;
  }
  // there are two complete wheels
  else
  {
    double ori;
    wheelarray.ready = true;
    if (tmp1.max_y > tmp2.max_y)
    {
      ori = atan(-(tmp1.short_fp.x - tmp2.short_fp.x)/(tmp1.short_fp.y - tmp2.short_fp.y));
    }
    else
    {
      ori = atan(-(tmp2.short_fp.x - tmp1.short_fp.x)/(tmp2.short_fp.y - tmp1.short_fp.y));
    }
    if (abs(ori) > p_max_orientation)
    {
      warning_out.error_code = err_no.LID_IDF_MAX_ORI;
      warning_out.message = "Excessive orientation";
      warning_pub.publish(warning_out);
      wheelarray.ready = false;
    }
    tmp1.orientation = ori;
    tmp2.orientation = ori; 
    tmp1.get_rep_point();
    tmp2.get_rep_point();
    // calculate the wheel distance
    double wheel_distance = tmp1.middle_point.distance(tmp2.middle_point);
    if (wheel_distance > p_max_wheels_distance)
    {
      /*cout << "tmp1 middle point is " << tmp1.middle_point;
      cout << "tmp2 middle point is " << tmp2.middle_point;
      cout << "the wheel_distance is " << wheel_distance;
      */
      warning_out.error_code = err_no.LID_IDF_MAX_DIS;
      warning_out.message = "Wheel distance too large!";
      warning_pub.publish(warning_out);

      wheelarray.interval = wheel_distance;
      wheelarray.ready = false;
    }
    else if(wheel_distance < p_min_wheels_distance)
    {
      warning_out.error_code = err_no.LID_IDF_MIN_INTV;
      warning_out.message = "Wheel distance too small!";
      warning_pub.publish(warning_out);

      wheelarray.interval = wheel_distance;
      wheelarray.ready = false;
    }
    // double tmp1_length = (tmp1.top_point - tmp1.middle_point).length();
    // double tmp2_length = (tmp2.top_point - tmp2.middle_point).length();
    double tmp1_length = (tmp1.bottom_point - tmp1.middle_point).length(); // TODO
    double tmp2_length = (tmp2.bottom_point - tmp2.middle_point).length();
    //cout << tmp1_length << ", " << tmp2_length << ", " << tmp1.num_points << ", " << tmp2.num_points << endl;
    
    // if (tmp1_length < 0.3 || tmp2_length < 0.3 || abs(tmp1.num_points - tmp2.num_points) > 10 || abs(tmp1_length - tmp2_length) > 0.2) // TODO: 20 was 10
    if (tmp1_length < 0.3 || tmp2_length < 0.3 || abs(tmp1.num_points - tmp2.num_points) > 20 || abs(tmp1_length - tmp2_length) > 0.2)
      good_position = false;
    else
      good_position = true;
    // cout << good_position << (tmp1_length < 0.3) << (tmp2_length < 0.3) << (abs(tmp1.num_points - tmp2.num_points) > 10) << (abs(tmp1_length - tmp2_length) > 0.2) << endl;
    MF.push_back(tmp1);
    MF.push_back(tmp2);
    // both wheels on right side
    if (MF[0].min_y <= 0 && MF[1].min_y <= 0)
    {
      first_points.push_back(MF[0].top_point);
      second_points.push_back(MF[0].middle_point);
      third_points.push_back(MF[0].bottom_point);
      first_points.push_back(MF[1].top_point);
      second_points.push_back(MF[1].middle_point);
      third_points.push_back(MF[1].bottom_point);
    }

    // both wheels on each side
    if ((MF[0].max_y <= 0 && MF[1].max_y >= 0) || (MF[0].max_y >= 0 && MF [1].max_y <= 0))
    {
        if (good_position == false)
        {
          if (MF[0].max_y > 0)
          { 
            first_points.push_back(Point (MF[0].max_x, MF[0].min_y));
            second_points.push_back(Point (MF[0].min_x, MF[0].min_y));
            third_points.push_back(Point (MF[0].min_x, MF[0].max_y));
            first_points.push_back(Point (MF[1].max_x, MF[1].max_y));
            second_points.push_back(Point (MF[1].min_x, MF[1].max_y));
            third_points.push_back(Point (MF[1].min_x, MF[1].min_y));
          }
          else
          {
            first_points.push_back(Point (MF[0].max_x, MF[0].max_y));
            second_points.push_back(Point (MF[0].min_x, MF[0].max_y));
            third_points.push_back(Point (MF[0].min_x, MF[0].min_y));
            first_points.push_back(Point (MF[1].max_x, MF[1].min_y));
            second_points.push_back(Point (MF[1].min_x, MF[1].min_y));
            third_points.push_back(Point (MF[1].min_x, MF[1].max_y));
          }
        }
        else  
        {
          first_points.push_back(MF[0].top_point);
          second_points.push_back(MF[0].middle_point);
          third_points.push_back(MF[0].bottom_point);
          first_points.push_back(MF[1].top_point);
          second_points.push_back(MF[1].middle_point);
          third_points.push_back(MF[1].bottom_point);
        }
    }

    // both wheels on left side
    if (MF[0].max_y >= 0 && MF[1].max_y >= 0)
    {
      if (MF[0].max_y > MF[1].max_y)
      {
        third_points.push_back(Point (MF[0].min_x, MF[0].max_y));
        second_points.push_back(Point (MF[0].min_x, MF[0].min_y));
        first_points.push_back(Point (MF[0].max_x, MF[0].min_y));
        third_points.push_back(Point (MF[1].min_x, MF[1].min_y));
        second_points.push_back(Point (MF[1].min_x, MF[1].max_y));
        first_points.push_back(Point (MF[1].max_x, MF[1].max_y));
      }
      else
      {
        third_points.push_back(Point (MF[1].min_x, MF[1].max_y));
        second_points.push_back(Point (MF[1].min_x, MF[1].min_y));
        first_points.push_back(Point (MF[1].max_x, MF[1].min_y));
        third_points.push_back(Point (MF[0].min_x, MF[0].min_y));
        second_points.push_back(Point (MF[0].min_x, MF[0].max_y));
        first_points.push_back(Point (MF[0].max_x, MF[0].max_y));
      }
    }
    //cout << "publish the data " << endl;
    for (int i = 0; i < 2; i++)
    {
       car_scanner::Wheel wheel;

       wheel.first_point.x = first_points[i].x;
       wheel.first_point.y = first_points[i].y;
       wheel.second_point.x = second_points[i].x;
       wheel.second_point.y = second_points[i].y;
       wheel.third_point.x = third_points[i].x;
       wheel.third_point.y = third_points[i].y;

       wheelarray.wheels.push_back(wheel);
    }
    double size1 = first_points[0].distance(second_points[0]); 
    double size2 = first_points[1].distance(second_points[1]);
    if (p_size_mode == 0)
      wheelarray.size = (size1 + size2)/2;
    if (p_size_mode == 1)
      wheelarray.size = max(size1, size2);
    if (p_size_mode == 2)
      wheelarray.size = min(size1, size2);
    double score = min(tmp1.confidence, tmp2.confidence);
    wheelarray.score = score;
    wheelarray.orientation = ori;
    wheelarray.header.frame_id = wheel_array_id_;
    wheelarray.header.stamp = scan_stamp;
    wheels_pub_.publish(wheelarray);
    return;
    //cout << "publish end" << endl;
  }
}


// input_points_ --(group)--> points_sets
void ObstacleExtractor::groupPoints(const double max_group_distance)
{
    typedef std::list<PointSet>::iterator PointSetIterator;
    typedef std::list<Point>::iterator PointIterator;

    // Initial point_set with the first point
    PointSet point_set;
    point_set.max_x = -100;
    point_set.min_x = 100;
    point_set.max_y = -100;
    point_set.min_y = 100;
    point_set.num_points = 1;
    point_set.group.push_back(input_points_.front());
    point_sets.push_back(point_set);

    // For each incoming point, try adding it to existing point_sets
    for (PointIterator point = ++input_points_.begin(); point != input_points_.end(); ++point)
    {
        double range = (*point).length();
        bool belong = false;
        int n_groups_belong_to = 0;
        PointSetIterator point_set_belong_to;

        // If the incoming point is close enough to a point in a point_set, then add it to the point_set
        // If the incoming point is close enough to multiple point_sets, then the point_sets also need to be combined
        for (PointSetIterator point_set_tmp = point_sets.begin(); point_set_tmp != point_sets.end(); ++point_set_tmp)
        { 

            // If the incoming point is close enough to a point in a point_set, then add it to the point_set
            for(PointIterator point_tmp = point_set_tmp->group.begin(); point_tmp != point_set_tmp->group.end(); ++point_tmp)
            {
                double distance = (*point - *point_tmp).length();
                // If this point is close to a point_set, then add the point to the point_set
                if (distance < max_group_distance + range * p_distance_proportion_)
                {   
                    // If this point doesn't belong to any point_set, add it to the point_set
                    if (belong == false)
                    {
                        point_set_tmp->group.push_back(*point);
                        belong = true;
                        n_groups_belong_to ++;
                        point_set_belong_to = point_set_tmp;

                        point_set_tmp->num_points++;
                        if (point->x > point_set_tmp->max_x)
                          point_set_tmp->max_x = point->x;
                        if (point->x < point_set_tmp->min_x)
                          point_set_tmp->min_x = point->x;
                        if (point->y > point_set_tmp->max_y)
                          point_set_tmp->max_y = point->y;
                        if (point->y < point_set_tmp->min_y)
                          point_set_tmp->min_y = point->y;
                        break;
                    }
                    // Else if this point already belongs to another point_set, add the previous point_set to the current point_set
                    else
                    {
                        point_set_tmp->append(*point_set_belong_to);
                        point_sets.erase(point_set_belong_to);
                        point_set_belong_to = point_set_tmp;
                        break;
                    }
                }
            }
        }

        // If the incoming point doesn't belong to any point_set, then a new point_set is created for the point
        if (belong == false) 
        { 
            PointSet point_set;
            point_set.max_x = point->x;
            point_set.min_x = point->x;
            point_set.max_y = point->y;
            point_set.min_y = point->y;
            point_set.num_points = 1;
            point_set.group.push_back(*point);
            point_sets.push_back(point_set);
        }
    }
}

void ObstacleExtractor::checkScan()
  {
    ros::Duration  time_interval = ros::Time::now() - scan_stamp;
    // cout << "the scan duration is " << time_interval.toSec() << endl;
    if (time_interval.toSec() > p_scan_timeout)
    {
      error_out.error_code = err_no.LID_IDF_NO_DATA;
      error_out.message = "Lidar identification module does not receive any lidar data!";
      error_pub.publish(error_out);
    }
    // To do:publish the error if time_interval is higher than threshold. 
  }
