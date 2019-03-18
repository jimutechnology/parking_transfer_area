// this file is used to convert the point cloud data into image format
#include <stdlib.h>
#include "ros/ros.h"
#include <math.h>
#include "point.h"
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>  
using namespace std;
using namespace obstacle_detector;

#pragma once

namespace obstacle_detector
{
class Converter
{
private:
  std::list<Point> laser_points;
  Eigen::Matrix2i rotation1;
  Eigen::Matrix2i rotation2;
  Eigen::Matrix2i rotation3;
  Eigen::RowVector2i translation;
  Point feature_point;
  int width;
  int length;

public:
	Converter() : width(50), length(50) {}
	~Converter(){}

 void get_data(const std::list<Point>& input_points, const Point& Feature_point)
{
  typedef std::list<Point>::const_iterator PointIterator;
  //cout << "go to clean" << endl;
  laser_points.clear();
 //cout << "clean list" << endl;
  for (PointIterator point = input_points.begin(); point != input_points.end(); point++)
  {
    // cout << "the get_data point is " << *point << endl; 
    laser_points.push_back(*point);
  }
  feature_point = Feature_point;
  //cout << "get data finished" << endl;
}
 void convert(cv::Mat& image)
{
  rotation1 << 0, -1,
  1, 0;

  rotation2 << -1, 0,
  0, -1;

  rotation3 << 0, -1, 
  -1, 0;

  translation << 0.5*width, 0.5*length;

  typedef std::list<Point>::iterator PointIterator;
  for (PointIterator point = laser_points.begin(); point != laser_points.end(); point++)
  {
     double dx = point->x - feature_point.x;
     double dy = point->y - feature_point.y;
    
     int x = trunc(dx * width);
     int y = trunc(dy * length);
     
     if (x < 0.5*length && x > -0.5*length && y > -0.5*width && y < 0.5*width)
     {
     Eigen::RowVector2i v(x, y);
     Eigen::RowVector2i w;
     w = v * rotation3 + translation;
     //cout << "the w is " << w << endl;
     image.at<float>(w(0,0), w(0,1)) = 65535;
     }
  }
   // debugging through displaying the converted image 
   // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
   // cv::imshow( "Display window", image);                   // Show our image inside it.
   // cv::waitKey(0);                                          // Wait for a keystroke in the window
  //cout << "conversion finished" << endl;
}

};
}
