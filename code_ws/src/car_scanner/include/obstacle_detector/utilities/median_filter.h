#pragma once

#include "point_set.h"
#include "point.h"
#include <math.h>


namespace obstacle_detector
{
class MedianFilter
{
public:
 
  MedianFilter(){};
  ~MedianFilter(){};

  double max_x;
  double min_x;
  double max_y;
  double min_y; 
  double orientation;
  int num_points;
  Point feature_point;
  Point top_point;
  Point bottom_point;
  Point middle_point;

  void get_data(PointSet point_set)
  {
    max_x_array.push_back(point_set.max_x);
    min_x_array.push_back(point_set.min_x);
    max_y_array.push_back(point_set.max_y);
    min_y_array.push_back(point_set.min_y);
    feature_point = point_set.feature_point;
    top_point = point_set.top_point;
    bottom_point = point_set.bottom_point;
    num_points = point_set.num_points;
  }
  
  void filter()
  {
    std::sort(max_x_array.begin(), max_x_array.end());
    std::sort(min_x_array.begin(), min_x_array.end());
    std::sort(max_y_array.begin(), max_y_array.end());
    std::sort(min_y_array.begin(), min_y_array.end());
    
    max_x = max_x_array[0];
    min_x = min_x_array[0];
    max_y = max_y_array[0];
    min_y = min_y_array[0];
    
    // cout << "getting the middle_point" << endl;
    double k = tan(orientation);
    double x0 = top_point.x;
    double y0 = top_point.y;
    double x1 = bottom_point.x;
    double y1 = bottom_point.y;

    middle_point.x = (k*y1 + x1 - k*y0 + k*k*x0)/(k*k + 1);
    middle_point.y = (-1/k*middle_point.x) + y1 + 1/k*x1;
  }
  
  void clear()
  {
   max_x_array.clear();
   min_x_array.clear();
   max_y_array.clear();
   min_y_array.clear();
   }
private:
       
      std::vector<double> max_x_array;
      std::vector<double> min_x_array;
      std::vector<double> max_y_array;
      std::vector<double> min_y_array;
 
       
       
};
}
