#pragma once

#include <list>
#include "point.h"
#include "line.h"
#include <stdlib.h>
#include <math.h>
using namespace std;
namespace obstacle_detector
{

typedef std::list<Point>::iterator PointIterator;
typedef std::list<Line>::iterator LineIterator;

class PointSet
{
public:
  PointSet() : num_points(0), is_visible(false), max_x(-100), min_x(100), max_y(-100), min_y(100) 
  {
    num_points = 0;
    feature_point.x = 100;
    feature_point.y = 100;
    short_fp.x = 100;
    short_fp.y = 100;
    top_point.x = 0;
    top_point.y = 0;
    bottom_point.x = 0;
    bottom_point.y = 0;
    middle_point.x = 0;
    middle_point.y = 0;
    confidence = 0;
  }
  ~PointSet(){}

  std::list<Point> group;

  double max_x;
  double min_x;
  double max_y;
  double min_y;
  double orientation; // the direction of long edge
  double length;
  Point feature_point;
  Point short_fp;
  Point top_point;
  Point bottom_point;
  Point middle_point;

  int class_result;
  double confidence;

  int num_points;
  bool is_visible;  // The point set is not occluded by any other point set

  void copy(PointSet& point_set)
  {
   max_x = point_set.max_x;
   min_x = point_set.min_x;
   max_y = point_set.max_y;
   min_y = point_set.min_y;
   num_points = point_set.num_points;
   is_visible = point_set.is_visible;
   class_result = point_set.class_result;
   confidence = point_set.confidence;
   feature_point = point_set.feature_point;
   short_fp = point_set.short_fp;
   top_point = point_set.top_point;
   bottom_point = point_set.bottom_point;
   middle_point = point_set.middle_point;
   group.clear();
   for (PointIterator point = point_set.group.begin(); point != point_set.group.end(); point++)
  {
    group.push_back(*point);
  }
  }


  // append another point_set to this
  PointSet& append(PointSet& point_set)
  {
      max_x = max(point_set.max_x, max_x);
      min_x = min(point_set.min_x, min_x);
      max_y = max(point_set.max_y, max_y);
      min_y = min(point_set.min_y, min_y);
      num_points += point_set.num_points;
      for (PointIterator ptr = point_set.group.begin(); ptr != point_set.group.end(); ptr++)
      {
        group.push_back(*ptr);
      }

      return *this;
  }
  
  Point get_feature_points()
  {
  if (group.front().x > group.back().x)
  {
   top_point = group.front();
   bottom_point = group.back();
  }
  else
  {
   top_point = group.back();
   bottom_point = group.front();
  }  
   double x = 0;
   double y = 0;
   for (PointIterator point = group.begin(); point != group.end(); point++)
  {
    //cout << "the point_x is" << point->x << endl;
    x += point->x;
    y += point->y;
  }
    x = x/num_points;
    y = y/num_points;
    feature_point.x = x;
    feature_point.y = y;
   double sx = 0;
   double sy = 0;
   int i = 0;

   if (group.begin()->x > (--group.end())->x)
   {
    for (PointIterator point = --group.end(); point != group.begin(); point--)
    {
      i++;
      // cout << "the short_fp_end is " << *point << endl; 
      sx += point->x;
      sy += point->y;
      if (i == 8)
        break;
    }
   }
   else
   {
    for (PointIterator point = group.begin(); point != group.end(); point++)
    {
      i++;
      // cout << "the short_fp_begin is " << *point << endl; 
      sx += point->x;
      sy += point->y;
      if (i == 8)
        break;
    }
   }
   short_fp.x = sx/8;
   short_fp.y = sy/8;
  return feature_point;
  }

  void get_rep_point()
  {
    if (group.front().x > group.back().x)
    {
       top_point = group.front();
       bottom_point = group.back();
    }
    else
    {
       top_point = group.back();
       bottom_point = group.front();
    }  
    double k = tan(orientation);
    cout << orientation << endl;
    double x0 = top_point.x;
    double y0 = top_point.y;
    double x1 = bottom_point.x;
    double y1 = bottom_point.y;

    middle_point.x = (k*y1 + x1 - k*y0 + k*k*x0)/(k*k + 1);
    middle_point.y = (-1/k*middle_point.x) + y1 + 1/k*x1;
  }

  double get_front_edge_dist(const PointSet& point_set) const
  {
    double k = -1/tan(orientation);
    double a = -middle_point.y + k*middle_point.x;
    double b = -point_set.middle_point.y + k*point_set.middle_point.x;
    double distance = abs(a-b)/sqrt(1 + pow(k, 2));
    return distance;
  }

  
};

} // namespace obstacle_detector

