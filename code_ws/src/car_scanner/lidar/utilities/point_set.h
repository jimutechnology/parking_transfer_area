

#pragma once

#include <list>
#include "point.h"
#include <stdlib.h>
#include <math.h>
using namespace std;
using namespace utility;

typedef std::vector<Point>::iterator PointIterator;

class PointSet
{
public:
  PointSet() : num_points(0), is_visible(false), max_x(-100), min_x(100), max_y(-100), min_y(100) {}

  PointIterator begin, end;    // The iterators point to the list of points existing somewhere else

  std::vector<Point> group;

  double max_x;
  double min_x;
  double max_y;
  double min_y;
  double orientation; 
  double length;
  Point feature_point;
  Point short_fp;
  Point top_point;
  Point bottom_point;
  Point middle_point;

  int class_result;

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
   begin = point_set.begin;
   end = point_set.end;
   group.clear();
   for (PointIterator point = point_set.group.begin(); point != point_set.group.end(); point++)
  {
    group.push_back(*point);
  }
  }

  Point get_feature_points()
  {
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
   for (PointIterator point = group.begin(); point != group.end(); point++)
   {
    //cout << "the point is " << *point << endl;
   }

   if (group.begin()->x > (--group.end())->x)
   {
    for (PointIterator point = --group.end(); point != group.begin(); point--)
    {
      i++;
      //cout << "the short_fp_end is " << *point << endl; 
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
      //cout << "the short_fp_begin is " << *point << endl; 
      sx += point->x;
      sy += point->y;
      if (i == 8)
        break;
    }
   }
   short_fp.x = sx/8;
   short_fp.y = sy/8;
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
  double x0 = top_point.x;
  double y0 = top_point.y;
  double x1 = bottom_point.x;
  double y1 = bottom_point.y;

  middle_point.x = (k*y1 + x1 - k*y0 + k*k*x0)/(k*k + 1);
  middle_point.y = (-1/k*middle_point.x) + y1 + 1/k*x1;
  return feature_point;
  }

  
};



