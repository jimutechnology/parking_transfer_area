#include <stdlib.h>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include "point.h"
#include "point_set.h"

using namespace utility;
typedef std::vector<Point>::iterator PointIterator;
typedef std::vector<PointSet>::iterator PointSetIterator;

class LidarDataProcessor
{
public:
	 LidarDataProcessor(){}	
	 ~LidarDataProcessor(){}

    void setup(double p_distance_proportion, double p_noise_filter_num, double p_noise_filter_dis)
    {
    	p_distance_proportion_ = p_distance_proportion;
    	p_noise_filter_num_ = p_noise_filter_num;
    	p_noise_filter_dis_ = p_noise_filter_dis;
    }

    void filter_noise(std::vector<PointSet>& point_sets, std::vector<Point>& input_points)
    {
		groupPoints(point_sets, p_noise_filter_dis_, input_points);
		input_points.clear();
		for (PointSetIterator point_set = point_sets.begin(); point_set != point_sets.end(); ++point_set)
		{
			if (point_set->num_points > p_noise_filter_num_)
			{
				for (PointIterator point = point_set->group.begin(); point != point_set->group.end(); ++point)
				{
				  input_points.push_back(*point);
				}
			}
		}
		point_sets.clear();
    }

    void groupPoints(std::vector<PointSet>& point_sets, double max_group_distance, std::vector<Point>& input_points_)
    {
		typedef std::vector<PointSet>::iterator PointSetIterator;
		typedef std::vector<Point>::iterator PointIterator;
		PointSet point_set;
		point_set.max_x = -100;
		point_set.min_x = 100;
		point_set.max_y = -100;
		point_set.min_y = 100;
		point_set.num_points = 1;
		point_set.group.push_back(input_points_.front());
		point_sets.push_back(point_set);


		for (PointIterator point = ++input_points_.begin(); point != input_points_.end(); ++point)
		{
			double range = (*point).length();
			bool belong = false;
			for (PointSetIterator point_set_tmp = point_sets.begin(); point_set_tmp != point_sets.end(); ++point_set_tmp)
			{ 
				int i = 0;
				for(PointIterator point_tmp = point_set_tmp->group.begin(); point_tmp != point_set_tmp->group.end(); ++point_tmp)
				{
					double distance = (*point - *point_tmp).length();
				if (distance < max_group_distance + range * p_distance_proportion_)
				{
					point_set_tmp->group.push_back(*point);
					belong = true;
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
				}
			}
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
private:
     double p_distance_proportion_;
     double p_noise_filter_num_;
     double p_noise_filter_dis_;
};
