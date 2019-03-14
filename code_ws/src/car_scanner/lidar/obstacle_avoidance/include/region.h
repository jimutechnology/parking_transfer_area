// this file is used to describe the security area(circle or polygon)
#include <stdlib.h>
#include "../../utilities/point.h"
#include <math.h>

// Macro definitions for TYPE of movement
#define TURN            0
#define LEFT_FORWARD    1 
#define RIGHT_FORWARD   2 
#define LEFT_BACKWARD   3
#define RIGHT_BACKWARD  4 
#define STANDBY         5

// Description of a circle
struct circle
{
    double x;     // Position X, [m]
    double y;     // Position Y, [m]
    double r;     // Radius, [m]
};

// Description of a Polygon
class Region
{
public:
    Region()
    {
      Point p1(0, 0);
      Point p2(0, 0);
      Point p3(0, 0);
      polygon.push_back(p1);
      polygon.push_back(p2);
      polygon.push_back(p3);
    }
    ~Region(){}

    // Load transformation into tran1
    void load_parameters_1(const vector<double>& transformation)
    {
        tran1.clear();
        for (int i = 0; i < transformation.size(); i++)
         {
            tran1.push_back(transformation[i]);
            //cout << "the tran1 is " << tran1.back() << endl;
          }
    }

    // Load transformation into tran2
    void load_parameters_2(const vector<double>& transformation)
    {
        tran2.clear();
        for (int i = 0; i < transformation.size(); i++)
         {
            tran2.push_back(transformation[i]);
            //cout << "the tran2 is " << tran2.back() << endl;
          }
    }

    // Load geometry of obstacle avoidance region
    void load_geometry(const double p_front_distance, const double p_back_distance, const double p_left_distance, const double p_right_distance, const double p_inflation_layer)
    {
        front_distance = p_front_distance;
        back_distance = p_back_distance;
        left_distance = p_left_distance;
        right_distance = p_right_distance;
        inflation_layer = p_inflation_layer;

        lf.x = front_distance + inflation_layer;
        lf.y = left_distance + inflation_layer;
        rf.x = front_distance + inflation_layer;
        rf.y = -(right_distance + inflation_layer);
        lr.x = -(back_distance + inflation_layer);
        lr.y = left_distance + inflation_layer;
        rr.x = -(back_distance + inflation_layer);
        rr.y = -(right_distance + inflation_layer);
    }

    void set_type(const int t)
    {
       type = t;
    }

    int get_type() const 
    {
       return type;
    }

 	  void set_sensor_id(const int id1, const int id2)
    {       
      sensor_id.clear();
      if (id1 != 0)
        sensor_id.push_back("laser_" + to_string(id1));
      else
        sensor_id.push_back("laser_" + to_string(-1));
      if (id2 != 0)
        sensor_id.push_back("laser_" + to_string(id2));
      else
        sensor_id.push_back("laser_" + to_string(-1));

      // For debug purpose
      // for (int i = 0; i < sensor_id.size(); i++)
      // {
      // cout << "the sensor_id is " << sensor_id[i] << endl;
      // }
    }

    vector<string> get_sensor_id() const
    {
       return sensor_id;
    }

   
    void set_region(const Point p1, const Point p2, const Point p3)
    {
      if (type == TURN)        // Turn
      {
        c.x = 0;
        c.y = 0;
        // ASSERT: left_distance == right_distance
        if (front_distance < back_distance)
        {
        c.r = sqrt(pow(left_distance, 2) + pow(back_distance, 2)) + inflation_layer;  
        }
        else
        {
        c.r = sqrt(pow(left_distance, 2) + pow(front_distance, 2)) + inflation_layer;
        }    
      }
      else if (type == LEFT_FORWARD)   // Move Left-Forward
      {
        polygon.clear();
        // construct the ploygon
        polygon.push_back(p1);
        polygon.push_back(p2);
        polygon.push_back(p3);

        polygon.push_back(rf);
        polygon.push_back(lf);
        polygon.push_back(lr);
      }
      else if (type == RIGHT_FORWARD)   // Move Right-Forward
      {
        polygon.clear();
        // construct the ploygon
        polygon.push_back(p1);
        polygon.push_back(p2);
        polygon.push_back(p3);

        polygon.push_back(rr);
        polygon.push_back(rf);
        polygon.push_back(lf);
      }
      else if (type == LEFT_BACKWARD)   // Move Left-Backward
      {
        polygon.clear();
        // construct the ploygon
        polygon.push_back(p1);
        polygon.push_back(p2);
        polygon.push_back(p3);

        polygon.push_back(lf);
        polygon.push_back(lr);
        polygon.push_back(rr);
      }
      else if (type == RIGHT_BACKWARD)   // Move Right-Backward
      {
        polygon.clear();
        // construct the ploygon
        polygon.push_back(p1);
        polygon.push_back(p2);
        polygon.push_back(p3);

        polygon.push_back(lr);
        polygon.push_back(rr);
        polygon.push_back(rf);
      }
      else if (type == STANDBY)   // STANDBY
      {
        polygon.clear();
        // construct the ploygon
        polygon.push_back(lf);
        polygon.push_back(rf);
        polygon.push_back(rr);
        polygon.push_back(lr);
      } 

    }

    vector<Point> get_polygon() const
    {
      return polygon;
    }

    circle get_circle() const
    {
       return c;
    }

    // Copy input Region to this object
    void copy(const Region& area)
    {
       type = area.get_type();
       vector<string> tmp;
       tmp = area.get_sensor_id();
       vector<Point> poly = area.get_polygon();
       sensor_id.clear();
       for (int i = 0; i < tmp.size(); i++)
       {
          sensor_id.push_back(tmp[i]);
       }
       polygon.clear();
       for (int i = 0; i < poly.size(); i++)
       {
          polygon.push_back(poly[i]);
       }
       c.x = area.get_circle().x;
       c.y = area.get_circle().y;
       c.r = area.get_circle().r;
    }

private:
  int   type;                 // Type of movement
  vector<string> sensor_id;   // Sensor id - frame
  circle c;                   // The Circle region
  vector<Point> polygon;      // The Region described as a Polygon, with a vector of Points
  Point lf;                   // The four obstacle avoidance corner points w.r.t. robot center
  Point rf;
  Point lr;
  Point rr;                                                                                                   
  double front_distance;      // Safety distance in the front, [m]
  double back_distance;       // Safety distance in the back, [m]
  double left_distance;       // Safety distance in the left, [m]
  double right_distance;      // Safety distance in the right, [m]
  double inflation_layer;     // Distance for the virtual inflation layer, which protects the robot on all sides, [m]
  vector<double> tran1;       // Transformation 1
  vector<double> tran2;       // Transformation 2
};
