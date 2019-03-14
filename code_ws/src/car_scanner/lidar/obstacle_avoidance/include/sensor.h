// the file is used to describe lidar sensor 
#include <stdlib.h>
#include <math.h>
#include "../../utilities/point.h"
#include "../../utilities/point_set.h"
#include "region.h"
#include <mutex>


class Sensor
{
public:
    Sensor(){}
    ~Sensor(){}

    // load transformation parameters
    void load_parameters(const vector<double>& transformation)
    {
        tran.clear();
        for (int i = 0; i < transformation.size(); i++)
        {
            tran.push_back(transformation[i]);
            //cout << "the tran is " << tran.back() << endl;
 	    }
    }

    // get the task security area
    void get_area_list(const Region& security_area)
    {
        vector<string> tmp = security_area.get_sensor_id();
        polygon = security_area.get_polygon();
        called = false;
        for (int i = 0; i < tmp.size(); i++)
        {
            if (id == tmp[i])
            {
                // cout << "the sensor " << id << " is working!" << endl;
                work_area.copy(security_area);
                called = true;
                break;
            }
        }
    	
    }

    void get_data(const std::vector<Point>& input_points)
    {
       scan_data_mutex.lock();
       scan_update = true;
       scan_data.clear();
       for (int i = 0; i < input_points.size(); i++)
       {
        scan_data.push_back(input_points[i]);
//        if (scan_data[i].y < 0.3 && scan_data[i].y > -0.3)
//            cout << "the target point is " << scan_data[i] << endl;
       }
//       cout << "the get data size is " << scan_data.size() << endl;
       scan_data_mutex.unlock();
    }

    // check whether the cloud points are in security area 
    vector<Point> process_data()
    {
        if (running == true && (called == true || scan_update == true))
        {
            obstacle = false;
            scan_data_mutex.lock();
            collision_points.clear();
            // cout << "the process size is " << scan_data.size() << endl;
            for (Point p : scan_data)
            {
            
                Point tp;
                // cout << "the raw point is " << p << endl;
                tp.x = cos(tran[2]) * p.x - sin(tran[2]) * p.y + tran[0];
                tp.y = sin(tran[2]) * p.x + cos(tran[2]) * p.y + tran[1];
                // if (id == "laser_1")
                    // cout << "the trans point is " << tp.x << " " << tp.y << endl;
                if (check(tp) == true)
                {
                    obstacle = true;
                    // cout << "the raw point is " << p << endl;
                    // cout << "the collision point is " << tp << endl;
                    collision_points.push_back(p);
                    //break;
                }
            }
            scan_data_mutex.unlock();
            // cout << "the obstacle is " << obstacle << endl;
            
        }
        scan_update = false;
        return collision_points;
    }
    
    // implement the collision checking algorithm 
    bool check(const Point& tp)
    {
        // cout << "checking" << endl;
        if (work_area.get_type() > 0) // Not TURNING
        {
            int count = 0;
            int i = 0;
            int n = polygon.size();
            // cout << "polygon size is " << polygon.size() << endl;
            Point extr;
            extr.x = tp.x;
            extr.y = 10000.0;
            do
            {
                int next = (i+1)%n;
                if (doIntersect(polygon[i], polygon[next], tp, extr))
                {
                    // If the point 'p' is colinear with line segment 'i-next',
                    // then check if it lies on segment. If it lies, return true,
                    // otherwise false
                    if (orientation(polygon[i], tp, polygon[next]) == 0)
                       return onSegment(polygon[i], tp, polygon[next]);
         
                    count++;
                }
                i = next;
            } while (i != 0);
            // cout << "finish check " << endl;
            return count&1;
        }
        else                        // TURNING
        {
            // cout << "the r is " << work_area.get_circle().r << endl;
            // cout << "the x is " << work_area.get_circle().x << endl;
            // cout << "the y is " << work_area.get_circle().y << endl;
            if(sqrt(pow((tp.x - work_area.get_circle().x), 2) + pow((tp.y - work_area.get_circle().y),2)) <= work_area.get_circle().r)
            {
                return true;
            }
        }
        return false;
    }

    bool onSegment(const Point& p, const Point& q, const Point& r)
    {
        if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
                q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
            return true;
        return false;
    }
 
    // To find orientation of ordered triplet (p, q, r).
    // The function returns following values
    // 0 --> p, q and r are colinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    int orientation(const Point& p, const Point& q, const Point& r)
    {
        double val = (q.y - p.y) * (r.x - q.x) -
                  (q.x - p.x) * (r.y - q.y);
     
        if (val == 0.0) return 0;  // colinear
        return (val > 0.0)? 1: 2; // clock or counterclock wise
    }
 
    // The function that returns true if line segment 'p1q1'
    // and 'p2q2' intersect.
    bool doIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2)
    {
        // Find the four orientations needed for general and
        // special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;

        // p1, q1 and p2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;

         // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // Doesn't fall in any of the above cases
    }

    bool get_result() const
    {
        return obstacle;
    }
    
    bool get_running() const
    {
        return running && (!b_force_off);
    }

    bool get_called() const
    {
        return called;
    }

    void turn_off()
    {
        running = false;
    }

    void turn_on()
    {
        running = true;
    }

    void set_force_off(bool force_off)
    {
        b_force_off = force_off;
    }

    string get_id() const
    {
        return id;
    }

    void setup(const string id_, const bool running_, const bool called_, const bool obstacle_, const bool scan_update_)
    {
         id = id_;
         running = running_;
         called = called_;
         obstacle = obstacle_;
         scan_update = scan_update_;
         b_force_off = false;
    }

    void show_running_status()
    {
        cout << "the sensor " << id << " is working!" << endl;
    }
    
private:
    std::mutex scan_data_mutex; 
    string id;
    bool running;
    bool b_force_off;       // sensor forced to be turned off
    bool called;
    bool scan_update;
    bool obstacle; // the variable represent whether there is obstacle
    std::vector<Point> scan_data;
    std::vector<Point> point_sets;
    Region work_area;
    vector<Point> polygon;

    vector<double> tran; // the coordinates transformation from robot base to sensor 
    vector<Point> collision_points;  

};
