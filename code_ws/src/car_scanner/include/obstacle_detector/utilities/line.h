#pragma once

#include <vector>
#include <math.h>
#include "point.h"

namespace obstacle_detector
{

class Line
{
public:
  Line(Point p1 = Point(), Point p2 = Point()) {
    // Swap if not counter-clockwise
    if (p1.cross(p2) > 0.0)
      first_point = p1, last_point = p2;
    else
      first_point = p2, last_point = p1;
  }

  double length() const {
    return (last_point - first_point).length();
  }

  double orientation() const {
    return atan((last_point.y - first_point.y) / (last_point.x - first_point.x));
}
  
  double lengthSquared() const {
    return (last_point - first_point).lengthSquared();
  }

  Point normal() const {
    return (last_point - first_point).perpendicular().normalized();
  }

  Point projection(const Point& p) const {
    Point a = last_point - first_point;
    Point b = p - first_point;
    return first_point + a.dot(b) * a / a.lengthSquared();
  }

  Point trueProjection(const Point& p) const {
    Point a = last_point - first_point;
    Point b = p - first_point;
    Point c = p - last_point;

    double t = a.dot(b) / a.lengthSquared();

    if (t < 0.0)
      return (first_point);
    else if (t > 1.0)
      return (last_point);
    else
      return first_point + a.dot(b) * a / a.lengthSquared();
  }

  double distanceTo(const Point& p) const {
    return (p - projection(p)).length();
  }

  double trueDistanceTo(const Point& p) const {
    Point a = last_point - first_point;
    Point b = p - first_point;
    Point c = p - last_point;

    double t = a.dot(b) / a.lengthSquared();

    if (t < 0.0)
      return b.length();
    else if (t > 1.0)
      return c.length();

    Point projection = first_point + t * a;
    return (p - projection).length();
  }
  friend bool operator== (const Line& l1, const Line& l2) { return (l1.first_point == l2.first_point && l1.last_point == l2.last_point); } 
  friend bool operator<  (const Line& l1, const Line& l2) { return (l1.first_point.x <= l2.first_point.x); }
  friend std::ostream& operator<<(std::ostream& out, const Line& l) {
    out << "p1: " << l.first_point << ", p2: " << l.last_point;
    return out;
  }

  Point first_point;
  Point last_point;
};

} // namespace obstacle_detector
