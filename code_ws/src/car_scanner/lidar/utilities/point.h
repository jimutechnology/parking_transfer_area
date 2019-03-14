

#pragma once

#include <cmath>
#include <iostream>

namespace utility
{

class Point
{
public:
  Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}
  Point(const Point& p) : x(p.x), y(p.y) {}
  static Point fromPoolarCoords(const double r, const double phi) { return Point(r * cos(phi), r * sin(phi)); }

  double distance(const Point &b) {
    return sqrt(pow(this->x - b.x, 2) + pow(this->y - b.y, 2));
  }
  
  double distance(const Point *b) {
    return sqrt(pow(this->x - b->x, 2) + pow(this->y - b->y, 2));
  }
  double length()        const { return sqrt(pow(x, 2.0) + pow(y, 2.0)); }
  double lengthSquared() const { return pow(x, 2.0) + pow(y, 2.0); }
  double angle()         const { return atan2(y, x); }
  double angleDeg()      const { return 180.0 * atan2(y, x) / M_PI; }
  double dot(const Point& p)   const { return x * p.x + y * p.y; }
  double cross(const Point& p) const { return x * p.y - y * p.x; }

  Point normalized() { return (length() > 0.0) ? *this / length() : *this; }
  Point reflected(const Point& normal) const { return *this - 2.0 * normal * (normal.dot(*this)); }
  Point perpendicular() const { return Point(-y, x); }

  friend Point operator+ (const Point& p1, const Point& p2) { return Point(p1.x + p2.x, p1.y + p2.y); }
  friend Point operator- (const Point& p1, const Point& p2) { return Point(p1.x - p2.x, p1.y - p2.y); }
  friend Point operator* (const double f, const Point& p)  { return Point(f * p.x, f * p.y); }
  friend Point operator* (const Point& p, const double f)  { return Point(f * p.x, f * p.y); }
  friend Point operator/ (const Point& p, const double f)  { return (f != 0.0) ? Point(p.x / f, p.y / f) : Point(); }

  Point operator- () { return Point(-x, -y); }
  Point operator+ () { return Point( x,  y); }

  Point& operator=  (const Point& p) { if (this != &p) { x = p.x; y = p.y; } return *this; }
  Point& operator+= (const Point& p) { x += p.x; y += p.y; return *this; }
  Point& operator-= (const Point& p) { x -= p.x; y -= p.y; return *this; }

  friend bool operator== (const Point& p1, const Point& p2) { return (p1.x == p2.x && p1.y == p2.y); }
  friend bool operator!= (const Point& p1, const Point& p2) { return !(p1 == p2); }
  friend bool operator<  (const Point& p1, const Point& p2) { return (p1.lengthSquared() < p2.lengthSquared()); }
  friend bool operator<= (const Point& p1, const Point& p2) { return (p1.lengthSquared() <= p2.lengthSquared()); }
  friend bool operator>  (const Point& p1, const Point& p2) { return (p1.lengthSquared() > p2.lengthSquared()); }
  friend bool operator>= (const Point& p1, const Point& p2) { return (p1.lengthSquared() >= p2.lengthSquared()); }
  friend bool operator!  (const Point& p1) { return (p1.x == 0.0 && p1.y == 0.0); }

  friend std::ostream& operator<<(std::ostream& out, const Point& p)
  { out << "(" << p.x << ", " << p.y << ")"; return out; }

  double x;
  double y;
};

}
