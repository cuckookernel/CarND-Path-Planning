#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <vector>
#include <cmath>
#include <iostream>
#include <tuple>

#include "json.hpp"
#include "spline.h"

#include "constants.h"

using std::vector;
using nlohmann::json;

// A point in 2D euclidean space
// and some basic vector operations to make working with it more pleasant
class Point2D {
  public: 
    double x; 
    double y;  
    
    explicit Point2D() : x(NaN), y(NaN) {}; 
    Point2D( double x_, double y_ ) : x(x_), y(y_) {}; 

  /*
    inline Point2D& set_t( double t_)  {
      t = t_;
      return *this;
    } */ 

    inline Point2D operator+( const Point2D& other ) const {
      return  {x + other.x, y + other.y};
    }

    inline Point2D operator-( const Point2D& other ) const {
      return  {x - other.x, y - other.y};
    }

    inline double dot( const Point2D& other ) const {
      return  x * other.x + y * other.y; 
    }

    inline Point2D operator*( double scalar ) const {
      return {x * scalar, y * scalar};
    }

    inline Point2D operator/( double scalar ) const {
      return {x / scalar, y / scalar};
    }

    inline double dist_to( const Point2D& other ) const {
        auto diff = other - (*this);
        return sqrt( diff.dot( diff ) );
    }

    inline Point2D rotate( double theta ) const {
      return Point2D( x * cos(theta) - y * sin(theta),
                      x * sin(theta) + y * cos(theta)  );
    }


    inline double norm( ) const {
      return sqrt( x * x  + y * y );
    }

    inline double fst() const {
      return x; 
    }

    inline double snd() const {
      return y; 
    }

};

inline Point2D operator*( double scalar, const Point2D& p ) {
  return {scalar * p.x, scalar * p.y};
}

std::ostream& operator<<( std::ostream& os, const Point2D& p ) {
  return (os << "P(" << p.x << "," << p.y << ")");
}

inline Point2D unit( double theta ) {
  return {cos(theta), sin(theta)};
}

inline double atan2( const Point2D& p2, const Point2D& p1 ) {
  return atan2( p2.y - p1.y, p2.x - p1.x );
}

inline double atan2( const Point2D& p ) {
  return atan2( p.y, p.x);
}

// A point in Frenet coordinates
class PointFr {
  public: 
    double s; 
    double d;
    double t;

  PointFr( double s_, double d_ ) : s(s_), d(d_), t(NaN) {}
  PointFr( double s_, double d_, double t_ ) : s(s_), d(d_), t(t_) {}
  PointFr( ) : s(NaN), d(NaN), t(NaN) {}

  inline PointFr operator-( const PointFr& other ) const {
      return  {s - other.s, d - other.d};
  }

  inline PointFr operator/( const double& scalar ) const {
      return  {s / scalar, d / scalar };
  }

  inline double fst() const {
      return s; 
  }

  inline double snd() const {
      return d; 
  }

  inline double norm() const {
      return sqrt( s*s + d * d); 
  }
};


inline double atan2( const PointFr& p ) {
  return atan2( p.s, p.d);
}
// project a "local xy-vector"  (such as a velocity or acceleration) to 
// a frenet "local vector" given the local unit vectors  n_s and n_d
inline PointFr project( Point2D p_xy, Point2D n_s, Point2D n_d ) {
  return PointFr( p_xy.dot(n_s), p_xy.dot(n_d) );
}

std::ostream& operator<<( std::ostream& os, const PointFr& p ) {
  return (os << "F(" << p.s << ", " << p.d << ")");
}
class Trajectory2D {

  vector<double> _xs;
  vector<double> _ys; 
  vector<double> _ts; 

  public: 
  int size() const {
    return _xs.size();
  }

  inline std::pair<double,double> operator[]( int i ) const {
    return std::make_pair(_xs[i], _ys[i]); 
  }

  inline std::tuple<double,double,double> at( int i ) const {
    return std::make_tuple(_xs[i], _ys[i], _ts[i]);
  }

  inline Point2D as_point( int i ) const {
    return Point2D(_xs[i], _ys[i]);
  }

  inline void reserve( int n ) {
    _xs.reserve(n);
    _ys.reserve(n);
    _ts.reserve(n);
  }

  void append( double x, double y, double t) { // Python for ever!
    _xs.push_back(x);
    _ys.push_back(y);
    _ts.push_back(t);
  }

  void append( const Point2D& p, double t) { // Python for ever!
    _xs.push_back(p.x);
    _ys.push_back(p.y);
    _ts.push_back(t);
  }

  void extend( const vector<Point2D>& pts, double t_default ) {
    reserve( pts.size() );
    // No loops, because that's how I roll!
    std::transform( pts.begin(), pts.end(), std::back_inserter(_xs), [](const Point2D& p) -> double { return p.x; } );
    std::transform( pts.begin(), pts.end(), std::back_inserter(_ys), [](const Point2D& p) -> double { return p.y; } );
    std::transform( pts.begin(), pts.end(), std::back_inserter(_ts), [&](const Point2D& p) -> double { return t_default; } );
    
  } 

  const vector<double>& xs() const { return _xs; };
  const vector<double>& ys() const { return _ys; };
  const vector<double>& ts() const { return _ts; };
  
  Trajectory2D interpolate(double dt) {
    tk::spline sx;
    tk::spline sy; 
    
    sx.set_points( _ts, _xs );
    sy.set_points( _ts, _ys );
    
    double min_tm = _ts[0];
    double max_tm = _ts[size()-1];

    Trajectory2D ret;         
    ret.reserve( (int) ceil( (max_tm - min_tm) / dt ) + 1 );

    for( double t=0.0; t < max_tm; t += dt ) {
      ret.append( sx(t), sy(t), t );    
    }

    return ret;    
  } 
};

class MapUtil {
  public: 

  MapUtil() {};
  
  vector<Point2D> xys;
  vector<double>  ss; 
  vector<Point2D> dxys;  

  // essentially the same as getXY but rewritten to use Point2D's primitives and members of this class
  Point2D toXY(double s, double d) const {
    int prev_wp = -1;

    while (s > ss[prev_wp+1] && (prev_wp < (int)(ss.size()-1))) {
      ++prev_wp;
    }

    int wp2 = (prev_wp+1) % xys.size();

    double heading = atan2(xys[wp2], xys[prev_wp]);
    // the x,y,s along the segment
    double seg_s = (s - ss[prev_wp]);
    auto seg = xys[prev_wp] + seg_s * unit(heading);    
    double perp_heading = heading - pi()/2;

    return  seg + d * unit( perp_heading );
  }

  // return a pair of unit vectors in (x,y)-coordinates 
  // that point in the direction of  the road (increasing s and constant d), 
  // and the perpendicular direction constant s, increasing d
  std::pair<Point2D, Point2D> localFrAxes( PointFr &p ) const {
    int prev_wp = -1;

    while (p.s > ss[prev_wp+1] && (prev_wp < (int)(ss.size()-1))) {
      ++prev_wp;
    }
    int wp2 = (prev_wp+1) % xys.size();
    double heading = atan2(xys[wp2], xys[prev_wp]);
    double perp_heading = heading - pi()/2; 

    return std::make_pair( unit(heading), unit(perp_heading) );
  }

  Trajectory2D toXY( const vector<double>& ss, const vector<double>& ds, const vector<double>& ts ) const {

    Trajectory2D  ret; 
    int n_points = ss.size();
    ret.reserve( n_points );
    for( int i =0; i < n_points; i++ ) {
      Point2D p = toXY(ss[i], ds[i]);
      ret.append( p.x, p.y, ts[i] );
    }

    return ret;
  }

  // essentially the same as ClosestWaypoint but rewritten to use Point2D's primitives and members of this class
  int closestWaypoint(const Point2D& p) const {
    double closestLen = std::numeric_limits<double>::infinity(); 
    int closest_idx = 0;

    for (int i = 0; i < xys.size(); ++i) {
      auto map_p = xys[i];      
      double dist = p.dist_to( map_p );
      if (dist < closestLen) {
        closestLen = dist;
        closest_idx = i;
      }
    }

    return closest_idx;
  }


  // Returns next waypoint of the closest waypoint
  // following exactly the same logic as NextWayPoint but using Point2D's capabilities and this class's members
  int nextWaypoint(const Point2D& p, double theta) const {
    int closest_idx = closestWaypoint(p);

    auto map_p = xys[closest_idx];    
    double heading = atan2(map_p, p);

    double angle = fabs(theta - heading);
    angle = std::min(2*pi() - angle, angle);

    if (angle > pi()/2) {
      ++closest_idx;
      if (closest_idx == xys.size()) {
        closest_idx = 0;
      }
    }

    return closest_idx;
  }

  
// Transform from Frenet s,d coordinates to Cartesian x,y
  /* vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                      const vector<double> &maps_x, 
                      const vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
      ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                           (maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
  } */ 


};

class TrajectoryFr {

  vector<double> ss; 
  vector<double> ds;
  vector<double> ts; 

  public: 
    void reserve( int n ) {
      ss.reserve(n);
      ds.reserve(n);
      ts.reserve(n);

    }
    void append( double s, double d, double t ) {
      ss.push_back( s );
      ds.push_back( d );
      ts.push_back( t );
    }    

    int size() const {
      return ss.size();
    }

    std::pair<double,double> operator[]( int i ) const {
      return std::make_pair(ss[i], ds[i]);
    }

    std::tuple<double,double,double> at( int i ) const {
      return std::make_tuple(ss[i], ds[i], ts[i]);
    }

    Trajectory2D toXY( const MapUtil& map ) const {
      return map.toXY( ss, ds, ts );
    }
    
};

// A record obtained from sensor fusion data 
// applies to other Cars
//  [ id, x, y, vx, vy, s, d]
struct OtherCarInfo {
  const int id;
  const Point2D pos; // ( x,  y) - pos 
  const Point2D v; // (vx, vy) - velocity
  const double  speed;
  const PointFr fr; // ( s,  d)
  
  OtherCarInfo( int id_, double x, double y, double vx, double vy, double s, double d ) :
     id(id_), pos(Point2D(x,y)), v(Point2D(vx,vy)), speed(Point2D(vx,vy).norm()), fr(PointFr(s,d)) {};
};

struct MsgInfo {

  Point2D car_p;
  PointFr car_fr;
  double car_yaw; // in radians!
  double car_speed; // in m/s
  double end_path_s;
  double end_path_d;
  
  json previous_path_x;
  json previous_path_y;

  int prev_size = 0;

  vector<OtherCarInfo> other_cars;
};

struct State {

  int lane = 1; // the lane {0,1,2} we are at or trying to go to
  double ref_speed = 0.; // in m / s
  double ref_yaw = 0.; // rad
  int prev_size = 0; 

  double clock = 0; // clock in seconds since start 

  double clock_last_lane_change = 0; 
};

#endif 