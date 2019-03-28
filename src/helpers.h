#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <numeric>
#include <iomanip>

#include "json.hpp"

using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double inf = std::numeric_limits<double>::infinity();

constexpr double MAX_JERK = 10.0; // m/s^3
constexpr double MAX_ACCEL = 10.0; // m/s^2
constexpr double DT = 0.02; // s
constexpr double MIN_CHANGE_LANE_TIME = 10.0; // s
constexpr double MIN_COLLISION_TIME = 5.0; // s
constexpr double MAX_V_D = 2.0; // m/s
constexpr double MAX_V = 47.0 * 1609.34 / 3600; // m/s ( aprox 22 m/s ) 
constexpr double LANE_WIDTH = 4.0; 


inline double sigmoid( double t, double beta ) {
   return  1.0 / ( 1.0 + exp(- beta * t) );   
}

template<typename Fn> 
void assert_fn( bool expr, Fn fn) {
  if (!expr) {
    fn();
    abort();
  }
}

// A point in 2D euclidean space
// and some basic vector operations to make working with it more pleasant
class Point2D {
  public: 
    double x; 
    double y;   

    explicit Point2D() : x(NaN), y(NaN) {}; 
    Point2D( double x_, double y_ ) : x(x_), y(y_) {}; 

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

  PointFr( double s_, double d_ ) : s(s_), d(d_) {}
  PointFr( ) : s(NaN), d(NaN) {}

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



// A record obtained from sensor fusion data 
//  [ id, x, y, vx, vy, s, d]
struct CarInfo {
  const int id;
  const Point2D pos; // ( x,  y) - pos 
  const Point2D v; // (vx, vy) - velocity
  const double  speed;
  const PointFr fr; // ( s,  d)
  
  CarInfo( int id_, double x, double y, double vx, double vy, double s, double d ) :
     id(id_), pos(Point2D(x,y)), v(Point2D(vx,vy)), speed(Point2D(vx,vy).norm()), fr(PointFr(s,d)) {};
};



struct MsgInfo {

  Point2D car_p;
  PointFr car_fr;
  //double yaw;
  double car_speed; 

  json previous_path_x;
  json previous_path_y;

  vector<CarInfo> other_cars;
};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2mps( double v_mph ) {  return v_mph * 1609 / 3600; }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}



// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1 ) % maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


class MapUtil {
  public: 

  MapUtil() {};
  
  vector<Point2D> xys;
  vector<double>  ss; 
  vector<Point2D> dxys;  

  // essentially the same as getXY but rewritten to use Point2D's primitives and members of this class
  Point2D toXY(const PointFr& p) const {
    int prev_wp = -1;

    while (p.s > ss[prev_wp+1] && (prev_wp < (int)(ss.size()-1))) {
      ++prev_wp;
    }

    int wp2 = (prev_wp+1) % xys.size();

    double heading = atan2(xys[wp2], xys[prev_wp]);
    // the x,y,s along the segment
    double seg_s = (p.s - ss[prev_wp]);

    auto seg = xys[prev_wp] + seg_s * unit(heading);
    
    double perp_heading = heading - pi()/2;

    return  seg + p.d * unit( perp_heading );
  }

  vector<Point2D> toXY( const vector<PointFr>& traj_fr ) const {

    vector<Point2D> ret; 
    ret.reserve( traj_fr.size() );
    for( auto p : traj_fr ) {
      ret.push_back( toXY(p) );
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


  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  PointFr toFrenet(const Point2D& p, double theta) {

    int next_wp = nextWaypoint(p, theta);
    int prev_wp = next_wp - 1;

    if (next_wp == 0) {
      prev_wp  = xys.size() - 1;
    }

    auto n = xys[next_wp] - xys[prev_wp];  
    auto x = p - xys[prev_wp];
    
    // find the projection of x onto n
    double proj_norm = x.dot(n) / n.dot(n);
    auto  proj = proj_norm * n;
    
    double frenet_d = x.dist_to(proj);

    //see if d value is positive or negative by comparing it to a center point
    // original
    // WEEIRD why 1000 and 2000 ? 
    // double center_x = 1000-maps_x[prev_wp];
    // double center_y = 2000-maps_y[prev_wp];

    auto center = Point2D(1000, 2000) - xys[prev_wp];
    
    double centerToPos = center.dist_to(x);
    double centerToRef = center.dist_to(proj);

    if (centerToPos <= centerToRef) {
      frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i) {
      frenet_s += xys[i].dist_to(xys[i+1]);
    }

    frenet_s += proj.norm();

    return {frenet_s,frenet_d};
}

};

// trace a path (in Frenet coordinata) that starts at point `start`,
// with velocity v0 and accelerates to v1 
// with a mean acceleration given by mean_a.
// it does this by first constructing a trapezoidal acceleration schedule for the s-component of the movement.
vector<PointFr> accelerate_to( const PointFr& start, double target_d, double v0, double v1, double target_tm ) {

  double T = 5;
  double a1 = (v1 - v0) / T;

  if( abs(target_d - start.d) < 2.0 || fabs(a1) > 0.8 * MAX_ACCEL || 3 * fabs( a1 ) / T > 0.8 * MAX_JERK ) {
    // this is not lane change or it is but the time is too short... 
    T = target_tm; 
    bool ok = false; 
    while( !ok ) {
        double delta_v = v1 - v0; 
        double mean_a = delta_v / T;   
        a1 = 6 * copysign( mean_a,  delta_v ) / 4;
        
        if ( fabs(a1) < 0.8 * MAX_ACCEL && (3 * fabs( a1 ) / T) < 0.8 * MAX_JERK  ) {
          ok = true; 
        } else {
          // if not okey try gain with a bigger time
          T *= 1.3;
        }        
    }
  }
  
  // an excesive value of mean_a will get us too high a jerk in the first and last sections 
  // of the trapezoid
  double initial_jerk  = 3 * fabs( a1 ) / T;
  assert_fn( initial_jerk <= MAX_JERK, 
            [&]() { cout << "delta_v= " << v1 - v0 << "a1= " << a1 
                    << " T= "<< T << " initial_jerk=" << initial_jerk << endl; }  );

  cout << "accelerate_to: T = " << T << "  a1= "<< a1 << " initial_jerk = " << initial_jerk << endl; 
  
  int n_points  = (int) ceil( T / DT );

  // Build vector longitudinal s coordinates
  // initial conditions
  double v = v0,  s = start.s;   
  double prev_a = 0; 
  double prev_v = v;
  
  vector<double> out_ss; 
  out_ss.reserve( n_points );
  
  for( double t = 0; t < T; t += DT  ){    

    double a = 0; 
    if ( t < T/3 ){  // first section of trapezoid: t in [0, T/3)
      a = 3 * (t / T) * a1;    
    } else if ( t >= T/3 && t < 2 * T / 3 ) {  // second section: t in [T/3, 2*T/3)
      a = a1;      
    } else { // third section: t in [2*T/3, T)
      a = a1 - 3 * (t/T  - 2.0/3.0) * a1;
    }

    v += ( a + prev_a) * 0.5 * DT;
    s += ( v + prev_v) * 0.5 * DT;
    prev_v = v;
    prev_a = a;  

    out_ss.push_back( s );
  }

  // now build vector of coordinates in the d direction following a sigmoid
  double d0 = start.d;
  vector<double> out_ds; 
  out_ds.reserve( n_points ); 

  for( double t = 0; t < T; t += DT  ){  
      double d = d0 + (target_d - d0) * sigmoid( t - T/2, MAX_V_D );
      out_ds.push_back( d ); 
  }

  // build final vector of frenet points
  vector<PointFr> ret;
  ret.reserve(n_points);

  assert_fn( out_ds.size() == out_ss.size(), 
             [&]() { cout << "Size of out_ds is " << out_ds.size() << " size of out_ss is " << out_ss.size() << endl; } );
  
  for( int i = 0; i < out_ds.size(); i++ ) {
    ret.push_back( {out_ss[i], out_ds[i]} );
  }

  cout << "accelerate_to: returning " << ret.size() << " points" << endl;
  return ret;
}

inline int getLane( const PointFr& f ) {
  return (int) floor( f.d  / LANE_WIDTH );
} 

template<typename PT>
bool examine_trajectory( const vector<PT>& traj, bool verbose = false ) {
  int n = traj.size();

  PT prev_p{0,0}; 
  PT prev_a{0,0}; 
  PT prev_v{0,0};
  bool success = true; 

  int i = 0;
  for( auto p : traj ) {
        
    auto v    = (i > 0 ? (p - prev_p) / DT : PT{0,0} );
    auto v_unit = v / v.norm();
    auto a    = (i > 1 ? (v - prev_v) / DT : PT{0,0} );
    double theta = atan2( v );
    double a_n   = (i > 1 ? a.norm() * cos(theta) : 0.0  );
    double a_d   = (i > 1 ? a.norm() * sin(theta) : 0.0  );
        
    auto jerk = (i > 2 ? (a - prev_a) / DT : PT{0,0} );

    if( (v.norm() > MAX_V) || ( a.norm() > 10 ) || (jerk.norm() > 10) ) {
      success = false;
    }
    
    if( verbose ) {
      cout << "i=" << i << ": " << std::fixed << std::setprecision(1) << p.fst() <<", "<< p.snd()
           << "  |v| = " << v.norm() 
           << "  |a| = " << a.norm()  << " a_n= " << a_n  << " a_d = " << a_d 
           << "  |j| = " << jerk.norm() 
           << ( v.norm() > MAX_V ? " max speed violation!" : "")  
           << ( a.norm() > 10    ? " max acceleration violation!" : "") 
           << ( jerk.norm() > 10 ? " max jerk violation! " : "") << endl;           
    }

    prev_p = p; 
    prev_v = v; 
    prev_a = a;
    i++;
  }

  return success; 
}

// Push a trajectory (vector<Point2D>) into to separate vectors of x and y coordinates.
// Optionally clear the vectors
void push_trajectory( const vector<Point2D>& traj, vector<double>& xs, vector<double>& ys, bool clear_first=false ) {  
  if( clear_first ) {
    xs.clear();
    ys.clear();
  }

  int n = traj.size();
  xs.reserve(n);
  ys.reserve(n);

  for( int i=0; i < n; i++) {
    auto p = traj[i];
    xs.push_back( p.x); 
    ys.push_back( p.y);
  }
}

double min_collision_time_for_lane( int lane_num, const MsgInfo& m, bool verbose ) {
  
  double d_l = lane_num * LANE_WIDTH;
  double d_r = (lane_num + 1) * LANE_WIDTH;

  double my_s = m.car_fr.s;
  double min_time = inf;

  for( auto other_car : m.other_cars ){

    // cout << " other_car( id= "<< other_car.id << ", d:  " << other_car.fr.d << " )" << endl; 

    if ( other_car.fr.d >= d_l &&  other_car.fr.d <= d_r ) {
      
      double other_s = other_car.fr.s;
      double speed_diff = ( m.car_speed - other_car.speed );
      double collision_time = (other_s - my_s) / speed_diff; 

      if( verbose ) {
        cout << "lane_num" << lane_num << "other_car " 
             << other_car.id << " speed_diff " << speed_diff << " coll_time " << collision_time << endl; 
      }

      if( collision_time >= 0 && collision_time < min_time ) {
         min_time = collision_time;
      }
    }
  }

  return min_time; 
}

#endif  // HELPERS_H