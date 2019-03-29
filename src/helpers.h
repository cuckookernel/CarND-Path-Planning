#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <numeric>
#include <iomanip>
#include <algorithm>

#include "json.hpp"
#include "spline.h"

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
constexpr double MAX_V_D = 4.0; // m/s
constexpr double MAX_V = 45.0 * 1609.34 / 3600; // m/s ( aprox 22 m/s ) 
constexpr double LANE_WIDTH = 4.0; 


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
  return (os << "P( " << p.x << ", " << p.y << " )");
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

inline int getLane( const PointFr& f ) {
  return (int) floor( f.d  / LANE_WIDTH );
} 

inline double atan2( const PointFr& p ) {
  return atan2( p.s, p.d);
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

  void append( double x, double y, double t) {
    _xs.push_back(x);
    _ys.push_back(y);
    _ts.push_back(t);
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


inline bool are_valid_accel_jerk( double a1, double T ) {
    return fabs(a1) < 0.8 * MAX_ACCEL && (3 * fabs( a1 ) / T) < 0.8 * MAX_JERK;
}

std::pair<double,double> solve_for_traj_time_v0( double start_d, double target_d, double v0, double v1, double target_tm ) {

  double T = NaN;
  
  if( abs(target_d - start_d) >= 2.0 ) {
    // this is a lane change 
    T = 5; 
  } else {
    T = target_tm;
  }

  double a1 = 1.5 * (v1 - v0) / T;  
  while( !are_valid_accel_jerk(a1, T) ) {
    T *= 1.3;         
    a1 = 1.5 * (v1 - v0) / T;                
  }
  
  double initial_jerk  = 3 * fabs( a1 ) / T;
  assert_fn( initial_jerk <= MAX_JERK, 
            [&]() { cout << "delta_v= " << v1 - v0 << "a1= " << a1 
                    << " T= "<< T << " initial_jerk=" << initial_jerk << endl; }  );

  cout << "accelerate_to: T = " << T << "  a1= "<< a1 << " initial_jerk = " << initial_jerk << endl;   
  return std::make_pair( T, a1 ); 
}

// trace a path (in Frenet coordinata) that starts at point `start`,
// with velocity v0 and accelerates to v1 
// with a mean acceleration given by mean_a.
// it does this by first constructing a trapezoidal acceleration schedule for the s-component of the movement.
TrajectoryFr accelerate_to_v0( const PointFr& start, double target_d, double v0, double v1, double target_tm, 
                            int sub_sample ) {
   
  auto sol = solve_for_traj_time_v0( start.d, target_d, v0, v1, target_tm ); 
  double T = sol.first, a1 = sol.second;   
  
  // Build vector longitudinal s coordinates
  // initial conditions
  double v = v0, s = start.s, prev_a = 0; 
  double prev_v = v;
  const double d0 = start.d;
      
  int n_points  = (int) ceil( T / DT );
  int n_returned = (n_points / sub_sample) + 1;

  TrajectoryFr ret; 
  ret.reserve( n_returned );

  for( int i=0; i < n_points; i++  ){    
    double t = i * DT; 

    double a = 0; 
    if ( t < T/3 ){  // first section of trapezoid: t in [0, T/3)
      a = 3 * (t / T) * a1;    
    } else if ( t >= T/3 && t < 2 * T / 3 ) {  // second section: t in [T/3, 2*T/3)
      a = a1;      
    } else { // third section: t in [2*T/3, T)
      a = a1 - 3 * (t/T  - 2.0/3.0) * a1;
    }

    v += ( a + prev_a ) * 0.5 * DT;
    s += ( v + prev_v ) * 0.5 * DT;
    prev_v = v;
    prev_a = a;  
    
    if ( i % sub_sample == 0 || (T - t) < DT ) {
        double d = d0 + (target_d - d0) * sigmoid( t - T/2, MAX_V_D );
        ret.append( s, d, t);        
    }    
  }

  cout << "accelerate_to_v0: returning " << ret.size() << " points" << endl;  
  return ret;
}


class QuarticTrajectoryGen  {

    const double _s0;
    const double _v0; 
    const double _a0;
    double _s3;
    double _s4;
    double _t1;

    public:

    /* assume a trajectory of the form 
        s(t) = v0 * t + a0 / 2 * t^2 +  s_3 / 6 * t^3 + s_4 / 24 * t^4

        Then v(t) = s'(t)  = v0 + a0  * t +  s_3 / 2 * t^2 + s_4 / 6 * t^3
        and  a(t) = a''(t) = a0  +  s_3 * t + s_4 / 2 * t^2

        Setting end conditions as: v(T) = v1, and a(T) = a1  we get the 2 x 2 system for s_3 and s_4 as follows 

          s_3 / 2 * T^2 + s_4 / 6 * T^3  = v1 - (v0 + a0 * T)
          s_3 * T + s_4 / 2 * T^ 2 =  a1 - a0

          Substituting  S_4 / 2 * T ^ 2 =  (a1-a0) - S_3 T in the first eq. yields 

          (T^2/2) * s_3  + ( a1 - a0 - S_3 T) * T/3 = v1 - (v0 + a0 * T), or 

          ( T^2 / 6 ) s_3 =  (v1 - v0) - (2/3) * a0 * T  - a1 * T/3 
     */

    QuarticTrajectoryGen( double s0_, double v0_, double a0_, double t1_ ) : 
      _s0( s0_ ), _v0( v0_ ), _a0( a0_ ), _t1( t1_ )  {}

    void solve( double v1, double a1, double t ) {
        _t1 = t;
        _s3 = 6 * ( (v1 - _v0) - (2.0/3.0) * _a0 * _t1 - a1 * _t1 / 3.0 ) / (_t1*_t1);
        _s4 = 2 * ( a1 - _a0 - _s3 * _t1 ) / (_t1 * _t1);
    }

    double max_jerk(  ) const {
      return std::max(fabs( _s3 ) , fabs(_s3 + _s4 * _t1) );
    }

    inline double t1() const {
      return _t1; 
    }

    double operator()( double t ) {
      // return _s0 + _v0 * t + _a0 * (t*t) / 2.0 + _s3 * (t*t*t) / 6.0 + _s4 * ( t*t*t*t) / 24.0;
      // save about 40% of flops:
      return ((( _s4 / 24.0 * t + _s3 / 6.0 ) * t + _a0 / 2.0 ) * t + _v0 ) * t + _s0;
    }
};

QuarticTrajectoryGen calibrate_traj_generator( const PointFr& p, double v0, double v1, double a0,
                                        double  target_tm ) {

    double t1 = target_tm; 

    QuarticTrajectoryGen  tgen( p.s, v0, a0, t1 );    
    
    do {
      tgen.solve( v1, 0, t1 );
      t1 *= 1.3;
    } while( fabs(tgen.max_jerk()) >= MAX_JERK );

    return tgen;
}

// trace a path (in Frenet coordinata) that starts at point `start`,
// with velocity v0 and accelerates to v1 
// with a mean acceleration given by mean_a.
// it does this by first constructing a trapezoidal acceleration schedule for the s-component of the movement.
TrajectoryFr accelerate_to( const PointFr& p, double target_d, 
                            double v0, double v1, double a0, double target_tm, 
                            int sub_sample ) {
   
  auto tgen = calibrate_traj_generator(p, v0, v1, a0, target_tm);
  double t1 = tgen.t1();

  cout << "accelerate_to: v0= " << v0 << "v1="<< v1 << "a0" << a0 
       << " t1 = " << t1 << endl;  
    
  const double d0 = p.d;
      
  int n_points   = (int) ceil( t1 / DT );
  int n_returned = (n_points / sub_sample) + 1;

  TrajectoryFr ret; 
  ret.reserve( n_returned );

  for( int i=0; i < n_points; i++ ){    
    double t = i * DT; 
    double s = tgen( t );
    
    if ( i % sub_sample == 0 || (t1 - t) < DT ) {
        double d = d0 + (target_d - d0) * sigmoid( t - t1/2, MAX_V_D );
        ret.append( s, d, t);        
    }    
  }

  cout << "accelerate_to: returning " << ret.size() << " points" << endl;  
  return ret;
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

std::pair<int, double> evaluate_lane_change( const MsgInfo& m, bool verbose ) {
  int cur_lane = getLane( m.car_fr );
  vector<int> lanes_to_consider;
  if( cur_lane == 0 ) {        
    lanes_to_consider = {0, 1};
  } else if( cur_lane == 1) {
    lanes_to_consider = {1, 0 ,2};
  } else if( cur_lane == 2) {
    lanes_to_consider = {2, 1};
  }

  int best_lane = cur_lane; 
  double max_min_collision_time = -inf;
  for( auto l : lanes_to_consider ) {
      double min_coll_time = min_collision_time_for_lane( l, m, verbose );
      if(verbose) {
        cout << "eval_lane: lane = " << l << " coll_time " << min_coll_time << endl;
      }
      if( min_coll_time > max_min_collision_time ) {
        best_lane = l;
        max_min_collision_time = min_coll_time; 
      }
  }

  return std::make_pair( best_lane, max_min_collision_time );
}

// return pair( t, v) where 
// t : the time to collision with the car which is ahead and closest 
// in the same lane, and
// v : the speed of said car.
std::pair<double, double> info_car_in_front( const MsgInfo& m, bool verbose ) {
  int cur_lane = getLane( m.car_fr );
  
  double min_collision_time = inf; 
  double my_s = m.car_fr.s; 
  double other_speed = NaN;
  
  for( auto other_car : m.other_cars ) {
    
    double other_s = other_car.fr.s; 
    if( getLane( other_car.fr ) == cur_lane && other_s >= my_s ){
      double collision_time = (other_s - my_s) / (m.car_speed - other_car.speed );

      if( collision_time >= 0 && collision_time < min_collision_time
          && collision_time < 3 * MIN_COLLISION_TIME ) {

        min_collision_time = collision_time;
        other_speed = other_car.speed;

      }
    }

  }
  return std::make_pair( min_collision_time, other_speed );
}

vector<CarInfo> get_other_cars( const json& sensor_fusion ){
// Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.  
  vector<CarInfo> other_cars;
  
  int num_other_cars = sensor_fusion.size();
  for( int i=0; i< num_other_cars; i++ ) {
    auto c = sensor_fusion[i];
    int cid = c[0];
    double x=c[1], y=c[2], vx=c[3], vy=c[4], s=c[5], d=c[6];
    other_cars.push_back( CarInfo(cid, x, y, vx, vy, s, d )); 
  }

  return other_cars;
}

MsgInfo extra_info_from_msg( const json& j) {

  MsgInfo m;      
  // Main car's localization Data
  m.car_p  = Point2D( j[1]["x"], j[1]["y"] );
  m.car_fr = PointFr( j[1]["s"], j[1]["d"] );    
  m.car_speed = mph2mps( j[1]["speed"] );
  
  // Previous path data given to the Planner
  m.previous_path_x = j[1]["previous_path_x"];
  m.previous_path_y = j[1]["previous_path_y"];
  // Previous path's end s and d values 
  // double end_path_s = j[1]["end_path_s"];
  // double end_path_d = j[1]["end_path_d"];

  m.other_cars = get_other_cars( j[1]["sensor_fusion"] );
  return m;
};

#endif  // HELPERS_H