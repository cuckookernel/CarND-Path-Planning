#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
// #include <numeric>
#include <iomanip>
#include <algorithm>
#include <ostream>

#include "spline.h"

#include "constants.h"
#include "common_types.h"
#include "traj_gen.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

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

inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mph2mps( double v_mph ) {  return v_mph * 1609 / 3600; }
inline double mps2mph( double v_mps ) {  return v_mps * 3600 / 1609; }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}

inline double sigmoid( double t, double beta ) {
   
   return  1.0 / ( 1.0 + exp(- beta * t) );   
}

string format_clock_time( double clock_ ) {
  std::stringstream s; 
  int mins = (int) floor( clock_ / 60.0 );
  double secs = clock_ - 60 * mins;
  
  s << std::setfill('0') << std::setw(2) << mins << ":" << std::setprecision(5) << std::setw(5) << secs;
  return s.str();
}

template<typename Fn> 
void assert_fn( bool expr, Fn fn) {
  if (!expr) {
    fn();
    abort();
  }
}

inline int which_lane( const PointFr& f ) {
  return (int) floor( f.d  / LANE_WIDTH );
} 

inline double lane_center( int lane_num ) {
  return (lane_num + 0.5) * LANE_WIDTH;
}

inline bool are_valid_accel_jerk( double a1, double T ) {
    return fabs(a1) < 0.8 * MAX_ACCEL && (3 * fabs( a1 ) / T) < 0.8 * MAX_JERK;
}

void load_map( string& map_file, MapUtil& map ) {
  std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

  string line;
  //Trajectory2D last_trajectory; 
  

  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x, y;  
    float s, d_x, d_y;
    
    iss >> x >> y >> s >> d_x >> d_y;
    map.xys.push_back(Point2D(x, y));
    map.ss.push_back(s);
    map.dxys.push_back(Point2D(d_x, d_y));    
  }
}

vector<OtherCarInfo> get_other_cars( const json& sensor_fusion ){
// Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.  
  vector<OtherCarInfo> other_cars;
  
  int num_other_cars = sensor_fusion.size();
  for( int i=0; i< num_other_cars; i++ ) {
    auto c = sensor_fusion[i];
    int cid = c[0];
    double x=c[1], y=c[2], vx=c[3], vy=c[4], s=c[5], d=c[6];
    other_cars.push_back( OtherCarInfo(cid, x, y, vx, vy, s, d )); 
  }

  return other_cars;
}

MsgInfo extra_info_from_msg( const json& j) {

  MsgInfo m;      
  // Main car's localization Data
  m.car_p  = Point2D( j[1]["x"], j[1]["y"] );
  m.car_fr = PointFr( j[1]["s"], j[1]["d"] );    
  m.car_speed = mph2mps( j[1]["speed"] );
  m.car_yaw   = deg2rad( j[1]["yaw"] ) ;
    
  // Previous path data given to the Planner
  m.previous_path_x = j[1]["previous_path_x"];
  m.previous_path_y = j[1]["previous_path_y"];
  m.prev_size = m.previous_path_x.size();
  
  // Previous path's end s and d values 
  m.end_path_s = j[1]["end_path_s"];
  m.end_path_d = j[1]["end_path_d"];

  m.other_cars = get_other_cars( j[1]["sensor_fusion"] );

  return m;
};


double min_collision_time_for_lane( int lane_num, const MsgInfo& m, bool verbose ) {
  
  double my_s = m.car_fr.s;
  double min_time = inf;

  for( auto other_car : m.other_cars ){
    // cout << " other_car( id= "<< other_car.id << ", d:  " << other_car.fr.d << " )" << endl; 
    if ( which_lane( other_car.fr ) == lane_num  ) {
      
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

// return a new reference speed to match a car in fron 
bool maybe_break_because_car_in_front( int lane_num, const MsgInfo& m, const State &state) {
  double my_future_s = m.end_path_s;

  for( auto other_car : m.other_cars ) {
      
      if( which_lane( other_car.fr ) == lane_num ){        
        double other_future_s = other_car.fr.s + m.prev_size * DT * other_car.speed; 

        if (other_future_s > my_future_s && other_future_s - my_future_s < AARONS_DISTANCE ) {
          cout << "\n *** Breaking because car (id="<< other_car.id << ") in front: dist " 
               <<  (other_future_s - my_future_s) << " m, speed: " << mps2mph(other_car.speed) << endl;
          return true;
        }        
      }  
  }

  return false; 
}

std::pair<int, double> evaluate_lane_change_v0( const MsgInfo& m, bool verbose ) {
  int cur_lane = which_lane( m.car_fr );
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

double lane_cost( int lane_num, const MsgInfo& m ) {
  double my_future_s = m.end_path_s;

  // cost will be maximum of 1/speed(c)  where c ranges over cars in this lane that are 
  // in front of us and v_c is the car speed or inf if there is car in this lane likely 
  // to collide with us (given that we changed instantly)

  double cost =  -inf; 
  for( auto other_car : m.other_cars ) {
      
      if( which_lane( other_car.fr ) == lane_num ){        
        double other_future_s = other_car.fr.s + m.prev_size * DT * other_car.speed; 

        if ( other_future_s > my_future_s) { // applies to other cars that are (will be) in front 
          if  (other_future_s - my_future_s < LANE_CHANGE_DISTANCE ) {          
            return inf; // possible collision
          } else {
            cost = std::max( cost, 1 / other_car.speed );
          }
        } 
        else { //  ( other_future_s <= my_future_s ) // cars that will be behind 
         if (  my_future_s - other_future_s < 0.33 * LANE_CHANGE_DISTANCE ) {
           return inf; // possible collision
         }
        }
      }  
  }
  return cost;

}

int evaluate_lane_change_v1( const MsgInfo& m, const State& state ) {

  int cur_lane = which_lane( m.car_fr );
  if( state.clock - state.clock_last_lane_change < MIN_CHANGE_LANE_TIME ) {
    return cur_lane;
  }
  
  vector<int> lanes_to_consider;
  if( cur_lane == 0 ) {        
    lanes_to_consider = {0, 1};
  } else if( cur_lane == 1) {
    lanes_to_consider = {1, 0 ,2};
  } else if( cur_lane == 2) {
    lanes_to_consider = {2, 1};
  }

  int best_lane = cur_lane; 
  double min_cost = +inf;
  for( auto l : lanes_to_consider ) {
      double ln_cost = lane_cost( l, m );
      
      if( ln_cost < min_cost  ) {
        best_lane = l;
      }
  }

  return best_lane;
}


// return pair( t, v) where 
// t : the time to collision with the car which is ahead and closest 
// in the same lane, and
// v : the speed of said car.
std::pair<double, double> info_car_in_front( const MsgInfo& m, bool verbose ) {
  int cur_lane = which_lane( m.car_fr );
  
  double min_collision_time = inf; 
  double my_s = m.car_fr.s; 
  double other_speed = NaN;
  
  for( auto other_car : m.other_cars ) {
    
    double other_s = other_car.fr.s; 
    if( which_lane( other_car.fr ) == cur_lane && other_s >= my_s ){
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


#endif  // HELPERS_H