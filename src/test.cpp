#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

// Check a trajectory and see where it violates speed, acceleration and jerk constraints  
template<typename TT, typename PT>
bool examine_trajectory( const TT& traj, bool verbose = false ) {
  int n = traj.size();

  PT prev_p{ traj[0].first, traj[0].second }; 
  PT prev_a{0,0}; 
  PT prev_v{0,0};
  double t_prev = 0.0;
  bool success = true; 
  
  int n_points = traj.size();
  for( int i=0; i < n_points; i++ ) {
    auto p  = traj.at(i);
    PT p2d( std::get<0>(p), std::get<1>(p) );
    double t = std::get<2>( p );
    double dt = t - t_prev;
        
    auto v    = (i > 1 ? (p2d - prev_p) / dt : PT{0,0} );
    auto v_unit = v / v.norm();
    auto a    = (i > 2 ? (v - prev_v) / dt : PT{0,0} );
    double theta = atan2( v );
    double a_n   = (i > 2 ? a.norm() * cos(theta) : 0.0  );
    double a_d   = (i > 2 ? a.norm() * sin(theta) : 0.0  );
        
    auto jerk = (i > 3 ? (a - prev_a) / dt : PT{0,0} );

    if( (v.norm() > MAX_SPEED) || ( a.norm() > MAX_ACCEL ) || (jerk.norm() > MAX_JERK) ) {
      success = false;
    }
    
    if( verbose ) {
      cout << "i=" << i << ": " << std::fixed << std::setprecision(1) << p2d.fst() <<", "<< p2d.snd()
           <<  "  t = " << t 
           << "  |v| = " << v.norm() 
           << "  |a| = " << a.norm()  << " a_n= " << a_n  << " a_d = " << a_d 
           << "  |j| = " << jerk.norm() 
           << ( v.norm() > MAX_SPEED ? " max speed violation!" : "")  
           << ( a.norm() > 10    ? " max acceleration violation!" : "") 
           << ( jerk.norm() > 10 ? " max jerk violation! " : "") << endl;           
    }

    prev_p = p2d; 
    prev_v = v; 
    prev_a = a;
    t_prev = t;
  }

  return success; 
}


int main() {

  QTrajGen qtg(1, 5, 17);

  double T = 2; 

  qtg.solve( 10, 7, 23, T);

  cout << "t=" << 0 <<  " p=" << qtg(0) << "  v=" << qtg.vel(0) << "  a=" << qtg.accel(0) << endl;
  cout << "t=" << T <<  " p=" << qtg(T) << "  v=" << qtg.vel(T) << "  a=" << qtg.accel(T) << endl;


  qtg = QTrajGen(2, 4, 13);
  
  qtg.solve( 15, 6, 28, T);

  cout << "t=" << 0 <<  " p=" << qtg(0) << "  v=" << qtg.vel(0) << "  a=" << qtg.accel(0) << endl;
  cout << "t=" << T <<  " p=" << qtg(T) << "  v=" << qtg.vel(T) << "  a=" << qtg.accel(T) << endl;
}

int main1() {
 
  MapUtil map; 

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x, y;  
    float s, d_x, d_y;
    
    iss >> x >> y >> s >> d_x >> d_y;
    map.xys.push_back(Point2D(x, y));
    map.ss.push_back(s);
    map.dxys.push_back(Point2D(d_x, d_y));    
  }

  /* 
  auto trajectory_fr = accelerate_to( PointFr{0.0, 0.0}, 
                                      /* v0 = * / PointFr{0.0, 0.0}, 
                                      /* a0 = * / PointFr{0.0, 0.0}, 
                                      /* tgt_d * / 0,                                     
                                      /* tgt_speed = * / MAX_V,                                       
                                      /* tgt_tm* / 10 /* m/s^2 * /,
                                      /* subsample * / 9  );

  cout << "\n\n\n Examining trajectory frenet" << endl;
  bool success = examine_trajectory<TrajectoryFr, PointFr>( trajectory_fr, true );
  cout << "success fr: " << success << endl;
  
  cout << "\n\n\n Examining trajectory 2D" << endl;
  auto trajectory_2d = trajectory_fr.toXY( map ).interpolate( DT );  
  success = examine_trajectory<Trajectory2D, Point2D>( trajectory_2d, true );
  cout << "success 2d: " << success << endl;
  */ 
}