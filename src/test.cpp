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


int main() {
 
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

  auto trajectory_fr = accelerate_to( PointFr{0.0, 0.0}, 0, 
                                      /* v0 = */ 0.0, /* v1 = */ MAX_V, 
                                      /*mean_a*/ 2.2 /* m/s^2 */  );

  cout << "\n\n\n Examining trajectory frenet" << endl;
  bool success = examine_trajectory( trajectory_fr, true );
  cout << "success fr: " << success << endl;
  
  cout << "\n\n\n Examining trajectory 2D" << endl;
  auto trajectory_2d = map.toXY( trajectory_fr );  
  success = examine_trajectory( trajectory_2d, true );
  cout << "success 2d: " << success << endl;
}