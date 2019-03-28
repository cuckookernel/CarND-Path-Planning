#include <uWS/uWS.h>
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
}

std::pair<int, double> evaluate_lane_change( MsgInfo& m, bool verbose ) {
  int cur_lane = getLane( m.car_fr );
  vector<int> lanes_to_consider;
  if( cur_lane == 0 ) {        
    lanes_to_consider = {0,1};
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

// return the time to collision with the next car which is ahead in the same 
// lane and the speed of said car.
std::pair<double, double> info_car_in_front( MsgInfo& m, bool verbose ) {
  int cur_lane = getLane( m.car_fr );
  
  double min_collision_time = inf; 
  double my_s = m.car_fr.s; 
  double other_speed = NaN;
  
  for( auto other_car : m.other_cars ) {
    double other_s = other_car.fr.s; 
    if( getLane( other_car.fr ) == cur_lane && other_s >= my_s ){
      double collision_time = (other_s - my_s) / (m.car_speed - other_car.speed );

      if( collision_time >= 0 && collision_time < min_collision_time && collision_time < 3 * MIN_COLLISION_TIME ) {
        min_collision_time = collision_time;
        other_speed = other_car.speed;
      }
    }
  }

  return std::make_pair( min_collision_time, other_speed );
}

json reply_to_telemetry( const json& j, const MapUtil& map,  int& msg_cnt, bool verbose ) {

  // j[1] is the data JSON object
    
    if( verbose ) {
      // cout << "\n\n\n msg " << msg_cnt << "\n"<< j.dump( ) << "\n\n" << endl;
    }
    
    MsgInfo m = extra_info_from_msg( j );

    json msgJson;
    vector<double> next_x_vals, next_y_vals;
   
    if(  m.previous_path_x.size() < 100 ){
      cout << "\n\ncar(x: "<< m.car_p.x << " y: " << m.car_p.y << " s: " << m.car_fr.s << " d: " 
           << m.car_fr.d << ") car_speed: " << m.car_speed << " m/s "<< endl;

      const auto lane_change = evaluate_lane_change( m, false );
      const auto ln_change_coll_time = lane_change.second; 

      if( ln_change_coll_time > MIN_CHANGE_LANE_TIME ){
        int cur_lane = getLane( m.car_fr );
        cout << " cur_lane " << cur_lane 
           << " lane_change best " << lane_change.first 
           << " collision time " << lane_change.second << endl;  
      
        double target_d = ( lane_change.first + 0.5 ) * LANE_WIDTH;
      
        auto trajectory_2d = map.toXY( accelerate_to( m.car_fr, 
                                                    /*target_d */ target_d,
                                                    /* v0 = */ m.car_speed, 
                                                    /* v1 = */ MAX_V, 
                                                    /*target_tm*/ 5 ));

        push_trajectory( trajectory_2d, next_x_vals, next_y_vals );
      } else { // 
        auto car_in_front = info_car_in_front( m, false );
        auto stay_coll_time = car_in_front.first;
        auto target_speed = ( !isnan( car_in_front.second ) ? car_in_front.second : MAX_V) ;
        auto target_tm    = ( !isnan( car_in_front.second ) ? stay_coll_time * 0.75 : 5.0 ) ;

        cout << " Should stay in lane and slow down: car in front speed:  "<< target_speed 
             << " coll_time " << stay_coll_time << endl; 
        
        auto trajectory_2d = map.toXY( accelerate_to( m.car_fr, 
                                                    /*target_d */ (getLane( m.car_fr ) + 0.5) * LANE_WIDTH,
                                                    /* v0 = */ m.car_speed, 
                                                    /* v1 = */ target_speed, 
                                                    /*target_tm*/ target_tm ));

        push_trajectory( trajectory_2d, next_x_vals, next_y_vals );
      }        
      
      msgJson["next_x"] = next_x_vals;
      msgJson["next_y"] = next_y_vals;

    } else {
      // cout << "previous_path has " << previous_path_x.size() << endl;
      msgJson["next_x"] = m.previous_path_x;
      msgJson["next_y"] = m.previous_path_y;            
    }

    msg_cnt ++;
    return msgJson;
}


int main() {
  uWS::Hub h;
 

  MapUtil map; 

  int msg_cnt = 0; 

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  // double max_s = 6945.554;

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

  h.onMessage([&map, &msg_cnt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          auto msgJson = reply_to_telemetry( j, map, msg_cnt, msg_cnt % 100 == 0);
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); 
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}

/* Not really needed.. 
void from_json_to_vec( vector<double>& dest, const json::value_type& json_vec ) {

    int n = json_vec.size(); 
    dest.reserve( n );
    for( int i =0; i < n; i++) {
      auto v = json_vec[i];
      dest.push_back( v); 
    } 
}*/ 