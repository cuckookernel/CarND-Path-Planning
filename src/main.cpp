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
using std::tie; // assignment from tuple!


Trajectory2D decide_on_action( const MsgInfo& m, const MapUtil& map, State& state) {
        
    int cur_lane = which_lane( m.car_fr );    
    int new_lane = evaluate_lane_change_v1( m, state ); 

    if( new_lane != cur_lane ) {
      cout << "decided to switch lane! to " << new_lane; 
      state.lane = new_lane;
      state.clock_last_lane_change = state.clock; 
    }
    else {  // decided to stay in lane
      bool want_to_break = maybe_break_because_car_in_front( which_lane( m.car_fr ), m , state );

      if( want_to_break && state.ref_speed > AARONS_ACCEL ) {
        state.ref_speed -= AARONS_ACCEL;       
      } else if(  state.ref_speed < MAX_SPEED ) {
        state.ref_speed += AARONS_ACCEL;
      }    
    }
    
    cout << format_clock_time( state.clock ) << " " 
         << std::setprecision(3) << " car_speed: " << mps2mph( m.car_speed ) 
         << " mph, ref_speed: "  << mps2mph( state.ref_speed ) << " mph "
         << " lane_change: "     << (new_lane != cur_lane) 
         << "  last consumed: "  << (AARONS_N_POINTS - m.prev_size) << endl;

    Trajectory2D ret = gen_trajectory_aaron_style( m, map, state );

    /* const auto lane_change = evaluate_lane_change( m, false );
    const auto ln_change_coll_time = lane_change.second; 

    if( ln_change_coll_time > MIN_CHANGE_LANE_TIME ){
      int cur_lane = which_lane( m.car_fr );
      cout << "cur_lane: " << cur_lane  << "  best lane for change: " << lane_change.first 
          << " (collision time " << lane_change.second << ")" << endl;  
    
      state.lane =  lane_change.first;  
      
      return gen_trajectory_aaron_style( m, map, state );

    } else { //  stay in lane down
      double stay_coll_time; // time after which we will collide with car in front 
      double c_in_front_speed; // speed of car in front
      tie( stay_coll_time, c_in_front_speed ) = info_car_in_front( m, false );      
      auto tgt_speed = ( !isnan( c_in_front_speed ) ? c_in_front_speed : MAX_V ) ;
      auto tgt_tm    = ( !isnan( c_in_front_speed ) ? stay_coll_time * 0.75 : 5.0 ) ;

      cout << "Should stay in lane and slow down: car in front speed:  "<< tgt_speed 
            << " coll_time " << stay_coll_time << " <<<<<<<<<<!!!!!! " << endl; 
      
      
      state.ref_speed = c_in_front_speed; 
      return gen_trajectory_aaron_style( m, map, state);
    }  */ 

    return ret; 

}


int main() {
  uWS::Hub h;
  
  int msg_cnt = 0; 
  // Waypoint map to read from
  string map_file = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  // double max_s = 6945.554;
  MapUtil map; 
  load_map( map_file, map );

  State state; 
  h.onMessage([&map, &msg_cnt, &state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          MsgInfo m = extra_info_from_msg( j );
          state.clock += DT * (AARONS_N_POINTS - m.prev_size);
          Trajectory2D traj = decide_on_action( m, map, state );
          json msgJson;
          msgJson["next_x"] = traj.xs();
          msgJson["next_y"] = traj.ys();
          
          /* old version 
          if( m.previous_path_x.size() < 100 ){
            Trajectory2D traj = decide_on_action( m, map, last_trajectory, state);
            msgJson["next_x"] = traj.xs();
            msgJson["next_y"] = traj.ys();
            
            last_trajectory = traj;
            msg_cnt ++;
          } else {
            msgJson["next_x"] = m.previous_path_x;
            msgJson["next_y"] = m.previous_path_y;            
          }
          */ 

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