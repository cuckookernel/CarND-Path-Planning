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

Trajectory2D decide_on_action( MsgInfo& m, const MapUtil& map, const Trajectory2D& last_trajectory ) {
    // j[1] is the data JSON object
    // if( verbose ) {
    // cout << "\n\n\n msg " << msg_cnt << "\n"<< j.dump( ) << "\n\n" << endl;
    // }
    int sub_sample = 36;   
    double a0 = NaN;
    int cnt_consumed = last_trajectory.size() - m.previous_path_x.size();

    // cout << "|last_traj| = " << last_trajectory.size()  << " cnt_consumed= " << cnt_consumed << endl;
    
    if( last_trajectory.size() < 3 || cnt_consumed < 3 )  {
       a0 = 0.0;
    } else { // estimate acceleration at the end of consumed trajectory
       auto p2 = last_trajectory.as_point(cnt_consumed); // first not consumed, coincides with m.previous_path[0]
       auto p1 = last_trajectory.as_point(cnt_consumed - 1);
       auto p0 = last_trajectory.as_point(cnt_consumed - 2);
       a0 = (((p2 - p1) - (p1 - p0)) / (DT * DT)).norm(); 
       
       cout << "cnt_consumed= "  << cnt_consumed 
          << "  p0=" << p0  << "  p1=" << p1 << " p2=" <<  p2 // << "p3: " << p3
          << "  v01=" << ((p1 - p0) / DT).norm() << "  v12: " << ((p2 -p1)/ DT).norm()
          << "\nprevious_path[0]: " << Point2D(m.previous_path_x[0],m.previous_path_y[1]) 
          << "  a0=" << a0  
          <<  endl;
    }
    
    cout << "\n\ncar(x: "<< m.car_p.x << " y: " << m.car_p.y << " s: " << m.car_fr.s << " d: " 
          << m.car_fr.d << ") car_speed: " << m.car_speed << " m/s "<< endl;

    const auto lane_change = evaluate_lane_change( m, false );
    const auto ln_change_coll_time = lane_change.second; 

    if( ln_change_coll_time > MIN_CHANGE_LANE_TIME ){
      int cur_lane = getLane( m.car_fr );
      cout << "cur_lane: " << cur_lane  << "  best lane for change: " << lane_change.first 
          << " (collision time " << lane_change.second << ")" << endl;  
    
      double target_d = ( lane_change.first + 0.5 ) * LANE_WIDTH;
    
      return accelerate_to( m.car_fr, 
                                    /*target_d */ target_d,
                                    /* v0 = */ m.car_speed, 
                                    /* v1 = */ MAX_V, 
                                    /* a0 = */ a0, 
                                    /*target_tm*/ 5, 
                                    /*subsample*/ sub_sample ).toXY( map ).interpolate( DT ) ;
    } else { //  stay in lane down
      auto car_in_front = info_car_in_front( m, false );
      auto stay_coll_time = car_in_front.first;
      auto target_speed = ( !isnan( car_in_front.second ) ? car_in_front.second : MAX_V ) ;
      auto target_tm    = ( !isnan( car_in_front.second ) ? stay_coll_time * 0.75 : 5.0 ) ;

      cout << "Should stay in lane and slow down: car in front speed:  "<< target_speed 
            << " coll_time " << stay_coll_time << " <<<<<<<<<<!!!!!! " << endl; 
      
      return  accelerate_to( m.car_fr, 
                              /*target_d */ (getLane( m.car_fr ) + 0.5) * LANE_WIDTH,
                              /* v0 = */ m.car_speed, 
                              /* v1 = */ target_speed, 
                              /* a0 = */ a0,
                              /*target_tm*/ target_tm,
                              /*subsambple*/ sub_sample ).toXY( map ).interpolate( DT );
    }       
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

  Trajectory2D last_trajectory; 

  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x, y;  
    float s, d_x, d_y;
    
    iss >> x >> y >> s >> d_x >> d_y;
    map.xys.push_back(Point2D(x, y));
    map.ss.push_back(s);
    map.dxys.push_back(Point2D(d_x, d_y));    
  }

  h.onMessage([&map, &msg_cnt, &last_trajectory](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          json msgJson;

          if( m.previous_path_x.size() < 100 ){
            Trajectory2D traj = decide_on_action( m, map, last_trajectory );
            msgJson["next_x"] = traj.xs();
            msgJson["next_y"] = traj.ys();
            
            last_trajectory = traj;
            msg_cnt ++;
          } else {
            msgJson["next_x"] = m.previous_path_x;
            msgJson["next_y"] = m.previous_path_y;            
          }

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