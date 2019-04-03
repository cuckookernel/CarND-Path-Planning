#include "helpers.h"


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
 
// trace a path (in Frenet coordinata) that starts at point `start`,
// with velocity v0 and accelerates to v1 
// with a mean acceleration given by mean_a.
// it does this by first constructing a trapezoidal acceleration schedule for the s-component of the movement.
TrajectoryFr accelerate_to( const PointFr& p0, const PointFr& v0, const PointFr& a0,
                            double tgt_d, double tgt_speed, double tgt_tm, 
                            int sub_sample ) {
  
  QTrajGen tgen_s, tgen_d;  
  // auto tgen = calibrate_traj_generator(p0, v0, a0, tgt_speed, tgt_tm);
  std::tie(tgen_s, tgen_d) = calibrate_traj_generator_5(p0, v0, a0, tgt_d, tgt_speed, tgt_tm);
  double t1 = tgen_s.t1();

  cout << "accelerate_to: v0: " << v0 << "  a0: " << a0 
       << "tgt_speed: "<< tgt_speed   << "  t1: " << t1 << endl;  
      
  int n_points   = (int) ceil( t1 / DT );
  int n_returned = (n_points / sub_sample) + 1;

  TrajectoryFr ret; 
  ret.reserve( n_returned );

  for( int i=0; i < n_points; i++ ){    
    double t = i * DT; 
    
    if ( i % sub_sample == 0 || (t1 - t) < DT ) {
      double s = tgen_s( t );
      double d = tgen_d( t );    
      // double d = d0 + (target_d - d0) * sigmoid( t - t1/2, MAX_V_D );
       ret.append( s, d, t);        
    }    
  }

  cout << "accelerate_to: returning " << ret.size() << " points" << endl;  
  return ret;
}


Trajectory2D decide_on_action_v0( const MsgInfo& m, const MapUtil& map, const Trajectory2D& last_trajectory, 
                               State& state) {
    // j[1] is the data JSON object
    // if( verbose ) {
    // cout << "\n\n\n msg " << msg_cnt << "\n"<< j.dump( ) << "\n\n" << endl;
    // }
    int sub_sample = 36;   
    PointFr v0_fr{NaN, NaN};
    PointFr a0_fr{NaN, NaN}; 
    
    double a0_norm = NaN;    
    int cnt_consumed = last_trajectory.size() - m.previous_path_x.size();

    // cout << "|last_traj| = " << last_trajectory.size()  << " cnt_consumed= " << cnt_consumed << endl;
        
    if( last_trajectory.size() < 3 || cnt_consumed < 3 )  {       
       v0_fr = PointFr{0., 0.};
       a0_fr = PointFr{0., 0.};
    } else { 
       // calculate velocity and acceleration at the end of consumed trajectory
       auto p2 = last_trajectory.as_point(cnt_consumed); // first not consumed, coincides with m.previous_path[0]
       auto p1 = last_trajectory.as_point(cnt_consumed - 1);
       auto p0 = last_trajectory.as_point(cnt_consumed - 2);
       auto v0_xy = (p2 - p1) / (DT);
       auto a0_xy = (((p2 - p1) - (p1 - p0)) / (DT * DT));
       
       Point2D n_s, n_d;
       std::tie(n_s, n_d) = map.localFrAxes( m.car_fr );
       v0_fr = project( v0_xy, n_s, n_d );
       a0_fr = project( a0_xy, n_s, n_d );
             
       cout << "cnt_consumed= "  << cnt_consumed 
          << "  p0=" << p0  << "  p1=" << p1 << " p2=" <<  p2 // << "p3: " << p3
          << "  v01: " << ((p1 - p0) / DT).norm() << "  v12: " << ((p2 -p1)/ DT).norm()
          << "\nprevious_path[0]: " << Point2D(m.previous_path_x[0], m.previous_path_y[1]) 
          << "  a0=" << a0_fr
          <<  endl;
    }
    
    cout << "\n\ncar(x: "<< m.car_p.x << " y: " << m.car_p.y << " s: " << m.car_fr.s << " d: " 
          << m.car_fr.d << ") car_speed: " << m.car_speed << " m/s "<< endl;

    const auto lane_change = evaluate_lane_change( m, false );
    const auto ln_change_coll_time = lane_change.second; 

    if( ln_change_coll_time > MIN_CHANGE_LANE_TIME ){
      int cur_lane = which_lane( m.car_fr );
      cout << "cur_lane: " << cur_lane  << "  best lane for change: " << lane_change.first 
          << " (collision time " << lane_change.second << ")" << endl;  
    
      state.lane =  lane_change.first;  
      // return accelerate_to( m.car_fr, /* v0 = */ v0_fr, /* a0 = */ a0_fr,
      //                      /* tgt_d */ target_d, /* tgt_speed = */ MAX_V, /*target_tm*/ 5, 
      //                      /*subsample*/ sub_sample ).toXY( map ).interpolate( DT ) ;
      return gen_trajectory_aaron_style( m, map, state);

    } else { //  stay in lane down
      double stay_coll_time; // time after which we will collide with car in front 
      double c_in_front_speed; // speed of car in front
      tie( stay_coll_time, c_in_front_speed ) = info_car_in_front( m, false );      
      auto tgt_speed = ( !isnan( c_in_front_speed ) ? c_in_front_speed : MAX_V ) ;
      auto tgt_tm    = ( !isnan( c_in_front_speed ) ? stay_coll_time * 0.75 : 5.0 ) ;

      cout << "Should stay in lane and slow down: car in front speed:  "<< tgt_speed 
            << " coll_time " << stay_coll_time << " <<<<<<<<<<!!!!!! " << endl; 
      
      // return  accelerate_to( m.car_fr, /* v0 = */ v0_fr,  /* a0 = */ a0_fr,
      //                       /*target_d */ lane_center(which_lane( m.car_fr )),      
      //                       /* tgt_speed = */ tgt_speed, /*target_tm*/ tgt_tm,
      //                       /*subsample*/ sub_sample ).toXY( map ).interpolate( DT );

      state.ref_speed = c_in_front_speed; 
      return gen_trajectory_aaron_style( m, map, state);
    }       
}
