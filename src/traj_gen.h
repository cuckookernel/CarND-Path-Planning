#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include <numeric>
#include <vector>
#include <algorithm>

#include "Eigen-3.3/Eigen/Dense"
#include "constants.h"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

double lane_center( int lane_num );
class PointFr;
int  which_lane( const PointFr& );

class QuarticTrajectoryGen  {

    double _s0 = NaN;
    double _v0 = NaN; 
    double _a0 = NaN;
    double _s3 = NaN;
    double _s4 = NaN;
    double _t1 = NaN;

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
    QuarticTrajectoryGen() : _s0(NaN), _v0(NaN), _a0(NaN), _s3(NaN), _s4(NaN), _t1(NaN) {};
   
    QuarticTrajectoryGen( double s0_, double v0_, double a0_, double t1_ ) : 
      _s0( s0_ ), _v0( v0_ ), _a0( a0_ ), _t1( t1_ )  {}

    inline void solve( double v1, double a1, double t ) {
        _t1 = t;
        _s3 = 6 * ( (v1 - _v0) - (2.0/3.0) * _a0 * _t1 - a1 * _t1 / 3.0 ) / (_t1*_t1);
        _s4 = 2 * ( a1 - _a0 - _s3 * _t1 ) / (_t1 * _t1);
    }

    inline double max_jerk(  ) const {
      return std::max(fabs(_s3) , fabs(_s3 + _s4 * _t1) );
    }

    inline double t1() const {
      return _t1; 
    }

    inline double operator()( double t ) const {
      // return _s0 + _v0 * t + _a0 * (t*t) / 2.0 + _s3 * (t*t*t) / 6.0 + _s4 * ( t*t*t*t) / 24.0;
      // save about 40% of flops:
      return ((( _s4 / 24.0 * t + _s3 / 6.0 ) * t + _a0 / 2.0 ) * t + _v0 ) * t + _s0;
    }
};


class QTrajGen  {

  vector<double> coefs; 
  double _t1;

  public: 

  QTrajGen() = default;
  
  QTrajGen( double s0, double v0, double a0) {
    coefs.resize(6);
    coefs[0] = s0; 
    coefs[1] = v0; 
    coefs[2] = 0.5 * a0;
  }

  void solve( double s1, double v1, double a1, double T1) {
    MatrixXd M(3,3); 
    VectorXd sf(3), rhs(3);
    // VectorXd si(3), sf(3), b(3);
    // si << start[0], start[1], start[2];
    
    sf << s1, v1, a1;
    double T2 = T1 * T1; 
    double T3 = T2 * T1; 
    double T4 = T2 * T2; 
    double T5 = T2 * T3;
  
    rhs << (sf[0] - (coefs[0] + coefs[1] * T1 + coefs[2] * T2) ),
         (sf[1] - (coefs[1] + 2 * coefs[2] * T1 )), 
         (sf[2] - 2 * coefs[2]);
    M <<     T3,       T4,      T5,
         3*  T2,   4 * T3,  5 * T4,
         6 * T1,  12 * T2, 20 * T3;

    VectorXd alpha = M.colPivHouseholderQr().solve( rhs );
    coefs[3] = alpha[0];
    coefs[4] = alpha[1];
    coefs[5] = alpha[2];
    _t1 = T1;
  }

  void solve_no_pos( double v1, double a1, double T1) {
    MatrixXd M(3,3); 
    VectorXd rhs(3);
    // VectorXd si(3), sf(3), b(3);
    // si << start[0], start[1], start[2];
        
    double T2 = T1 * T1; 
    double T3 = T2 * T1; 
    double T4 = T2 * T2; 
    
    rhs << (v1 - (coefs[1] + 2 * coefs[2] * T1 )), 
           (a1 - 2 * coefs[2]), 
           0.0;  // solve for 0 jerk at time T1 
    M << 3 * T2,  4 * T3,  5 * T4,
         6 * T1, 12 * T2, 20 * T3,
         6     , 24 * T1, 60 * T2; 

    VectorXd alpha = M.colPivHouseholderQr().solve( rhs );
    coefs[3] = alpha[0];
    coefs[4] = alpha[1];
    coefs[5] = alpha[2];
    _t1 = T1;
  }


  // compute position at time t
  inline double operator()( double t ) const {
    return (((((coefs[5] * t + coefs[4]) * t + coefs[3] ) * t + coefs[2]) * t + coefs[1]) * t  + coefs[0]);
  } 

  // compute velocity at time t, only used for testing purposes
  inline double vel( double t ) const {
    return ((((5 * coefs[5] * t + 4* coefs[4]) * t + 3* coefs[3] ) * t + 2 * coefs[2]) * t + coefs[1]);
  } 

  // compute acceleration time t, only used for testing purposes
  inline double accel( double t ) const {
    return (((20 * coefs[5] * t + 12 * coefs[4]) * t + 6 * coefs[3] ) * t + 2 * coefs[2]);
  }    

  // compute absolute value of jerk at time t
  inline double abs_jerk( double t ) const {
    // (60 = 5 * 4 * 3) c5 t^2 + (24 = 4 * 3 * 2) c4 t + c3 
    return fabs( (60 * coefs[5] * t  + 24 * coefs[4]) * t + 6 * coefs[3] );    
  }

  inline double t1() const {
    return _t1;
  }

  // compute maximum of jerk over [0, T]
  inline double max_jerk( ) const {
    int t_star = ( fabs(coefs[5]) > 1e-8 ?  -24 * coefs[4] / ( 120 * coefs[5] ) : NaN );

    if( t_star >= 0  && t_star < _t1 ) {
      return std::max( { abs_jerk(0), abs_jerk(_t1), abs_jerk(t_star) } );
    } else {
      return std::max( abs_jerk(0), abs_jerk(_t1));
    }    
  } 
 
};

std::pair<QTrajGen, QTrajGen> 
calibrate_traj_generator_5( const PointFr& p0, const PointFr& v0, const PointFr& a0,
                          double tgt_d, double tgt_speed, double  tgt_tm ) {

    double t1 = tgt_tm; 

    QTrajGen tgen_s( p0.s, v0.s, a0.s );    
    QTrajGen tgen_d( p0.d, v0.d, a0.d );    

    do {
        tgen_s.solve_no_pos( tgt_speed, 0, t1 );
        tgen_d.solve( tgt_d, 0., 0., t1 );
        t1 *= 1.3;
    } while( fabs(tgen_s.max_jerk()) + fabs( tgen_d.max_jerk() ) >= MAX_JERK );

    return std::make_pair( tgen_s, tgen_d );
}

Trajectory2D draft_trajectory_local_aaron( const MsgInfo& m, const MapUtil& map, State& st, bool is_lane_change ) {
    vector<Point2D> pts_g; //  (x,y)-coordinates in the global xy coordinate system
    
    Point2D ref_p = m.car_p; 
    
    // First two points 
    if( m.prev_size < 2 ) {
        Point2D prev_car_p = m.car_p - unit( m.car_yaw ); 
        st.ref_yaw = 0.;        

        pts_g.push_back( prev_car_p );
        pts_g.push_back( m.car_p );
    } else {
        // redefined ref_p
        ref_p = Point2D( m.previous_path_x[m.prev_size-1],  m.previous_path_y[m.prev_size-1]) ;
        Point2D prev_car_p( m.previous_path_x[m.prev_size-2],  m.previous_path_y[m.prev_size-2]);
        
        st.ref_yaw = atan2( ref_p, prev_car_p );

        pts_g.push_back( prev_car_p );
        pts_g.push_back( ref_p );
    }

    // three more points way ahead...
    double ln_center = lane_center( st.lane );

    double ref_distance = ( is_lane_change ? LANE_CHANGE_DISTANCE : AARONS_DISTANCE );

    pts_g.push_back( map.toXY( m.car_fr.s +     ref_distance, ln_center ) );
    pts_g.push_back( map.toXY( m.car_fr.s + 2 * ref_distance, ln_center ) );
    pts_g.push_back( map.toXY( m.car_fr.s + 3 * ref_distance, ln_center ) );

    vector<Point2D> pts_o; // pts with (x,y) in the car's own "local" coordinate system
    for( auto p_g : pts_g ) {
        // shift and rotate Teo's style        
        pts_o.push_back( (p_g - ref_p).rotate( -st.ref_yaw ) );
    } 

    // convert to a trajectory that is convenient for doing spline
    Trajectory2D ret; 
    ret.extend( pts_o, 0.0 );

    return ret;   
}

Trajectory2D gen_trajectory_aaron_style( const MsgInfo& m, const MapUtil& map, State& st) {
    
    Trajectory2D next_pts; // to be returned 
    next_pts.reserve( AARONS_N_POINTS );

    for( int i=0; i < m.prev_size; ++i ) {
        next_pts.append( m.previous_path_x[i], m.previous_path_y[i], 0.0 );
    }

    const bool is_lane_change = (which_lane( m.car_fr ) != st.lane)  ;

    Trajectory2D traj_lcl = draft_trajectory_local_aaron( m, map, st, is_lane_change );
    tk::spline s;
    s.set_points( traj_lcl.xs(), traj_lcl.ys() );

    double tgt_x = ( is_lane_change ? LANE_CHANGE_DISTANCE : AARONS_DISTANCE );
    double tgt_y = s( tgt_x );
    double tgt_dist = Point2D( tgt_x, tgt_y ).norm(); 


    Point2D ref_p;  // recompute just this in the same way we di in draft_trajectory_local_aaron to avoid passing it around
    if( m.prev_size < 2 ) {
        ref_p = m.car_p;
    } else {
        ref_p = Point2D( m.previous_path_x[m.prev_size-1],  m.previous_path_y[m.prev_size-1]);
    }

    double cum_x  = 0.;
    for( int i=0; i < AARONS_N_POINTS - m.prev_size; ++i ) {
        double N = tgt_dist / ( DT * st.ref_speed );
        cum_x += tgt_x / N;
        Point2D next_p( cum_x, s(cum_x) );

        next_pts.append( next_p.rotate( st.ref_yaw ) + ref_p, 0 );
    }

    return next_pts;
}



#endif