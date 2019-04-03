
#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <numeric>
#include <cmath>

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double inf = std::numeric_limits<double>::infinity();

constexpr double pi() { return M_PI; }

constexpr double MAX_JERK = 10.0; // m/s^3
constexpr double MAX_ACCEL = 10.0; // m/s^2
constexpr double DT = 0.02; // s
constexpr double MIN_CHANGE_LANE_TIME = 10.0; // s
constexpr double MIN_COLLISION_TIME = 5.0; // s
constexpr double MAX_SPEED_D = 4.0; // m/s
constexpr double MAX_SPEED = 45.0 * 1609.34 / 3600; // m/s ( 45 mph = aprox 22 m/s ) 
constexpr double LANE_WIDTH = 4.0; 
constexpr double AARONS_DISTANCE = 30;  // 30m, magic number thanks to Aaron...
constexpr double LANE_CHANGE_DISTANCE = 60; // to avoid max jerk / acceleration when changin lanes...

constexpr int    AARONS_N_POINTS = 50;
constexpr double AARONS_ACCEL = .150;

#endif 
