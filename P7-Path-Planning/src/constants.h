#ifndef CONSTANTS_H_
#define CONSTANTS_H_

const int MIN_LANE = 0;
const int MAX_LANE = 2;
const int LANE_WIDTH = 4;

const double LOOK_AHEAD = 30;
const double LOOK_BEHIND = 30;
const double DT = 0.02;
const double VEL_INCREMENT = 0.224;

const int WP0 = 50;
const int WP1 = 70;
const int WP2 = 90;

const double SPLINE_TARGET = 30;
const int TRAJ_SIZE = 50;

const double MAX_VELOCITY = 49.5;
const double MPS_TO_MPH = 2.24;

#endif // CONSTANTS_H_