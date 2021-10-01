#include "behavior_planner.h"

vector<double> BehaviorPlanner::proposeTargets() {
       //////////////////////////////////
    // Preventing collitions.
    
    // Prediction : Analysing other cars positions.
    bool car_ahead = false;
    bool car_left = false;
    bool car_righ = false;
    for ( int i = 0; i < sensor_fusion.size(); i++ ) {
        float d = sensor_fusion[i][6];
        int car_lane = -1;
        // is it on the same lane we are
        if ( d > 0 && d < 4 ) {
            car_lane = 0;
        } else if ( d > 4 && d < 8 ) {
            car_lane = 1;
        } else if ( d > 8 && d < 12 ) {
            car_lane = 2;
        }
        if (car_lane < 0) {
            continue;
        }
        // Find car speed.
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];
        // Estimate car s position after executing previous trajectory.
        check_car_s += ((double)prev_size*0.02*check_speed);

        if ( car_lane == lane ) {
            // Car in our lane.
            car_ahead |= check_car_s > car_s && check_car_s - car_s < 30;
        } else if ( car_lane - lane == -1 ) {
            // Car left
            car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
        } else if ( car_lane - lane == 1 ) {
            // Car right
            car_righ |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
        }
    }

    // Behavior : Let's see what to do.
    double speed_diff = 0;
    const double MAX_SPEED = 49.5;
    const double MAX_ACC = .224;
    if ( car_ahead ) { // Car ahead
        if ( !car_left && lane > 0 ) {
        // if there is no car left and there is a left lane.
        lane--; // Change lane left.
        } else if ( !car_righ && lane != 2 ){
        // if there is no car right and there is a right lane.
        lane++; // Change lane right.
        } else {
        speed_diff -= MAX_ACC;
        }
    } else {
        if ( lane != 1 ) { // if we are not on the center lane.
        if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) {
            lane = 1; // Back to center.
        }
        }
        if ( ref_vel < MAX_SPEED ) {
        speed_diff += MAX_ACC;
        }
    }
    return {lane, ref_vel}
}