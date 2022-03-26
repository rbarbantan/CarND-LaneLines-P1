#include <cstdio>
#include "behavior_planner.h"

void BehaviorPlanner::updateTraffic(vector<Vehicle> traffic) {
    this->traffic = traffic;
}

void BehaviorPlanner::updateEgo(Vehicle ego, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s) {
    this->ego = ego;
    this->prev_size = previous_path_x.size();
    this->end_path_s = end_path_s;
}

vector<double> BehaviorPlanner::proposeTargets() {
    double car_s = ego.s;
    int lane = int(ego.d/4);
    if (prev_size > 0) {
        car_s = end_path_s;
    }

    // Prediction : Analysing other cars positions.
    bool car_ahead = false;
    bool car_left = false;
    bool car_righ = false;
    for (auto car: traffic) {
        int car_lane = -1;
        // is it on the same lane we are
        if ( car.d > 0 && car.d < 4 ) {
            car_lane = 0;
        } else if ( car.d > 4 && car.d < 8 ) {
            car_lane = 1;
        } else if ( car.d > 8 && car.d < 12 ) {
            car_lane = 2;
        }
        if (car_lane < 0) {
            continue;
        }
        // Find car speed.
        double check_speed = car.velocity;
        double check_car_s = car.s;
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
    printf("s %f d %f speed_diff %f\n", ego.s+30,  4.0*lane+2, speed_diff);
    
    lane = int(ego.s/100)%3;
    return {4.0*lane+2, speed_diff};
}