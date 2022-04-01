#include <cstdio>
#include <cmath>
#include "behavior_planner.h"
#include "constants.h"

void BehaviorPlanner::updateTraffic(vector<Vehicle> traffic) {
    this->traffic = traffic;
}

void BehaviorPlanner::updateEgo(Vehicle ego, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d) {
    this->ego = ego;
    this->prev_size = previous_path_x.size();
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
}

Goal BehaviorPlanner::proposeTargets() {
    
    double ego_s = ego.s;
    double ego_d = ego.d;

    // if available, compute for the last point on the trajecotry
    if (prev_size > 0) {
        ego_s = end_path_s;
        ego_d = end_path_d;
    }
    
    int ego_lane = int(ego_d / LANE_WIDTH);
    
    // default behavior: go as fast as possible in current lane
    double target_delta_velocity = 0;
    int target_lane = ego_lane;

    bool too_close = false;
    bool car_to_left = false;
    bool car_to_right = false;

    for (auto car: traffic) {
        int car_lane = int(car.d / LANE_WIDTH);
        // Estimate car s position after executing previous trajectory.
        double predicted_s = car.s + ((double)prev_size * DT * car.velocity);

        if (car_lane == ego_lane) { // traffic ahead
            too_close |= (predicted_s > ego_s) && ((predicted_s - ego_s) < LOOK_AHEAD);
        } else if (car_lane == ego_lane - 1) { // traffic to the left
            car_to_left |= ((ego_s - LOOK_BEHIND) < predicted_s) && ((ego_s + LOOK_AHEAD) > predicted_s);
        } else if (car_lane == ego_lane + 1) { // traffic to the right
            car_to_right |= ((ego_s - LOOK_BEHIND) < predicted_s) && ((ego_s + LOOK_AHEAD) > predicted_s);
        }
    }

    if (too_close) {
        if (!car_to_right && target_lane < MAX_LANE) {  // switch to right lane
            target_lane += 1;
        } else if (!car_to_left && target_lane > MIN_LANE) { // switch to left lane
            target_lane -= 1;
        } else { // slow down
            target_delta_velocity = -VEL_INCREMENT;
        }
    } else {
        int mid_lane = int((MAX_LANE-MIN_LANE)/2);
        if ((target_lane != mid_lane) && ((target_lane == MAX_LANE && !car_to_left) || (target_lane == MIN_LANE && !car_to_right))) {
            // switch back to middle lane
            target_lane = mid_lane;
        } else {
            // traffic clear so speed up
            target_delta_velocity = VEL_INCREMENT;
        }
    }
    
    Goal goal = Goal(target_lane, target_delta_velocity);
    return goal;
}
