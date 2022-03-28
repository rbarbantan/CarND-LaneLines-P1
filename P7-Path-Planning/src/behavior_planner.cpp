#include <cstdio>
#include "behavior_planner.h"

void BehaviorPlanner::updateTraffic(vector<Vehicle> traffic) {
    this->traffic = traffic;
}

void BehaviorPlanner::updateEgo(Vehicle ego, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d) {
    this->ego = ego;
    this->prev_size = previous_path_x.size();
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
}

vector<double> BehaviorPlanner::proposeTargets() {
    const double delta = 0.224;
    double ego_s = ego.s;
    double ego_d = ego.d;

    if (prev_size > 0) {
        ego_s = end_path_s;
        ego_d = end_path_d;
    }
    
    int ego_lane = int(ego_d/4);

    // default behavior: go as fast as possible in current lane
    double target_delta_velocity = delta;
    int target_lane = ego_lane;

    for (auto car: traffic) {
        int car_lane = int(car.d/4);
        if (ego_lane == car_lane) {
            // Estimate car s position after executing previous trajectory.
            double predicted_s = car.s + ((double)prev_size*0.02*car.velocity);
            if (predicted_s > ego_s  && (predicted_s - ego_s < 30)) { // ahead and too close
                if (ego_lane > 0) { // try left lane change
                    target_lane -= 1;
                } else if (ego.d < 2) { //try right lane change
                    target_lane += 1;
                }  // slow down
                target_delta_velocity = -delta;
                
                break;
            }
        }
    }

    printf("target lane %d, target_delta_vel %f\n", target_lane, target_delta_velocity);
    return {2.0 + 4.0*target_lane, target_delta_velocity};
}