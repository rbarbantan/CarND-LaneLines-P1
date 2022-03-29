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

    if (prev_size > 0) {
        ego_s = end_path_s;
        //ego_d = end_path_d;
    }
    
    int ego_lane = int(ego_d / LANE_WIDTH);

    // default behavior: go as fast as possible in current lane
    double target_delta_velocity = VEL_INCREMENT;
    int target_lane = ego_lane;

    for (auto car: traffic) {
        int car_lane = int(car.d / LANE_WIDTH);
        if (ego_lane == car_lane) {
            // Estimate car s position after executing previous trajectory.
            double predicted_s = car.s + ((double)prev_size * DT * car.velocity);
            if (predicted_s > ego_s  && (predicted_s - ego_s < LOOK_AHEAD)) { // ahead and too close
                if (isLaneChangeValid(ego_lane, ego_s, ego_lane-1, prev_size, traffic)) { // left lane change
                    //printf("left\n");
                    target_lane = ego_lane - 1;
                } else if (isLaneChangeValid(ego_lane, ego_s, ego_lane+1, prev_size, traffic)) { //right lane change
                    //printf("right\n");
                    target_lane = ego_lane + 1;
                } else {
                    target_delta_velocity = -VEL_INCREMENT;
                }
                break;
            }
        }
    }
    
    Goal goal = Goal(target_lane, target_delta_velocity);
    return goal;
}

bool BehaviorPlanner::isLaneEmpty(int lane, double s, int time_step, vector<Vehicle> traffic) {
    bool empty = true;
    for (auto car: traffic) {
        int car_lane = int(car.d / LANE_WIDTH);
        if (car_lane == lane) {
            double predicted_s = car.s + ((double)time_step * DT * car.velocity);
            if ((s-LOOK_BEHIND < predicted_s) && (predicted_s < s+LOOK_AHEAD)) { 
                empty = false;
                break;
            }
        }
    }
    return empty;
}

bool BehaviorPlanner::isLaneChangeValid(int source_lane, double source_s, int target_lane, int time_step, vector<Vehicle> traffic){
    bool legal = (MIN_LANE <= target_lane) && (target_lane <= MAX_LANE) && ((source_lane > MIN_LANE && target_lane > MIN_LANE) || (source_lane < MAX_LANE && target_lane < MAX_LANE));
    //printf("%d %d %d\n", source_lane, target_lane, legal);
    return legal && isLaneEmpty(target_lane, source_s, time_step, traffic);
}
