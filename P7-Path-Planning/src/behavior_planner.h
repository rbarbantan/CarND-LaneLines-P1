#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <vector>
#include "vehicle.h"

using std::vector;


class BehaviorPlanner {
    public:
        void updateTraffic(vector<Vehicle> traffic);
        void updateEgo(Vehicle ego, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
        Goal proposeTargets();
        double ref_vel = 0;
    
    private:
        vector<Vehicle> traffic;
        Vehicle ego;
        int prev_size;
        double end_path_s;
        double end_path_d;
        int maneuver_age = 0;
        int previous_lane = -1;
        bool isLaneEmpty(int lane, double, int time_step, vector<Vehicle> traffic);
        bool isLaneChangeValid(int source_lane, double source_s, int target_lane, int time_step, vector<Vehicle> traffic);
};

#endif // BEHAVIOR_PLANNER_H_