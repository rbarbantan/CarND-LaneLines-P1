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
    
    private:
        vector<Vehicle> traffic;
        Vehicle ego;
        int prev_size;
        double end_path_s;
        double end_path_d;
};

#endif // BEHAVIOR_PLANNER_H_