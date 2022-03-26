#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <vector>
#include "vehicle.h"

using std::vector;


class BehaviorPlanner {
    public:
        void updateTraffic(vector<Vehicle> traffic);
        void updateEgo(Vehicle ego, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s);
        vector<double> proposeTargets();
        double ref_vel;
    
    private:
        vector<Vehicle> traffic;
        Vehicle ego;
        int prev_size;
        double end_path_s;
};

#endif // BEHAVIOR_PLANNER_H_