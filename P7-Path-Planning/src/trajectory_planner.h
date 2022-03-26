#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

#include <vector>
#include "vehicle.h"

using std::vector;

class TrajectoryPlanner {
    public:
        vector<vector<double>> trajectory_for_target(double target_d, double target_speed);
        void setWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);
        void updateEgo(Vehicle ego, vector<double> previous_path_x, vector<double> previous_path_y);
        double ref_vel;
        
    private:
        vector<double> map_waypoints_s;
        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        Vehicle ego;
        vector<double> previous_path_x;
        vector<double> previous_path_y;
};
#endif // TRAJECTORY_PLANNER_H_