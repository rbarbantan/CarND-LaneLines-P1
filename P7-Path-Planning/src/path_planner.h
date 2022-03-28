#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include "trajectory_planner.h"
#include "behavior_planner.h"

class PathPlanner {

    public:
        void setWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);
        void updateEgo(Vehicle ego, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
        void updateTraffic(vector<Vehicle> traffic);
        vector<vector<double>> plan_trajectory();
    
    private:
        BehaviorPlanner behavior_planner;
        TrajectoryPlanner trajectory_planner;
};

#endif // PATH_PLANNER_H_
