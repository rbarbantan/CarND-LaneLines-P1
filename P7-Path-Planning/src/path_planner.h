#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include "trajectory_planner.h"


class PathPlanner {

    public:
        void setWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);
        void updateEgo(double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y);
        vector<vector<double>> plan_trajectory(double target_s, double target_d, double target_speed);
    
    private:
        TrajectoryPlanner trajectory_planner;
};

#endif // PATH_PLANNER_H_
