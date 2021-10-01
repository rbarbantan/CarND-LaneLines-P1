#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

#include <vector>

using std::vector;

class TrajectoryPlanner {
    public:
        vector<vector<double>> trajectory_for_target(double target_s, double target_d, double target_speed);
        void setWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);
        void updateEgo(double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_speed,
                        vector<double> previous_path_x, vector<double> previous_path_y);

    private:
        vector<double> map_waypoints_s;
        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        double car_x; 
        double car_y; 
        double car_yaw; 
        double car_s; 
        double car_d; 
        double car_speed;
        double ref_vel;
        vector<double> previous_path_x;
        vector<double> previous_path_y;
};
#endif // TRAJECTORY_PLANNER_H_