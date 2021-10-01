#include "path_planner.h"

void PathPlanner::setWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
    trajectory_planner.setWaypoints(map_waypoints_s, map_waypoints_x, map_waypoints_y);
}

void PathPlanner::updateEgo(double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_speed, 
                            vector<double> previous_path_x, vector<double> previous_path_y) {
    trajectory_planner.updateEgo(car_x, car_y, car_yaw, car_s, car_d, car_speed, previous_path_x, previous_path_y);
}

vector<vector<double>> PathPlanner::plan_trajectory(double target_s, double target_d, double target_speed) {
    return trajectory_planner.trajectory_for_target(target_s, target_d, target_speed);
}