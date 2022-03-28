#include "path_planner.h"

void PathPlanner::setWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
    trajectory_planner.setWaypoints(map_waypoints_s, map_waypoints_x, map_waypoints_y);
}

void PathPlanner::updateEgo(Vehicle ego, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d) {
    behavior_planner.updateEgo(ego, previous_path_x, previous_path_y, end_path_s, end_path_d);
    trajectory_planner.updateEgo(ego, previous_path_x, previous_path_y);
}

void PathPlanner::updateTraffic(vector<Vehicle> traffic) {
    behavior_planner.updateTraffic(traffic);
} 

vector<vector<double>> PathPlanner::plan_trajectory() {
    vector<double> targets = behavior_planner.proposeTargets();
    vector<vector<double>> trajectory = trajectory_planner.trajectory_for_target(targets[0], targets[1]);
    behavior_planner.ref_vel = trajectory_planner.ref_vel;
    return trajectory;
}