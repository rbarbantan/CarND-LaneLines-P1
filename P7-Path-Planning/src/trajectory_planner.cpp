#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
#include "helpers.h"
#include "trajectory_planner.h"

void TrajectoryPlanner::setWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
}

void TrajectoryPlanner::updateEgo(double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_speed, 
                                    vector<double> previous_path_x, vector<double> previous_path_y) {
    this->car_x = car_x;
    this->car_y = car_y;
    this->car_yaw = car_yaw;
    this->car_s = car_s;
    this->car_d = car_d;
    this->car_speed = car_speed;
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
}

vector<vector<double>> TrajectoryPlanner::trajectory_for_target(double target_s, double target_d, double target_speed) {
    int prev_size = previous_path_x.size();

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // add previous predictions (to assure continuity) to high level trajectory
    if (prev_size < 2)
    {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // add next waypoints to high level trajectory
    vector<double> next_wp0 = getXY(car_s + 30, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); ++i)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    // compute a spline from proposed high level trajectory
    tk::spline s;
    s.set_points(ptsx, ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (int i = 0; i < previous_path_x.size(); ++i)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30;
    //double target_x = target_s - car_s;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_addon = 0;

    printf("ref_vel %f", ref_vel);
    for (int i = 1; i <= 50 - prev_size; ++i)
    {   
        if (ref_vel > target_speed) {
            ref_vel = target_speed;
        } else if (ref_vel < target_speed) {
            ref_vel += 0.36;
        }
        double n = target_dist / (DT * ref_vel);
        double x_point = x_addon + target_x / n;
        double y_point = s(x_point);

        x_addon = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    vector<double> velocities;
    for (int i=1; i<next_x_vals.size(); i++) {
        double d = distance(next_x_vals[i], next_y_vals[i], next_x_vals[i-1], next_y_vals[i-1]);
        velocities.push_back(d/DT);
    }
    double total_acc = 0;
    for (int i=1; i<velocities.size(); i++) {
        double acc = abs(velocities[i]-velocities[i-1]);
        total_acc += acc;
    }
    printf("total acc: %f\n", total_acc);
    return {next_x_vals, next_y_vals};
};
