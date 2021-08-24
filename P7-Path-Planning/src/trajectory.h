#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <random>

#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"

using std::vector;

const double TARGET_DISTANCE = 20.0; // m

// maximum disturbance to be aplied to the given goal when proposing a trajectory
const double MAX_SPEED_DISTURBANCE = 10; // m/s
const double MAX_ACC = 9.0;

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> dis(-1.0, 1.0);


vector<vector<double>> get_simple_frenet(double car_s, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i)
    {
        double next_s = car_s + (i + 1) * dist_inc;
        double next_d = 6;
        vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
    return {next_x_vals, next_y_vals};
}

vector<vector<double>> trajectory_for_target(double car_x, double car_y, double car_yaw, double car_s,
                                             vector<double> previous_path_x, vector<double> previous_path_y, 
                                             vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                                             double target_s, double target_d, double target_speed) {
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

    double target_x = 30; //target_s - car_s;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_addon = 0;
    
    double actual_speed = 1; // start slow
    if (prev_size >= 2) {
        double last_speed = distance(previous_path_x[prev_size-1], previous_path_y[prev_size-1], previous_path_x[prev_size-2], previous_path_y[prev_size-2]) / DT;
        double desired_acc = target_speed - last_speed;
        double actual_acc = std::max(-MAX_ACC, std::min(MAX_ACC, desired_acc));
        actual_speed = last_speed + actual_acc * DT;
    }
    for (int i = 1; i <= 50 - prev_size; ++i)
    {
        double n = target_dist / (DT * actual_speed);
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
    return {next_x_vals, next_y_vals};
}

vector<vector<double>> propose_trajectory(double car_x, double car_y, double car_yaw, double car_s,
                                          vector<double> previous_path_x, vector<double> previous_path_y,
                                          vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
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
    vector<double> next_wp0 = getXY(car_s + 30, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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

    double ref_vel = (MAX_VELOCITY - 0.5) - (1 - dis(gen)) * MAX_SPEED_DISTURBANCE;
    double target_x = (1 + dis(gen)) * TARGET_DISTANCE;
    //std::cout << ref_vel << ", " << target_x << "; ";

    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_addon = 0;

    for (int i = 1; i <= 50 - prev_size; ++i)
    {
        double n = target_dist / (DT * ref_vel / MPS_TO_MPH);
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
    return {next_x_vals, next_y_vals};
}

vector<double> JMT(vector<double> &start, vector<double> &end, double T)
{
    /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
    MatrixXd A = MatrixXd(3, 3);
    A << T * T * T, T * T * T * T, T * T * T * T * T,
        3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
        6 * T, 12 * T * T, 20 * T * T * T;

    MatrixXd B = MatrixXd(3, 1);
    B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai * B;

    vector<double> result = {start[0], start[1], .5 * start[2]};

    for (int i = 0; i < C.size(); ++i)
    {
        result.push_back(C.data()[i]);
    }

    return result;
}

#endif // TRAJECTORY_H