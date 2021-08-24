#ifndef EVALUATE_H
#define EVALUATE_H

#include <vector>
#include <chrono>
#include <limits>
#include "helpers.h"

using std::vector;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

const double DISTANCE_TO_GOAL = 30.0;
const double SAFE_DISTANCE = 3.0;
const double MAX_ACCELERATION = 10.0;
const vector<double> weights = {0.5, 0.0, 0.1, 0.4};

vector<double> predict(double x, double y, double vx, double vy, double t)
{
    /**
     * Predicts where a car will be at time t (in cartesian coordinates)
    */

    // use simple constrant velocity model
    double pred_x = x + vx * t;
    double pred_y = y + vy * t;

    return {pred_x, pred_y};
}

double evaluate_collisions(vector<vector<double>> trajectory, vector<vector<double>> sensor_fusion)
{
    /**
     * Check if the predicted trajectory leads to any collisions
    */
    double closest = std::numeric_limits<double>::max();

    vector<double> x_pos = trajectory[0];
    vector<double> y_pos = trajectory[1];

    for (int i = 0; i < x_pos.size(); ++i)
    { // each planned point
        double t = i * 0.02;
        for (auto car : sensor_fusion)
        { // for each car in front
            double x = car[1];
            double y = car[2];
            double vx = car[3];
            double vy = car[4];
            vector<double> prediction = predict(x, y, vx, vy, t);
            double dist = distance(x_pos[i], y_pos[i], prediction[0], prediction[1]);

            // find closest point between prediction and traffic
            if (dist <= closest)
            {
                closest = dist;
            }
        }
    }
    // check for collision
    if (closest < SAFE_DISTANCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

double evaluate_distance_to_target(vector<vector<double>> trajectory, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
    /**
     * Evaluate how far is the proposed trajectory from the given waypoints
    */
    vector<double> x_pos = trajectory[0];
    vector<double> y_pos = trajectory[1];
    double min_distance = std::numeric_limits<double>::max();

    for (int i = 0; i < x_pos.size(); ++i)
    { // each planned point
        for (int j = 0; j < map_waypoints_x.size(); ++j)
        {
            double d = distance(x_pos[i], y_pos[i], map_waypoints_x[j], map_waypoints_y[j]);
            if (d < min_distance)
            {
                min_distance = d;
            }
        }
    }

    if (min_distance > DISTANCE_TO_GOAL)
    {
        return 1;
    }
    else
    {
        return min_distance / DISTANCE_TO_GOAL;
    }
}

vector<double> compute_velocities(vector<vector<double>> trajectory)
{
    vector<double> velocities;
    vector<double> x_pos = trajectory[0];
    vector<double> y_pos = trajectory[1];
    for (int i = 1; i < x_pos.size(); ++i)
    {
        velocities.push_back(distance(x_pos[i - 1], y_pos[i - 1], x_pos[i], y_pos[i]) / DT);
    }
    std::cout << "Vel: ";
    print_vector(velocities);
    return velocities;
}

double evaluate_velocity(vector<vector<double>> trajectory)
{
    vector<double> velocities = compute_velocities(trajectory);
    double max_velocity = *std::max_element(velocities.begin(), velocities.end());

    //convert to mph
    max_velocity = max_velocity * MPS_TO_MPH;
    //std::cout << max_speed << " ";
    if (max_velocity > MAX_VELOCITY)
    {
        return 1;
    }
    else
    {
        return (MAX_VELOCITY - max_velocity) / MAX_VELOCITY;
    }
}

vector<double> compute_accelerations(vector<vector<double>> trajectory)
{
    vector<double> accelerations;
    vector<double> velocities = compute_velocities(trajectory);
    for (int i = 1; i < velocities.size(); ++i)
    {
        accelerations.push_back(std::abs(velocities[i] - velocities[i - 1]) / DT);
    }
    return accelerations;
}

double evaluate_acceleration(vector<vector<double>> trajectory)
{
    vector<double> accelerations = compute_accelerations(trajectory);
    double total_acc = std::accumulate(accelerations.begin(), accelerations.end(), 0.0);
    std::cout << "t_acc: " << total_acc << "\n";
    if (total_acc < -MAX_ACCELERATION || total_acc > MAX_ACCELERATION)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

double evaluate_trajectory(vector<vector<double>> trajectory, vector<vector<double>> sensor_fusion,
                           vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
    double collision_cost = evaluate_collisions(trajectory, sensor_fusion);
    double distance_cost = evaluate_distance_to_target(trajectory, map_waypoints_x, map_waypoints_y);
    double velocity_cost = evaluate_velocity(trajectory);
    double acceleration_cost = evaluate_acceleration(trajectory);
    double total_cost = collision_cost * weights[0] + distance_cost * weights[1] + velocity_cost * weights[2] + acceleration_cost * weights[3];
    //std::cout << "c: " << collision_cost << ", v: " << velocity_cost << ", a:" << acceleration_cost << ", t: " << total_cost << "\n";
    return total_cost;
}

vector<vector<double>> hist;
vector<double> errors;

void measure_prediction_error(vector<vector<double>> sensor_fusion)
{
    /**
     * Utility function to evaluate how good is our prediction for other cars
    */
    int car_to_test = 2;
    double prediction_time = 0.5;

    double now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    for (auto car : sensor_fusion)
    {
        if (car[0] == car_to_test)
        {
            // predict a future position
            auto prediction = predict(car[1], car[2], car[3], car[4], prediction_time);

            // save prediction
            hist.push_back({now + prediction_time * 1000, prediction[0], prediction[1]});

            // go through history of predictions and compare current poistion with predicted one
            for (int j = 0; j < hist.size(); ++j)
            {
                if (abs(hist[j][0] - now) <= 5)
                {
                    errors.push_back(distance(hist[j][1], hist[j][2], car[1], car[2]));
                    break;
                }
            }

            if (hist.size() % 10 == 0)
            {
                std::cout << accumulate(errors.begin(), errors.end(), 0.0) / errors.size() << "\n";
            }
        }
    }
}
#endif // EVALUATE_H