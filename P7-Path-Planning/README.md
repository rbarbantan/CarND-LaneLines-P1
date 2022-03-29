# **CarND-Path-Planning-Project**
Self-Driving Car Engineer Nanodegree Program
   
## Project scope
The goal for this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

For more details on the setup and constraints, see the [original instructions](instructions.md)


## Writeup / README
I tried to follow the architecture suggested in the lecture and organized my code in three separate modules:

### Path Planner
Represents the [planner](src/path_planner.cpp) itself, in order to keep the code separate from the [utility code](src/main.cpp) provided to communicate with the simulation. 
It is a wrapper for the other two modules: a **behavior planner** and **trajectory planner**.
```
vector<vector<double>> PathPlanner::plan_trajectory() {
    Goal goal = behavior_planner.proposeTargets();
    vector<vector<double>> trajectory = trajectory_planner.trajectory_for_target(goal);
    return trajectory;
}
```

I originally invetigated the use of a *prediction* module as well, but I decided to use a very simple physical-based model, so the prediction of other participants in traffic is included in the behavior planner.

### Behavior Planner
This [module](src/behavior_planner.cpp) deals with the high-level logic of the ego car. It analyzes the traffic around the ego car and predicts their motion.
```
for (auto car: traffic) {
        int car_lane = int(car.d / LANE_WIDTH);
        // Estimate car s position after executing previous trajectory.
        double predicted_s = car.s + ((double)prev_size * DT * car.velocity);
```
Then it applies a very simple state machine logic, heavily inspired from the discussion in the knowledgebase. 

It checks for traffic ahead and in the adjacent lanes, and picks the an empty lane, always preferring the middle one, and going as fast as legally possible.
If there is traffic ahead, the car slows down. The pseudo code looks something like:
```
  for car in traffic:
    check if car is too close ahead
    check if car is near ego to the left (+- given distance)
    check if car is near rgo to the left (+- given distance)
  
  if car ahead:
    if left lane is free:
      go left
    else if right lane is free:
      go right
    else:
      slow down
  else:
    if not in middle lane:
      go to middle
    else:
      accelerate
```
The output of the behavior module is the desired [goal](src/vehicle.h#15): an `intended lane`, and `desired action`: *accelerate* or *decelerate*

### Trajectory Planner
Given the current status of the ego car and the goal given by the behavior planner, the role of the trajectory planner is to [compute a valid trajectory](src/trajectory_planner.cpp#19).

By valid, I mean a list of 50 x,y coordinates to be executed by the simulator, each point being 0.02 seconds apart. This trajectory needs to respect all the imposed constraints like maximum allowed velocity, acceleration, jerk, etc. 
For the final solution, I have used the spline approach and library suggested in the `Project Q&A Session`.

I construct a high-level trajectory based on the last 2 points from the old trajectory (for continuity) 
```
ref_x = previous_path_x[prev_size - 1];
ref_y = previous_path_y[prev_size - 1];
...
ptsx.push_back(ref_x_prev);
ptsx.push_back(ref_x);

ptsy.push_back(ref_y_prev);
ptsy.push_back(ref_y);

```
and 3 points in the future, based on the lane proposed by the behavior planner. Here is an example for first way point WP0:
```
vector<double> next_wp0 = getXY(ego.s + WP0, LANE_WIDTH/2 + LANE_WIDTH*goal.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

...
ptsx.push_back(next_wp0[0]);
ptsy.push_back(next_wp0[1]);
...
```
These points are converted in the ego car's frame of reference (for easier calculations) and a spline is fitted.
```
// compute a spline from proposed high level trajectory
tk::spline s;
s.set_points(ptsx, ptsy);
```
The final trajectory consists of the old trajectory not yet executed by the simulated controller, plus some new points computed using the spline we just computed. 
```
for (int i = 0; i < prev_size; ++i)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
```
For these last points the same trick used by the instructor is applied. I compute a set of x points for which I get the corresponding y, based on the spline and taking into account the action (accelerate/decelerate) suggested by the behavior planner.
```
    for (int i = 1; i <= TRAJ_SIZE - prev_size; ++i)
    {   
        
        ref_vel += goal.delta_velocity;
        if ( ref_vel > MAX_VELOCITY ) {
            ref_vel = MAX_VELOCITY;
        } else if ( ref_vel < VEL_INCREMENT ) {
            ref_vel = VEL_INCREMENT;
        }
        
        double n = target_dist / (DT * ref_vel/MPS_TO_MPH);
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
```
Since I wanted to act as fast as possible based on the desired behavior, I apply acceleration/deceleration on each node, so this is where I also limit the velocity to it's legal boundaries.

In the end, the entire trajectory is converted back into world coordinates and sent to the simulation.

### Other approaches
Initially I intended to use a diffrent approach both for the behavior and the trajectory planner (closer to the theory presented in the course as opposed to the Q&A session), but this proved to be more challenging than expected. 

One possible approach would be for the trajectories to be generated using a quintic polynomial solver, but in practice it was difficult to make a smooth transition from one proposed trajectory to another using this approach. One obviously needs to incude trajectory history into the equation, but doing so with just the stat/end states proved challenging.

Once the trajectory generation is stable and flexible enough, it could be used to generate a set of possible candidate with different possible targets: different lanes, different timings, etc. I also implemented soome basic cost evaluation functions, but in practice they also proved difficult to tune the weights properly.

In the end, due to time constraints I chose to revert to a simple state machine similar to the one suggested by mentors in the knowledge section.

### Possible improvements
The most obvious improvement to this project would include re-attempting to implement the failed attempts. With more debugging and better logging an efficient trajectory cost evaluation function is possible and would cover a lot more scenarios than the ones covered by the state machine.

Even better, the "weights" of the various cost functions could be learned by using reinforcement learning or perhaps supervised imitation learning.

In a similar way, predicting the behavior of other participants in traffic could use a more complex model or a neural network model trained on a [dataset like this](https://level-5.global/data/prediction/), using supervised or unsupervised learning.
