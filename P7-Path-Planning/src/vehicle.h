#ifndef VEHICLE_H_
#define VEHICLE_H_

class Vehicle {
  public:
    Vehicle();
    Vehicle(int id, double d, double s, double velocity);
    ~Vehicle();

    int id;
    double d, s, velocity, x, y, yaw;
    
};

class Goal {
  public:
    Goal(int lane, double delta_velocity);
    ~Goal();

    int lane;
    double delta_velocity;

};
#endif  // VEHICLE_H_