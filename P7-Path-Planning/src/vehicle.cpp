#include "vehicle.h"

Vehicle::Vehicle(){
    id = -1;
}

Vehicle::Vehicle(int id, double d, double s, double velocity) {
    this->id = id;
    this->d = d;
    this->s = s;
    this->velocity = velocity;
}
Vehicle::~Vehicle() {}

Goal::Goal(int lane, double delta_velocity) {
    this->lane = lane;
    this->delta_velocity = delta_velocity;
}

Goal::~Goal() {}