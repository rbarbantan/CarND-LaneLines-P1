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