#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <vector>

using std::vector;


class BehaviorPlanner {
    public:
        vector<double> proposeTargets();
};

#endif // BEHAVIOR_PLANNER_H_