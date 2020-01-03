#ifndef DRIVING_PARAMS_H
#define DRIVING_PARAMS_H

#include <string>

using std::string;

class DrivingParams
{
public:
    // Constructors
    DrivingParams();

    // Destructor
    virtual ~DrivingParams();

    int lane;
    double vel;
    double car_s;
    double acc;
    double yaw;
    string new_state;
    int proposed_lane;
    double proposed_vel;
};

#endif