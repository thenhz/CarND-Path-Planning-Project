#ifndef BRAIN_H
#define BRAIN_H

#include <vector>
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;

using std::vector;

struct driving_params {
  int lane;
  double vel;
  double car_s;
};

driving_params figureOutNextState(nlohmann::json j, int lane, double actual_vel)
{
    double ref_vel = 0.0;
    // Main car's localization Data
    double car_x = j[1]["x"];
    double car_y = j[1]["y"];
    double car_s = j[1]["s"];
    double car_d = j[1]["d"];
    double car_yaw = j[1]["yaw"];
    double car_speed = j[1]["speed"];

    // Previous path data given to the Planner
    auto previous_path_x = j[1]["previous_path_x"];
    auto previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = j[1]["end_path_s"];
    double end_path_d = j[1]["end_path_d"];
    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    auto sensor_fusion = j[1]["sensor_fusion"];

    int prev_size = previous_path_x.size();
    if (prev_size > 0)
    {
        car_s = end_path_s;
    }
    bool too_close = false;

    //Sensor fusion part
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx - vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size * .02 * check_speed);
            if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
            {
                too_close = true;
                //TODO: change lane or try that
            }
        }
    }

    if (too_close)
    {
        //TODO:Slow down depending on the car in front
        ref_vel = actual_vel - .224;

    }
    else if (actual_vel < 49.5)
    {
        ref_vel = actual_vel + .224;
    }else{
        ref_vel = actual_vel;
    }

    driving_params params;

    params.lane = lane;
    params.vel = ref_vel;
    params.car_s = car_s;

    return params;
}

#endif // BRAIN_H