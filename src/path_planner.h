#pragma once

#include "stdafx.h"
#include "helpers.h"

using namespace std;

class PathPlanner
{
  public:
    std::vector<std::vector<double>> getPath(VehicleData &veh_data,
                                             const vector<double> &maps_x,
                                             const vector<double> &maps_y,
                                             const vector<double> &maps_s,
                                             const vector<double> &maps_dx,
                                             const vector<double> &maps_dy);
};