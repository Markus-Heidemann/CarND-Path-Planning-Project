#pragma once

#include "stdafx.h"
#include "helpers.h"

using namespace std;

class PathPlanner {
public:
    std::tuple<std::vector<double>, std::vector<double>> getPath(VehicleData& veh_data);
};