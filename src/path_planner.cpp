#include "path_planner.h"

std::tuple<std::vector<double>, std::vector<double>> PathPlanner::getPath(VehicleData& veh_data)
{
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    double pos_x;
    double pos_y;
    double angle;
    int path_size = veh_data.previous_path_x.size();

    for (int i = 0; i < path_size; i++)
    {
        next_x_vals.push_back(veh_data.previous_path_x[i]);
        next_y_vals.push_back(veh_data.previous_path_y[i]);
    }

    if (path_size == 0)
    {
        pos_x = veh_data.car_x;
        pos_y = veh_data.car_y;
        angle = deg2rad(veh_data.car_yaw);
    }
    else
    {
        pos_x = veh_data.previous_path_x[path_size - 1];
        pos_y = veh_data.previous_path_y[path_size - 1];

        double pos_x2 = veh_data.previous_path_x[path_size - 2];
        double pos_y2 = veh_data.previous_path_y[path_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
    }

    double dist_inc = 0.5;
    for (int i = 0; i < 50 - path_size; i++)
    {
        next_x_vals.push_back(pos_x + (dist_inc)*cos(angle + (i + 1) * (pi() / 100)));
        next_y_vals.push_back(pos_y + (dist_inc)*sin(angle + (i + 1) * (pi() / 100)));
        pos_x += (dist_inc)*cos(angle + (i + 1) * (pi() / 100));
        pos_y += (dist_inc)*sin(angle + (i + 1) * (pi() / 100));
    }

    return std::make_tuple(next_x_vals, next_y_vals);
}