#include "path_planner.h"
#include "spline.h"

std::vector<std::vector<double>> PathPlanner::getPath(VehicleData &veh_data)
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

    return {next_x_vals, next_y_vals};
}

std::vector<std::vector<double>> PathPlanner::followLane(VehicleData &veh_data,
                                                         int lane_idx,
                                                         double speed,
                                                         const vector<double> &maps_x,
                                                         const vector<double> &maps_y,
                                                         const vector<double> &maps_s,
                                                         const vector<double> &maps_dx,
                                                         const vector<double> &maps_dy)
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

    double dist_inc = 0.4;

    vector<double> frenet = getFrenet(pos_x, pos_y, angle, maps_x, maps_y);
    double frenet_s = frenet[0];
    double frenet_d = frenet[1];

    for (int i = 0; i < 50 - path_size; i++)
    {
        frenet_s += dist_inc;
        vector<double> cartesian = getXY(frenet_s, frenet_d, maps_s, maps_x, maps_y);

        next_x_vals.push_back(cartesian[0]);
        next_y_vals.push_back(cartesian[1]);
    }

    return {next_x_vals, next_y_vals};
}