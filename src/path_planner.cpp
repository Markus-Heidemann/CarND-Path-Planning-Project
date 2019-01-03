#include "path_planner.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

std::vector<std::vector<double>> PathPlanner::getPath(VehicleData &veh_data,
                                                      const vector<double> &maps_x,
                                                      const vector<double> &maps_y,
                                                      const vector<double> &maps_s,
                                                      const vector<double> &maps_dx,
                                                      const vector<double> &maps_dy)
{
    int lane = 1;
    double ref_vel = 49.5;

    double car_x = veh_data.car_x;
    double car_y = veh_data.car_y;
    double car_yaw = deg2rad(veh_data.car_yaw);

    std::vector<double> pts_x;
    std::vector<double> pts_y;

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    int prev_path_size = veh_data.previous_path_x.size();

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = car_yaw;

    if (prev_path_size < 2)
    {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        pts_x.push_back(prev_car_x);
        pts_x.push_back(car_x);

        pts_y.push_back(prev_car_y);
        pts_y.push_back(car_y);
    }
    else
    {
        ref_x = veh_data.previous_path_x[prev_path_size - 1];
        ref_y = veh_data.previous_path_y[prev_path_size - 1];

        double ref_prev_x = veh_data.previous_path_x[prev_path_size - 2];
        double ref_prev_y = veh_data.previous_path_y[prev_path_size - 2];

        ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);

        pts_x.push_back(ref_prev_x);
        pts_x.push_back(ref_x);

        pts_y.push_back(ref_prev_y);
        pts_y.push_back(ref_y);
    }

    double wp_offset = 30;

    // calculate waypoints 30, 60, and 90 meters ahead of the ego vehicle
    for (unsigned int i = 0; i < 3; i++)
    {
        vector<double> wp = getXY(veh_data.car_s + 30 * (i + 1), 2 + lane * 4, maps_s, maps_x, maps_y);
        pts_x.push_back(wp[0]);
        pts_y.push_back(wp[1]);
    }

    // coordinate transformation to ego vehicle coordinate system
    for (unsigned int i = 0; i < pts_x.size(); i++)
    {
        double shift_x = pts_x[i] - ref_x;
        double shift_y = pts_y[i] - ref_y;

        pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    tk::spline s;

    s.set_points(pts_x, pts_y);

    for (unsigned int i = 0; i < prev_path_size; i++)
    {
        next_x_vals.push_back(veh_data.previous_path_x[i]);
        next_y_vals.push_back(veh_data.previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_addon = 0;

    for (unsigned int i = 0; i < 50 - veh_data.previous_path_x.size(); i++)
    {
        double N = target_dist / (0.02 * ref_vel / 2.24);
        double x_point = x_addon + target_x / N;
        double y_point = s(x_point);

        x_addon = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // back transformation to global coordinate system
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    return {next_x_vals, next_y_vals};
}

vector<double> JMT(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    MatrixXd A = MatrixXd(3, 3);
    A << T * T * T, T * T * T * T, T * T * T * T * T,
        3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
        6 * T, 12 * T * T, 20 * T * T * T;

    MatrixXd B = MatrixXd(3, 1);
    B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai * B;

    vector<double> result = {start[0], start[1], .5 * start[2]};
    for (int i = 0; i < C.size(); i++)
    {
        result.push_back(C.data()[i]);
    }

    return result;
}