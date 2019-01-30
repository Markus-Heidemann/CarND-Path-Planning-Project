#include "path_planner.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Trajectory PathPlanner::getPath(VehicleData &veh_data,
                                const vector<double> &maps_x,
                                const vector<double> &maps_y,
                                const vector<double> &maps_s,
                                const vector<double> &maps_dx,
                                const vector<double> &maps_dy)
{
    MapData map_data = MapData(maps_x, maps_y, maps_s, maps_dx, maps_dy);

    int curr_lane = 1;
    int lane_for_traj;
    double ref_vel = v_max;
    double target_vel = v_max;

    double car_x = veh_data.car_x;
    double car_y = veh_data.car_y;
    double car_s = veh_data.car_s;
    double car_vel = veh_data.car_speed;
    double car_yaw = deg2rad(veh_data.car_yaw);
    double end_path_s = veh_data.end_path_s;

    int prev_path_size = veh_data.previous_path_x.size();

    curr_lane = lane_from_d(veh_data.car_d);

    if (car_vel < 40.0)
    {
        m_wp_offset = 35.0;
    }
    else
    {
        m_wp_offset = 50.0;
    }


    std::vector<double> lane_speeds = std::vector<double>(num_lanes, v_max);

    if (0 < veh_data.sensor_fusion.size())
    {
        vector<vector<FusionObjData>> fus_obj_by_lane = vector<vector<FusionObjData>>(num_lanes, vector<FusionObjData>());

        for (auto fus_obj : veh_data.sensor_fusion)
        {
            int lane_idx = lane_from_d(fus_obj.d);
            if (0 <= lane_idx)
            {
                fus_obj_by_lane[lane_idx].push_back(fus_obj);
            }
        }

        lane_speeds = getLaneSpeeds(fus_obj_by_lane, car_s, 2.0 * car_vel / 2.24);
        auto fastest_lanes = getFastestLane(lane_speeds);
        assert(fastest_lanes.size() > 0);

        bool first_state_evalution = true;

        /*
         *  FOLLOWLANE
         */
        if ((eState::FOLLOWLANE == m_state) && first_state_evalution)
        {
            first_state_evalution = false;
            lane_for_traj = curr_lane;

            // check, if current lane is not one of the fastest lanes
            if (std::find(fastest_lanes.begin(), fastest_lanes.end(), curr_lane) == fastest_lanes.end())
            {
                // get the closest of the fastest lane
                int min_diff = 999; //INT_INF;
                for (const auto& poss_lane : fastest_lanes)
                {
                    int diff = abs(poss_lane - curr_lane);
                    if (diff < min_diff)
                    {
                        min_diff = diff;
                        m_target_lane = poss_lane;
                    }
                }
                m_state = eState::PREPARELANECHANGE;
                std::cout << "PREPARELANECHANGE: " << m_target_lane << "\n";
            }
            // get target vel from vehicle in front
            target_vel = setACCVel(fus_obj_by_lane, curr_lane, car_s, veh_data.end_path_s);
        }

        /*
         *  PREPARELANECHANGE
         */
        if ((eState::PREPARELANECHANGE == m_state) && first_state_evalution)
        {
            first_state_evalution = false;
            lane_for_traj = curr_lane;

            // check, if target curr_lane is left or right
            int lane_chg_dir = 0;
            if (m_target_lane > curr_lane)
            {
                lane_chg_dir = 1;
            }
            else
            {
                lane_chg_dir = -1;
            }

            m_tmp_target_lane = curr_lane + lane_chg_dir;

            bool lane_chg_possible = checkIfLaneChangePossible(veh_data,
                                                               fus_obj_by_lane[m_tmp_target_lane],
                                                               m_tmp_target_lane,
                                                               lane_speeds[m_tmp_target_lane],
                                                               map_data);

            if (lane_chg_possible)
            {
                lane_for_traj = m_tmp_target_lane;
                m_state = eState::CHANGELANE;
                std::cout << "CHANGELANE\n";
            }
            else
            {
                // reevaluate, if lane change is necessary
                if (std::find(fastest_lanes.begin(), fastest_lanes.end(), curr_lane) != fastest_lanes.end())
                {
                    m_state = eState::FOLLOWLANE;
                    std::cout << "FOLLOWLANE\n";
                }
            }
            target_vel = setACCVel(fus_obj_by_lane, curr_lane, car_s, veh_data.end_path_s);
        }

        /*
         *  CHANGELANE
         */
        if ((eState::CHANGELANE == m_state) && first_state_evalution)
        {
            first_state_evalution = false;
            lane_for_traj = curr_lane;
            if (curr_lane == m_tmp_target_lane)
            {
                if (m_rep_ctr > 49)
                {
                    m_rep_ctr = 0;
                    m_state = eState::FOLLOWLANE;
                    std::cout << "FOLLOWLANE\n";
                }
                else
                {
                    m_rep_ctr++;
                }
            }
            else
            {
                lane_for_traj = m_tmp_target_lane;
                m_wp_offset = 50.0;
            }
            target_vel = 1.0 * setACCVel(fus_obj_by_lane, curr_lane, car_s, veh_data.end_path_s);
        }
    }

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = car_yaw;

    Trajectory rough_traj = getRoughTrajectoryStart(veh_data);

    // calculate the reference point from where the trajectory is extended
    if (prev_path_size > 1)
    {
        ref_x = veh_data.previous_path_x[prev_path_size - 1];
        ref_y = veh_data.previous_path_y[prev_path_size - 1];
        double ref_prev_x = veh_data.previous_path_x[prev_path_size - 2];
        double ref_prev_y = veh_data.previous_path_y[prev_path_size - 2];
        ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);
    }

    // calculate waypoints 30, 60, and 90 meters ahead of the ego vehicle
    for (unsigned int i = 0; i < 2; i++)
    {
        vector<double> wp = getXY(veh_data.car_s + m_wp_offset * (i + 1), 2 + lane_for_traj * 4, maps_s, maps_x, maps_y);
        rough_traj.x.push_back(wp[0]);
        rough_traj.y.push_back(wp[1]);
    }

    Trajectory traj = calculateTrajectory(rough_traj, ref_x, ref_y, ref_yaw, target_vel, 50, veh_data, map_data);

    return traj;
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

Trajectory PathPlanner::calculateTrajectory(Trajectory rough_traj,
                                            double ref_x,
                                            double ref_y,
                                            double ref_yaw,
                                            double target_vel,
                                            int num_steps,
                                            const VehicleData &veh_data,
                                            const MapData &map_data)
{
    Trajectory traj;

    int prev_path_size = veh_data.previous_path_x.size();

    for (unsigned int i = 0; i < prev_path_size; i++)
    {
        traj.x.push_back(veh_data.previous_path_x[i]);
        traj.y.push_back(veh_data.previous_path_y[i]);
    }

    // coordinate transformation to ego vehicle coordinate system
    for (unsigned int i = 0; i < rough_traj.x.size(); i++)
    {
        double shift_x = rough_traj.x[i] - ref_x;
        double shift_y = rough_traj.y[i] - ref_y;

        rough_traj.x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        rough_traj.y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    tk::spline s;

    s.set_points(rough_traj.x, rough_traj.y);

    double target_x = m_wp_offset;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_addon = 0;

    // double arc_len_addon = 0;

    for (unsigned int i = 0; i < num_steps - prev_path_size; i++)
    {
        double curr_vel;

        // calculate speed at the end of the current trajectory
        if (prev_path_size > 0)
        {
            double prev_dx = traj.x[traj.x.size() - 1] - traj.x[traj.x.size() - 2];
            double prev_dy = traj.y[traj.y.size() - 1] - traj.y[traj.y.size() - 2];
            curr_vel = (sqrt(prev_dx * prev_dx + prev_dy * prev_dy) / 0.02) * 2.24;
        }
        else
        {
            curr_vel = veh_data.car_speed;
        }

        if (target_vel < curr_vel)
        {
            curr_vel -= a_max * 2.24;
        }
        else
        {
            curr_vel += a_max * 2.24;
        }

        double N = target_dist / (0.02 * curr_vel / 2.24);
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

        traj.x.push_back(x_point);
        traj.y.push_back(y_point);
    }
    return traj;
}

std::vector<double> PathPlanner::getLaneSpeeds(const std::vector<FusionData> &veh_in_lanes,
                                                const double &s,
                                                const double max_dist)
{
    auto vehicles_front_by_lane = getVehiclesFront(veh_in_lanes, s, max_dist);
    std::vector<double> res = std::vector<double>(vehicles_front_by_lane.size(), v_max);

    for (unsigned int i = 0; i < vehicles_front_by_lane.size(); i++)
    {
        if (!vehicles_front_by_lane[i].is_default)
        {
            res[i] = getVehSpeedMph(vehicles_front_by_lane[i]);
        }
    }

    return res;
}

std::vector<FusionObjData> PathPlanner::getVehiclesFront(const std::vector<FusionData> &veh_in_lanes,
                                                        const double &s,
                                                        const double max_dist)
{
    std::vector<FusionObjData> ret = std::vector<FusionObjData>(veh_in_lanes.size(), FusionObjData());
    for (unsigned int i = 0; i < veh_in_lanes.size(); i++)
    {
        auto curr_lane = veh_in_lanes[i];
        double s_diff = 0;

        for (auto &veh : curr_lane)
        {
            s_diff = veh.s - s;
            if (s_diff >= 0 && s_diff < max_dist)
            {
                ret[i] = veh;
            }
        }
    }
    return ret;
}

Predictions PathPlanner::getTrajectoryPredictions(const FusionData &vehicles, int num_steps, MapData map_data)
{
    Predictions pred;
    for (auto const &veh : vehicles)
    {
        // predict a rough trajectory based on the current position and velocity
        Trajectory traj;
        double s = veh.s;
        double vel = getVehSpeedMs(veh);
        for (unsigned int i = 0; i < num_steps; i++)
        {
            s += vel * 0.02;
            vector<double> xy = getXY(s, veh.d, map_data.maps_s, map_data.maps_x, map_data.maps_y);
            traj.x.push_back(xy[0]);
            traj.y.push_back(xy[1]);
        }
        pred.push_back(traj);
    }
    return pred;
}

bool PathPlanner::checkIfLaneChangePossible(const VehicleData &veh_data,
                                            const FusionData &vehicles,
                                            int target_lane,
                                            double target_vel,
                                            const MapData &map_data)
{
    bool res = true;
    int num_predicted_steps = 100;

    Predictions predictions = getTrajectoryPredictions(vehicles, num_predicted_steps, map_data);

    // calculate planned ego trajectory
    // get start of planned ego trajectory from the end of the previously planned trajectory
    Trajectory planned_ego_traj;
    int prev_path_size = veh_data.previous_path_x.size();
    double ref_x = veh_data.previous_path_x[prev_path_size - 1];
    double ref_y = veh_data.previous_path_y[prev_path_size - 1];
    double ref_prev_x = veh_data.previous_path_x[prev_path_size - 2];
    double ref_prev_y = veh_data.previous_path_y[prev_path_size - 2];
    double ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);

    Trajectory rough_traj = getRoughTrajectoryStart(veh_data);

    // calculate waypoints 30, 60, and 90 (depending on current value of m_wp_offset) meters
    // ahead of the ego vehicle
    for (unsigned int i = 0; i < 3; i++)
    {
        vector<double> wp = getXY(veh_data.car_s + m_wp_offset * (i + 1),
                                    2 + target_lane * 4,
                                    map_data.maps_s,
                                    map_data.maps_x,
                                    map_data.maps_y);
        rough_traj.x.push_back(wp[0]);
        rough_traj.y.push_back(wp[1]);
    }

    Trajectory ego_traj = calculateTrajectory(rough_traj,
                                                ref_x,
                                                ref_y,
                                                ref_yaw,
                                                target_vel,
                                                num_predicted_steps,
                                                veh_data,
                                                map_data);

    for (auto const &pred_traj : predictions)
    {
        res = res && !trajToClose(pred_traj, ego_traj, 7.0);
    }

    return res;
}

Trajectory PathPlanner::getRoughTrajectoryStart(const VehicleData &veh_data)
{
    Trajectory traj;

    int prev_path_size = veh_data.previous_path_x.size();

    if (prev_path_size < 2)
    {
        double prev_car_x = veh_data.car_x - cos(veh_data.car_yaw);
        double prev_car_y = veh_data.car_y - sin(veh_data.car_yaw);

        traj.x.push_back(prev_car_x);
        traj.x.push_back(veh_data.car_x);

        traj.y.push_back(prev_car_y);
        traj.y.push_back(veh_data.car_y);
    }
    else
    {
        for (unsigned int i = 3; i >= 1; i--)
        {
            traj.x.push_back(veh_data.previous_path_x[prev_path_size - i]);
            traj.y.push_back(veh_data.previous_path_y[prev_path_size - i]);
        }
    }
    return traj;
}

double PathPlanner::setACCVel(vector<FusionData> fus_obj_by_lane,
                                int curr_lane,
                                double car_s,
                                double end_path_s)
{
    double target_vel = v_max;
    auto lane_speeds = getLaneSpeeds(fus_obj_by_lane, car_s);
    auto veh_front = getVehiclesFront(fus_obj_by_lane, car_s);
    if (!veh_front[curr_lane].is_default)
    {
        double s_diff = veh_front[curr_lane].s - end_path_s;
        std::cout << "s_diff: " << s_diff << "\n";
        if (s_diff < dist_margin)
        {
            target_vel = lane_speeds[curr_lane] + s_diff;
        }
    }
    target_vel = min(target_vel, v_max);
    return target_vel;
}

vector<int> PathPlanner::getFastestLane(vector<double> lane_speeds)
{
    vector<int> res;
    double max_speed = *std::max_element(lane_speeds.begin(), lane_speeds.end());
    for (unsigned int i = 0; i < lane_speeds.size(); i++)
    {
        if (lane_speeds[i] == max_speed)
        {
            res.push_back(i);
        }
    }
    return res;
}