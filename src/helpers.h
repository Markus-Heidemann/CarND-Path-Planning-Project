#pragma once

#include "stdafx.h"
#include "spline.h"

using namespace std;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double INT_INF = std::numeric_limits<int>::infinity();
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

struct MapData
{
    vector<double> maps_x;
    vector<double> maps_y;
    vector<double> maps_s;
    vector<double> maps_dx;
    vector<double> maps_dy;

    MapData(vector<double> maps_x,
            vector<double> maps_y,
            vector<double> maps_s,
            vector<double> maps_dx,
            vector<double> maps_dy)
        : maps_x(maps_x),
          maps_y(maps_y),
          maps_s(maps_s),
          maps_dx(maps_dx),
          maps_dy(maps_dy)
    {
    }
};

struct FusionObjData
{
    //[ id, x, y, vx, vy, s, d]
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;

    bool is_default;

    FusionObjData() : is_default(true) {}
};

typedef std::vector<FusionObjData> FusionData;

inline void from_json(const nlohmann::json &j, FusionData &fus_data)
{
    int fus_data_size = j.size();
    for (unsigned int i = 0; i < fus_data_size; ++i)
    {
        FusionObjData fus_obj;
        fus_obj.id = j[i][0];
        fus_obj.x = j[i][1];
        fus_obj.y = j[i][2];
        fus_obj.vx = j[i][3];
        fus_obj.vy = j[i][4];
        fus_obj.s = j[i][5];
        fus_obj.d = j[i][6];
        fus_obj.is_default = false;
        fus_data.push_back(fus_obj);
    }
}

struct VehicleData
{
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    // Previous path data given to the Planner
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    // Previous path's end s and d values
    double end_path_s;
    double end_path_d;

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    FusionData sensor_fusion;
};

struct Trajectory
{
    vector<double> x;
    vector<double> y;
};

typedef vector<Trajectory> Predictions;

inline void from_json(const nlohmann::json &j, VehicleData &veh_data)
{
    // Ego car's localization Data
    veh_data.car_x = j["x"];
    veh_data.car_y = j["y"];
    veh_data.car_s = j["s"];
    veh_data.car_d = j["d"];
    veh_data.car_yaw = j["yaw"];
    veh_data.car_speed = j["speed"];

    // Previous path data given to the Planner
    int prev_path_size = j["previous_path_x"].size();
    for (unsigned int i = 0; i < prev_path_size; ++i)
    {
        veh_data.previous_path_x.push_back(j["previous_path_x"][i]);
        veh_data.previous_path_y.push_back(j["previous_path_y"][i]);
    }
    // Previous path's end s and d values
    veh_data.end_path_s = j["end_path_s"];
    veh_data.end_path_d = j["end_path_d"];

    veh_data.sensor_fusion = j["sensor_fusion"].get<FusionData>();
}

inline double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

inline int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = min(2 * pi() - angle, angle);

    if (angle > pi() / 2)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline vector<double> getFrenet(double x,
                                double y,
                                double theta,
                                const vector<double> &maps_x,
                                const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline vector<double> getXY(double s,
                            double d,
                            const vector<double> &maps_s,
                            const vector<double> &maps_x,
                            const vector<double> &maps_y)
{
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

/*
 * Get's the lane index of the current lane based on the Frenet's d value
 */
inline int lane_from_d(double d, int num_lanes = 3, double lane_width = 4.0)
{
    int res = -1;
    for (unsigned int lane_idx = 0; lane_idx < num_lanes; lane_idx++)
    {
        if ((d < ((lane_idx + 1) * lane_width)) and (d >= (lane_idx * lane_width)))
        {
            res = lane_idx;
        }
    }
    return res;
}

inline double getVehSpeedMph(FusionObjData veh)
{
    return sqrt(veh.vx * veh.vx + veh.vy * veh.vy) * 2.24;
}

inline double getVehSpeedMs(FusionObjData veh)
{
    return sqrt(veh.vx * veh.vx + veh.vy * veh.vy);
}

inline bool trajToClose(Trajectory t1, Trajectory t2, double thresh)
{
    bool res = false;
    int min_size = min(t1.x.size(), t2.x.size());
    for (unsigned int i = 0; i < min_size; i++)
    {
        if (thresh > distance(t1.x[i], t1.y[i], t2.x[i], t2.y[i]))
        {
            res = true;
        }
    }
    return res;
}

/*
 * NOT USED IN THE CURRENT IMPLEMENTATION
 */
// TODO make faster by returning look-up table of arc length, so that
// not all the values have to be recalculated in each iteration on path planner
inline double x_for_arc_length(const tk::spline &s, double arc_length, double x_inc)
{
    double res = 0;

    double cumm_arc_length = 0;
    double curr_x = x_inc;
    double prev_x = 0;
    double prev_cumm_arc_length = 0;

    while (cumm_arc_length < arc_length)
    {
        double prev_y = s(prev_x);
        double curr_y = s(curr_x);
        cumm_arc_length += distance(prev_x, prev_y, curr_x, curr_y);
        prev_x = curr_x;
        prev_cumm_arc_length = cumm_arc_length;
        curr_x += x_inc;
    }

    if (abs(prev_cumm_arc_length - arc_length) < abs(cumm_arc_length - arc_length))
    {
        res = prev_x;
    }
    else
    {
        res = curr_x;
    }
    return res;
}