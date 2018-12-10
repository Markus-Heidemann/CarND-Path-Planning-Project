#pragma once

#include "stdafx.h"

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

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

    // void from_json(const nlohmann::json& j, FusionData& fus) {
    //     fus.id = j.at("test").get<A>();
    // }
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

inline void from_json(const nlohmann::json &j, VehicleData &veh_data)
{
    // Main car's localization Data
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