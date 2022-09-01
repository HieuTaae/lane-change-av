#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <uWS/uWS.h>

#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

using nlohmann::json;
using std::string;
using std::vector;

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        iss >> x;
        iss >> y;
        iss >> s;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
    }

    double ref_speed{0.0};
    int lane{1};

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &ref_speed, &lane]
                        (uWS::WebSocket<uWS::SERVER> ws, const char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (!s.empty()) {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_yaw = j[1]["yaw"];
                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    double end_path_s = j[1]["end_path_s"];
                    // Sensor fusion data of surrounding agents
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    int prev_size = previous_path_x.size();
                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    // Predicts the agents trajectory using sensor fusion data
                    map<int, vector<Vehicle>> predictions;
                    for (auto &i: sensor_fusion) {
                        int v_id = i[0];
                        int v_lane;
                        // Predicts only for agents on the same direction as ego
                        if (i[6] >= 0 && i[6] <= 12) {
                            v_lane = (int) i[6] / 4;
                        } else {
                            continue;
                        }
                        double vx = i[3];
                        double vy = i[4];
                        double v_speed = sqrt(vx * vx + vy * vy);
                        double v_s = i[5];
                        Vehicle agent(v_lane, v_s, v_speed, "KL");
                        vector<Vehicle> preds = agent.generate_predictions();
                        predictions[v_id] = preds;
                    }

                    Vehicle ego(lane, car_s, ref_speed, "KL");
                    vector<Vehicle> trajectory = ego.choose_next_state(predictions);
                    lane = trajectory[1].lane;

                    if (ref_speed < trajectory[1].speed) {
                        ref_speed += 0.224;
                    } else if (ref_speed > trajectory[1].speed) {
                        ref_speed -= 0.224;
                    }

                    // Heading waypoints in vehicle frame
                    vector<double> ptsx;
                    vector<double> ptsy;

                    double start_x = car_x;
                    double start_y = car_y;
                    double start_yaw = car_yaw;

                    if (prev_size < 2) {
                        double prev_x = start_x - cos(start_yaw);
                        double prev_y = start_y - sin(start_yaw);

                        ptsx.push_back(prev_x);
                        ptsx.push_back(start_x);

                        ptsy.push_back(prev_y);
                        ptsy.push_back(start_y);
                    } else {
                        double prev_x = previous_path_x[prev_size - 2];
                        double prev_y = previous_path_y[prev_size - 2];

                        start_x = previous_path_x[prev_size - 1];
                        start_y = previous_path_y[prev_size - 1];
                        start_yaw = atan2(start_y - prev_y, start_x - prev_x);

                        ptsx.push_back(prev_x);
                        ptsx.push_back(start_x);

                        ptsy.push_back(prev_y);
                        ptsy.push_back(start_y);
                    }

                    for (int i = 0; i < 3; ++i) {
                        double next_s = car_s + 30 * (i + 1);
                        int next_lane = 2 + 4 * lane;
                        vector<double> next_wp = getXY(next_s, next_lane, map_waypoints_s, map_waypoints_x,
                                                       map_waypoints_y);

                        ptsx.push_back(next_wp[0]);
                        ptsy.push_back(next_wp[1]);
                    }

                    for (int i = 0; i < ptsx.size(); ++i) {
                        double diff_x = ptsx[i] - start_x;
                        double diff_y = ptsy[i] - start_y;

                        ptsx[i] = diff_x * cos(-start_yaw) - diff_y * sin(-start_yaw);
                        ptsy[i] = diff_x * sin(-start_yaw) + diff_y * cos(-start_yaw);
                    }

                    json msgJson;

                    // Trajectory generated using initial and
                    // target trajectory from the behavior planner
                    // Waypoints are transformed back to world frame
                    tk::spline traj;
                    traj.set_points(ptsx, ptsy);

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int i = 0; i < prev_size; ++i) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    double end_x = 30.0;
                    double end_y = traj(end_x);
                    double dist = sqrt(end_x * end_x + end_y * end_y);
                    double curr = 0;

                    for (int i = 0; i < 50 - prev_size; ++i) {
                        double step = dist / (0.02 * ref_speed / 2.24);
                        double pnt_x = curr + end_x / step;
                        double pnt_y = traj(pnt_x);

                        curr = pnt_x;

                        double tmp_x = pnt_x;
                        double tmp_y = pnt_y;

                        pnt_x = tmp_x * cos(start_yaw) - tmp_y * sin(start_yaw);
                        pnt_y = tmp_x * sin(start_yaw) + tmp_y * cos(start_yaw);

                        pnt_x += start_x;
                        pnt_y += start_y;

                        next_x_vals.push_back(pnt_x);
                        next_y_vals.push_back(pnt_y);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}