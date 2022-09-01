#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:
    map<string, int> lane_direction = {{"LCL", -1},
                                       {"LCR", 1}};

    int lane{0};

    double s{0.0};

    double speed{0.0};

    string state{"KL"};

    // Constructor
    Vehicle();
    Vehicle(int lane, double s, double speed, string state = "KL");

    // Destructor
    virtual ~Vehicle();

    vector<Vehicle> generate_predictions();
    vector<string> successor_states();
    vector<Vehicle> choose_next_state(const map<int, vector<Vehicle>>& predictions);
    vector<Vehicle> generate_trajectory(string state, const map<int, vector<Vehicle>> &predictions);
    vector<Vehicle> lane_change_traj(string state, map<int, vector<Vehicle>> predictions);
    vector<Vehicle> keep_lane_traj(map<int, vector<Vehicle>> predictions);
    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle &rVehicle);
    double get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
    double calculate_cost(const vector<Vehicle> &trajectory);
};

#endif //VEHICLE_H
