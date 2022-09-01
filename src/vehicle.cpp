#include "vehicle.h"
#include <cmath>
#include <utility>

// Constructor
Vehicle::Vehicle() = default;

Vehicle::Vehicle(int lane, double s, double spd, string state) {
    this->lane = lane;
    this->s = s;
    this->speed = spd;
    this->state = std::move(state);
}

// Destructor
Vehicle::~Vehicle() = default;

vector<Vehicle> Vehicle::generate_predictions() {
    vector<Vehicle> predictions;

    for (int i = 0; i < 10; ++i) {
        double next_s = this->s + 0.02 * (double) i * this->speed;

        predictions.emplace_back(Vehicle(this->lane, next_s, this->speed, this->state));
    }

    return predictions;
}

// If state is either "LCL" or "LCR", then just return "KL"
vector<string> Vehicle::successor_states() {
    vector<string> states;

    states.emplace_back("KL");
    if (this->state == "KL") {
        states.emplace_back("LCL");
        states.emplace_back("LCR");
    }

    return states;
}

vector<Vehicle> Vehicle::choose_next_state(const map<int, vector<Vehicle>>& predictions) {
    vector<double> costs;
    vector<vector<Vehicle>> final_traj;
    vector<string> states = successor_states();

    for (auto &state: states) {
        vector<Vehicle> trajectory = generate_trajectory(state, predictions);
        if (!trajectory.empty()) {
            double cost = calculate_cost(trajectory);
            costs.push_back(cost);
            final_traj.push_back(trajectory);
        }
    }

    for (int i = 1; i < costs.size(); ++i) {
        if (fabs(costs[0] - costs[i]) < 0.001) {
            costs[0] = costs[0] * 1.5;
        }
    }

    auto best_cost = max_element(costs.begin(), costs.end());
    int best_cost_idx = distance(costs.begin(), best_cost);

    return final_traj[best_cost_idx];
}

vector<Vehicle> Vehicle::generate_trajectory(string state, const map<int, vector<Vehicle>> &predictions) {
    vector<Vehicle> trajectory;

    if (state == "KL") {
        trajectory = keep_lane_traj(predictions);
    } else if (state == "LCL" || state == "LCR") {
        trajectory = lane_change_traj(state, predictions);
    }

    return trajectory;
}


vector<Vehicle> Vehicle::lane_change_traj(string state, map<int, vector<Vehicle>> predictions) {
    int new_lane = this->lane + lane_direction[state];
    double allowable_dist = 20.0;
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;

    if (new_lane < 0 || new_lane > 2) {
        return trajectory;
    }

    for (auto &prediction: predictions) {
        next_lane_vehicle = prediction.second[0];

        if (fabs(next_lane_vehicle.s - this->s) < allowable_dist && next_lane_vehicle.lane == new_lane) {
            return trajectory;
        }
    }
    trajectory.emplace_back(Vehicle(this->lane, this->s, this->speed, this->state));
    double kinematics = get_kinematics(predictions, new_lane);
    trajectory.emplace_back(Vehicle(new_lane, this->s, kinematics, state));

    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_traj(map<int, vector<Vehicle>> predictions) {
    double kinematics = get_kinematics(predictions, this->lane);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->speed, this->state)};
    trajectory.emplace_back(Vehicle(this->lane, this->s, kinematics, "KL"));

    return trajectory;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle &rVehicle) {
    double min_s = 100000.0;
    double allowable_dist = 15.0;
    bool found_vehicle = false;
    Vehicle temp_vehicle;

    for (auto &prediction: predictions) {
        temp_vehicle = prediction.second[0];

        if (temp_vehicle.lane == lane && fabs(temp_vehicle.s - this->s) < allowable_dist && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

double Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    Vehicle vehicle_ahead;
    double max_allowable_spd;
    double max_speed = 49.5;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        max_allowable_spd = min(vehicle_ahead.speed, max_speed);
    } else {
        max_allowable_spd = max_speed;
    }

    return max_allowable_spd;
}

double Vehicle::calculate_cost(const vector<Vehicle> &trajectory) {
    Vehicle target = trajectory[1];

    return target.speed * target.speed;
}