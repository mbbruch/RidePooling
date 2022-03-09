#pragma once

#include <map>
#include <unordered_map>
#include "util.h"
#include <vector>
#include "Vehicle.h"
#include "Request.h"
#include "globals.h"

using namespace std;

class RVGraph {
    void prune();

public:
    std::unordered_map<int, std::vector<std::pair<int, int> >> car_req_cost;
    std::vector<int> req_nearest_car_node;
    RVGraph(vector<Vehicle>& vehicles, vector<Request>& requests);

    bool has_vehicle(int vehicle);

    int get_vehicle_num();

    const vector<std::pair<int,int>>& get_vehicle_edges(int vehicle) const { return car_req_cost.find(vehicle)->second; }
};
