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

    void add_edge_vehicle_req(int vehicle, int req, int cost);

    void prune();

public:
    map<int, map<int, int> > car_req_cost;
    map<int, vector<pair<int, int> > > req_cost_car;
    unordered_map<int, int> req_nearest_car_node;
    set_of_pairs req_car;
    int entries;
    RVGraph(vector<Vehicle>& vehicles, vector<Request>& requests);

    bool has_vehicle(int vehicle);

    int get_vehicle_num();

    const map<int,int>& get_vehicle_edges(int vehicle) const { return car_req_cost.find(vehicle)->second; }
};
