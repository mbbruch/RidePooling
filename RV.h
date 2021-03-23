#pragma once

#include <map>
#include "util.h"
#include <vector>
#include "Vehicle.h"
#include "Request.h"

using namespace std;

class RVGraph {

    void add_edge_vehicle_req(int vehicle, int req, int cost);

    void prune();

public:
    map<int, map<int, int> > car_req_cost;
    map<int, vector<pair<int, int> > > req_cost_car;
    RVGraph(vector<Vehicle>& vehicles, vector<Request>& requests,
        map_of_pairs& dist);

    bool has_vehicle(int vehicle);

    int get_vehicle_num();

    const map<int,int>& get_vehicle_edges(int vehicle) const { return car_req_cost.find(vehicle)->second; }
};
