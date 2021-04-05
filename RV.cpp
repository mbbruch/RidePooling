#include <map>
#include "util.h"
#include <vector>
#include <omp.h>
#include <set>
#include <algorithm>
#include "util.h"
#include "travel.h"
#include "globals.h"
#include "RV.h"
using namespace std;

void RVGraph::add_edge_vehicle_req(int vehicle, int req, int cost) {
    car_req_cost[vehicle][req] = cost;
    req_cost_car[req].push_back(make_pair(cost, vehicle));
    req_car.insert(make_pair(req, vehicle));
    entries++;
}

void RVGraph::prune() {
    for (auto it = req_cost_car.begin(); it != req_cost_car.end(); it++) {
        sort(it->second.begin(), it->second.end());
        if (it->second.size() <= max_v_per_req) {
            continue;
        }
        for (int idx = max_v_per_req; idx < it->second.size(); idx++) {
            int vId = it->second[idx].second;
            car_req_cost[vId].erase(it->first);
            if (car_req_cost[vId].empty()) {
                car_req_cost.erase(vId);
            }
        }
    }
}

RVGraph::RVGraph(vector<Vehicle>& vehicles, vector<Request>& requests, map_of_pairs& dist) {
    /* Get arc costs and add arcs for request-vehicle matches*/
    int connected = 0;
    int disconnected = 0;
    int i = 0;
	entries = 0;
    const int nVeh = vehicles.size();
    const int nReq = requests.size();
    #pragma omp parallel for default(none) private(i) shared(vehicles, requests, dist)
    for (i = 0; i < nVeh; i++) {
        if (vehicles[i].isAvailable()) {
            for (int j = 0; j < nReq; j++) {
                Request thisReq = requests[j];
				Request* reqs[1];
                reqs[0] = &thisReq;
                TravelHelper th;
                int cost = th.travel(vehicles[i], reqs, 1, dist, false);
                if (cost >= 0) {
                    #pragma omp critical (addevr)
                    add_edge_vehicle_req(i, j, th.getTravelCost());
                }
            }
        }
    }
    req_car.rehash(entries);
    /*
    map<bool, int> carCounts;
    for (auto it = carConnections.begin(); it != carConnections.end(); it++) {
        carCounts[it->second > 0]++;
    }
   TODO fix map<bool, int> requestsCounts;
    for (auto it = requestConnections.begin(); it != requestConnections.end(); it++) {
        requestsCounts[it->second > 0]++;
    }

    printf("Connected requests: %d, disconnected: %d\n", requestsCounts[true], requestsCounts[false]);
    printf("Connected cars: %d, disconnected: %d\n", carCounts[true], carCounts[false]);
    disconnectedCars = carCounts[false];
*/
    // prune(); TODO "spread out" pruned trips better so that if vehicles all start at the same place, the same cars don't all get "kept"
}

bool RVGraph::has_vehicle(int vehicle) {
    return (car_req_cost.find(vehicle) != car_req_cost.end());
}

int RVGraph::get_vehicle_num() {
    return int(car_req_cost.size());
}
