#include <map>
#include "util.h"
#include <vector>
#include <omp.h>
#include <set>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include "hps_src/hps.h"
#include "util.h"
#include "travel.h"
#include "globals.h"
#include "RV.h"
#include "GPtree.h"
using namespace std;

RVGraph::RVGraph(vector<Vehicle>& vehicles, vector<Request>& requests) {
    /* Get arc costs and add arcs for request-vehicle matches*/
    int connected = 0;
    int disconnected = 0;
    int j = 0;
    const int nVeh = vehicles.size();
    const int nReq = requests.size();
    req_nearest_car_node = std::vector<int>(nReq, -1);
    std::vector<std::pair<int, std::pair<int, int>>> all_v_r_cost;
    //#pragma omp parallel for default(none) num_threads(omp_get_max_threads()) private(j) shared(vehicles,  nReq, nVeh, requests, treeCost)
    for (j = 0; j < nReq; j++) {
        std::vector<std::pair<int, std::pair<int, int>>> v_r_cost(nVeh,make_pair(-1,make_pair(j,-1)));
        int lowestDist = -1;
        int closestNode = -1;
        Request thisReq = requests[j];
        Request* reqs[1];
        reqs[0] = &thisReq;
        for (int i = 0; i < nVeh; i++) {
            if (vehicles[i].isAvailable()) {
                TravelHelper th;
                int cost = th.travel(vehicles[i], reqs, 1, false);
                if (cost >= 0) {
                    if (0 != lowestDist) {
                        if (vehicles[i].getAvailableSince() < 0) {
                            lowestDist = 0;
                        }
                        else {
                            int dist = treeCost.get_dist(vehicles[i].get_location(), thisReq.start).second;
                            if (lowestDist < 0 || dist < lowestDist) {
                                lowestDist = dist;
                                closestNode = vehicles[i].get_location();
                            }
                        }
                    }
                    v_r_cost[i].first = i;
                    v_r_cost[i].second.first = j;
                    v_r_cost[i].second.second = cost;
                }
                else {
                    int x = 5;
                }
            }
        }
        if (lowestDist > 0) {
            req_nearest_car_node[j] = closestNode;
        }
        v_r_cost.erase(std::remove_if(v_r_cost.begin(), v_r_cost.end(),
            [](const std::pair<int, std::pair<int, int>>& o) { return o.first == -1; }),
            v_r_cost.end());
        #pragma omp critical (addevr)
        all_v_r_cost.insert(all_v_r_cost.end(), v_r_cost.begin(), v_r_cost.end());
    }
    std::vector<int>sizes(nVeh, 0);
    for (int i = 0; i < all_v_r_cost.size(); i++) {
        sizes[all_v_r_cost[i].first]++;
    }
    for (int i = 0; i < sizes.size(); i++) {
        if (sizes[i] > 0) car_req_cost[i].reserve(sizes[i]);
    }

    for (int i = 0; i < all_v_r_cost.size(); i++) {
        car_req_cost[all_v_r_cost[i].first].push_back(all_v_r_cost[i].second);
    }
    // prune(); TODO "spread out" pruned trips better so that if vehicles all start at the same place, the same cars don't all get "kept"
}

bool RVGraph::has_vehicle(int vehicle) {
    return (car_req_cost.find(vehicle) != car_req_cost.end());
}

int RVGraph::get_vehicle_num() {
    return int(car_req_cost.size());
}
