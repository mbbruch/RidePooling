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

void RVGraph::add_edge_vehicle_req(int vehicle, int req, int cost) {
    car_req_cost[vehicle][req] = cost;
    entries++;
}

RVGraph::RVGraph(vector<Vehicle>& vehicles, vector<Request>& requests) {
    /* Get arc costs and add arcs for request-vehicle matches*/
    int connected = 0;
    int disconnected = 0;
    int j = 0;
    entries = 0;
    const int nVeh = vehicles.size();
    const int nReq = requests.size();
   // #pragma omp parallel for default(none) private(j) shared(vehicles, requests, treeCost)
    for (j = 0; j < nReq; j++) {
        int lowestCost = -1;
        int closestNode = -1;
        Request thisReq = requests[j];
        Request* reqs[1];
        reqs[0] = &thisReq;
        for (int i = 0; i < nVeh; i++) {
            if (vehicles[i].isAvailable()) {
                Vehicle vehicleCopy = vehicles[i];
                TravelHelper th;
                int cost = th.travel(vehicleCopy, reqs, 1, false);
                if (cost >= 0) {
                    if (vehicles[i].getAvailableSince() < 0) {
                        lowestCost = 0;
                    }
                    else if(0!= lowestCost){
                        cost = treeCost.get_dist(vehicleCopy.get_location(), thisReq.start).first;
                        if (lowestCost < 0 || cost < lowestCost) {
                            lowestCost = cost;
                            closestNode = vehicleCopy.get_location();
                        }
                    }
                    #pragma omp critical (addevr)
                    add_edge_vehicle_req(i, j, th.getTravelCost());
                }
            }
        }
        if (lowestCost > 0) {
            #pragma omp critical (addClosestNode)
            req_nearest_car_node.insert(make_pair(j, closestNode));
        }
    }

    req_nearest_car_node.rehash(req_nearest_car_node.size());
    // prune(); TODO "spread out" pruned trips better so that if vehicles all start at the same place, the same cars don't all get "kept"
}

bool RVGraph::has_vehicle(int vehicle) {
    return (car_req_cost.find(vehicle) != car_req_cost.end());
}

int RVGraph::get_vehicle_num() {
    return int(car_req_cost.size());
}
