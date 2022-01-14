#include <map>
#include "util.h"
#include <vector>
#include <array>
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
	std::vector<std::array<int,3>> v_r_cost(nVeh*nReq, std::array<int,3>{{-1, -1, -1}});
    #pragma omp parallel for default(none) num_threads(omp_get_max_threads()) private(j) schedule(static) shared(v_r_cost, vehicles, outDir, logFile, requests, treeCost)
    for (j = 0; j < nReq; j++) {
        int lowestDist = -1;
        int closestNode = -1;
        Request thisReq = requests[j];
        Request* reqs[1];
        reqs[0] = &thisReq;
		TravelHelper th;
        for (int i = 0; i < nVeh; i++) {
            if (vehicles[i].isAvailable()) {
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
					int thisIdx = i*nReq+j;
                    v_r_cost[thisIdx][0] = i;
                    v_r_cost[thisIdx][1] = j;
                    v_r_cost[thisIdx][2] = cost;
                }
            }
        }
        if (lowestDist > 0) {
            req_nearest_car_node[j] = closestNode;
        }
    }
	
        v_r_cost.erase(std::remove_if(v_r_cost.begin(), v_r_cost.end(),
		[](const std::array<int,3>& o) { return o[0] == -1; }),
            v_r_cost.end());
			
    std::vector<int>sizes(nVeh, 0);
    for (int i = 0; i < v_r_cost.size(); i++) {
        sizes[v_r_cost[i][0]]++;
    }
	
    for (int i = 0; i < sizes.size(); i++) {
        if (sizes[i] > 0) {
			car_req_cost[i].reserve(sizes[i]);
		}
    }

	print_line(outDir, logFile, std::string("RV: filling in map"));
    for (int i = 0; i < v_r_cost.size(); i++) {
        car_req_cost[v_r_cost[i][0]].push_back(make_pair(v_r_cost[i][1],v_r_cost[i][2]));
    }
    // prune(); TODO "spread out" pruned trips better so that if vehicles all start at the same place, the same cars don't all get "kept"
}

bool RVGraph::has_vehicle(int vehicle) {
    return (car_req_cost.find(vehicle) != car_req_cost.end());
}

int RVGraph::get_vehicle_num() {
    return int(car_req_cost.size());
}
