#include <map>
#include "util.h"
#include <vector>
#include <tuple>
#include <array>
#include <omp.h>
#include <set>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
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
	auto condition = [](const std::array<int, 3>& o) { return o[0] != -1; };

	int numAvailable = std::count_if(vehicles.begin(), vehicles.end(), [](const Vehicle& v) {return v.available; });
	vector<int> availableIdxes(numAvailable, -1);
	int thisCounter = 0;
	for (int i = 0; i < vehicles.size(); i++) {
		if (vehicles[i].available) {
			availableIdxes[thisCounter++] = i;
		}
	}

	std::map<int, int> psgrCounter;
	std::map<int, int> waitingCounter;
	std::map<int, int> onBoardCounter;
	std::map<int, int> areaCounter;
	std::map<int, int> startOccupancy;
	std::map<int, int> endOccupancy;

	for (int i = 0; i < numAvailable; i++) {
		int thisIdx = availableIdxes[i];
		int agenda = vehicles[thisIdx].get_num_passengers();
		int thisArea = treeCost.node_areas[vehicles[thisIdx].get_location() - 1];
		int onBoardCnt = 0;
		int waitingCnt = 0;
		for (int j = 0; j < vehicles[thisIdx].passengers.size(); j++) {
			if (vehicles[thisIdx].passengers[j].status == Request::requestStatus::waiting) {
				waitingCnt++;
			}
			else if (vehicles[thisIdx].passengers[j].status == Request::requestStatus::onBoard) {
				onBoardCnt++;
			}
		}
		psgrCounter[agenda]++;
		waitingCounter[waitingCnt]++;
		onBoardCounter[onBoardCnt]++;
		areaCounter[thisArea]++;
		startOccupancy[vehicles[thisIdx].getOccupancyAt(now_time)]++;
		endOccupancy[vehicles[thisIdx].getOccupancyAt(now_time + 300)]++;
	}
	std::ofstream ofs;
	ofs.open(outDir + "VehicleStatuses/psgrCounter.csv", std::ofstream::out | std::ofstream::app);
	for (auto iter = psgrCounter.begin(); iter != psgrCounter.end(); iter++) {
		ofs << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "\n";
	}
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/waitingCounter.csv", std::ofstream::out | std::ofstream::app);
	for (auto iter = waitingCounter.begin(); iter != waitingCounter.end(); iter++) {
		ofs << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "\n";
	}
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/onBoardCounter.csv", std::ofstream::out | std::ofstream::app);
	for (auto iter = onBoardCounter.begin(); iter != onBoardCounter.end(); iter++) {
		ofs << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "\n";
	}
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/areaCounter.csv", std::ofstream::out | std::ofstream::app);
	for (auto iter = areaCounter.begin(); iter != areaCounter.end(); iter++) {
		ofs << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "\n";
	}
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/startOccupancy.csv", std::ofstream::out | std::ofstream::app);
	for (auto iter = startOccupancy.begin(); iter != startOccupancy.end(); iter++) {
		ofs << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "\n";
	}
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/endOccupancy.csv", std::ofstream::out | std::ofstream::app);
	for (auto iter = endOccupancy.begin(); iter != endOccupancy.end(); iter++) {
		ofs << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "\n";
	}
	ofs.close();

	int pre_size = numAvailable * nReq;
	print_line(outDir, logFile, string_format("RV step: %d cars, %d reqs ", numAvailable, nReq));
	std::vector<std::tuple<int, int, int>> cost_r_v(numAvailable * nReq, std::tuple<int, int, int>(-1, -1, -1));
#pragma omp parallel for default(none) num_threads(omp_get_max_threads()) private(j) schedule(static) shared(availableIdxes, numAvailable, cost_r_v, vehicles, condition, outDir, logFile, requests, treeCost)
	for (j = 0; j < nReq; j++) {
		int lowestDist = -1;
		int closestNode = -1;
		Request thisReq = requests[j];
		Request* reqs[1];
		reqs[0] = &thisReq;
		TravelHelper th;
		for (int i = 0; i < numAvailable; i++) {
			int thisIdx = availableIdxes[i];
			int cost = th.travel(vehicles[thisIdx], reqs, 1, false);
			if (cost >= 0) {
				if (0 != lowestDist) {
					if (vehicles[thisIdx].getAvailableSince() < 0) {
						lowestDist = 0;
					}
					else {
						int dist = treeCost.get_dist(vehicles[thisIdx].get_location(), thisReq.start).second;
						if (lowestDist < 0 || dist < lowestDist) {
							lowestDist = dist;
							closestNode = vehicles[thisIdx].get_location();
						}
					}
				}
				int thisIdxInVRC = i * nReq + j;
				std::get<0>(cost_r_v[thisIdxInVRC]) = cost;
				std::get<1>(cost_r_v[thisIdxInVRC]) = j;
				std::get<2>(cost_r_v[thisIdxInVRC]) = thisIdx;
			}
		}
		if (lowestDist >= 0) {
			req_nearest_car_node[j] = closestNode;
		}
	}
	cost_r_v.erase(std::remove_if(cost_r_v.begin(), cost_r_v.end(),
		[](const std::tuple<int, int, int>& o) { return std::get<0>(o) == -1; }),
		cost_r_v.end());

	int pre_size_2 = cost_r_v.size();
	cost_r_v.shrink_to_fit();
	std::sort(cost_r_v.begin(), cost_r_v.end());

	std::vector<int> rAssignments(nReq, -1);
	std::vector<int> vAssignments(nVeh, -1);
	std::vector<int> rAssignmentCounts(nReq, 0);
	std::vector<int> vAssignmentCounts(nVeh, 0);
	int assignableCnt = 0;
	for (int i = 0; i < cost_r_v.size(); i++) {
		int rIdx = std::get<1>(cost_r_v[i]);
		int vIdx = std::get<2>(cost_r_v[i]);
		if (vAssignmentCounts[vIdx] >= min_req_per_v && rAssignmentCounts[rIdx] >= max_v_per_req && (-1 != vAssignments[vIdx] || -1 != rAssignments[rIdx])) {
			std::get<0>(cost_r_v[i]) = -1;
			continue;
		}
		vAssignmentCounts[vIdx]++;
		rAssignmentCounts[rIdx]++;
		if (-1 == vAssignments[vIdx] && -1 == rAssignments[rIdx]) {
			vAssignments[vIdx] = rIdx;
			rAssignments[rIdx] = vIdx;
			assignableCnt++;
		}
	}
	std::vector<int>().swap(rAssignments);
	std::vector<int>().swap(vAssignments);
	cost_r_v.erase(std::remove_if(cost_r_v.begin(), cost_r_v.end(),
		[](const std::tuple<int, int, int>& o) { return std::get<0>(o) == -1; }),
		cost_r_v.end());

	cost_r_v.shrink_to_fit();
	std::sort(cost_r_v.begin(), cost_r_v.end(),
		[](const std::tuple<int, int, int>& a, const std::tuple<int, int, int>& b) {
			return std::get<1>(a) < std::get<1>(b) || (std::get<1>(a) == std::get<1>(b) && std::get<2>(a) < std::get<2>(b));
		}); //orders ascending by req then veh

	int post_size = cost_r_v.size();
	print_line(outDir, logFile, string_format("RV graph shrunk from %d to %d to %d.", pre_size, pre_size_2, post_size));

	for (int i = 0; i < vAssignmentCounts.size(); i++) {
		if (vAssignmentCounts[i] > 0) {
			car_req_cost[i].reserve(vAssignmentCounts[i]);
		}
	}
	for (int i = 0; i < cost_r_v.size(); i++) {
		car_req_cost[std::get<2>(cost_r_v[i])].push_back(make_pair(std::get<1>(cost_r_v[i]), std::get<0>(cost_r_v[i]))); //vehicle, req, cost
	}
}

bool RVGraph::has_vehicle(int vehicle) {
	return (car_req_cost.find(vehicle) != car_req_cost.end());
}

int RVGraph::get_vehicle_num() {
	return int(car_req_cost.size());
}
