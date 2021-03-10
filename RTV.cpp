#include <map>
#include <unordered_set>
#include "util.h"
#include <algorithm>
#include <string>
#include <cstdio>
#include "util.h"
#include <ctime>
#include <cmath>
#include <chrono>
#include <omp.h>
#include <random>
#include "fusion.h"
#include "gurobi_c++.h"
#include "globals.h"
#include "RTV.h"
#include "RV.h"
#include "GPtree.h"
#include "travel.h"
#include "util.h"
using namespace std;

RTVGraph* RTVGraph::TIdxComparable::rtvGraph;

bool equal_to_sub(vector<int>& compared, vector<int>& origin, int excludeIdx) {
    auto iterCompared = compared.begin();
    auto iterOrigin = origin.begin();
    int originIdx = 0;
    while (iterCompared != compared.end() && iterOrigin != origin.end()) {
        if (originIdx == excludeIdx) {
            iterOrigin++;
            originIdx++;
            continue;
        }
        if (*iterCompared != *iterOrigin) {
            return false;
        }
        iterCompared++;
        iterOrigin++;
        originIdx++;
    }
    return true;
}

int RTVGraph::addVehicleId(int vehicleId) {
        vIds.push_back(vehicleId);
        vIdx_tIdxes.push_back(vector<pair<int, pair<int,int>> >());
        return numVehicles++;
}

void RTVGraph::add_edge_trip_vehicle(uos& reqsInTrip, int vIdx, int cost) {
	int tIdx;
	#pragma omp critical (addetv1)
	tIdx = getTIdx(reqsInTrip);
	TIdxComparable tIdxComparable(tIdx);
/*	tIdx_vCostIdxes[tIdxComparable].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{0, vIdx}));
	vIdx_tIdxes[vIdx].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{0, tIdx})); */
	#pragma omp critical (addetv2)
	tIdx_vCostIdxes[tIdxComparable].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfCars(gen1), vIdx}));
	#pragma omp critical (addetv3)
	vIdx_tIdxes[vIdx].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfTrips(gen2), tIdx}));
}

bool RTVGraph::TIdxComparable::operator<(const TIdxComparable& other) const {
	int size1 = rtvGraph->trips[tIdx].size();
	int size2 = rtvGraph->trips[other.tIdx].size();
	if (size1 > size2) {
		return true;
	}
	else if (size1 == size2) {
		return tIdx < other.tIdx;
	}
	else {
		return false;
	}
}


int RTVGraph::getTIdx(const uos& trip) {
//#pragma omp critical (addetv1)
//    {
    auto iter = trip_tIdx.find(trip);
    if (iter != trip_tIdx.end()) {
        return iter->second;
    }
    trip_tIdx[trip] = numTrips;
    trips.push_back(trip);
    auto iterRIdx = trip.begin();
    while (iterRIdx != trip.end()) {
        int rId = *iterRIdx;
        if (rId_tIdxes.find(rId) == rId_tIdxes.end()) {
            rId_tIdxes[rId] = set<int>();
        }
        rId_tIdxes[rId].insert(numTrips);
        iterRIdx++;
    }
    return numTrips++;
//    }
}

void RTVGraph::build_potential_trips(RVGraph* rvGraph, vector<Request>& requests, map_of_pairs& dist) {

    vector<vector<tripCandidate>> thesePotentialTrips(allPotentialTrips);
    //Enumerate trips of size=1
	thesePotentialTrips[0].reserve(requests.size());
    for (int i = 0; i < requests.size(); i++) {
        thesePotentialTrips[0].push_back(tripCandidate(uos{ i }));
    }

	auto time2 = std::chrono::system_clock::now();
    int travelCounter = 0;
    int lastSizeSize = requests.size();
    int test1counter = 0, test2counter = 0, test3counter = 0;
    for (int k = 2; k <= max_trip_size; k++) {
        /*  Ex: to combine 4 requests, previous entry is 3-way combos,
        of which we need at least 4: 1-2-3, 1-2-4, 1-3-4, 2-3-4 */
        if (lastSizeSize < k) break;
		auto startOfSizeK = std::chrono::system_clock::now(); 
        map_of_uos newTrips; //key: set of requests (union of two trips); 
                                                     //value: indices within thesePotentialTrips[k-2]
        std::vector<tripCandidate> smallerTrips(thesePotentialTrips[k - 2].begin(), thesePotentialTrips[k - 2].end());

        if (k == 2) {
			std::vector<pair<uos,pair<int, uos>>> allCombosOfTwo;
			allCombosOfTwo.reserve(lastSizeSize*(lastSizeSize-1)/2);
            for (int i = 0; i < lastSizeSize; i++) {
                for (int j = i + 1; j < lastSizeSize; j++) {
					allCombosOfTwo.push_back(pair<uos,pair<int, uos>>{uos{ i, j },pair<int, uos>{1,uos{i,j}}});
                }
            }
			newTrips.insert(allCombosOfTwo.begin(), allCombosOfTwo.end());
        }
        else {
            /*
            When generating trips of size 3:
            Only need to look for pairwise combos of size=2 trips that have overlap = 1 request.
            SO, they need to have one shared dependency (trip of size=1).
            Can iterate over trips of size 1 where dependentTrips.size()>=2, and get all the pairwise combos of size=2 trips from that.
            (Should be no duplicates?? 99% sure but should sketch it out maybe)
            How many times? They have to have TWO pairwise combos (of trips of size 2) with overlap 1.
            (eg: 12, 23, 13)

            When generating trips of size 4:
            Only need to look for pairwise combos of size=3 trips that have overlap = 2 requests.
            SO, they need to have one shared dependency (trip of size = 2).
            Can iterate over trips of size 2 where dependentTrips.size()>=2, and get all the pairwise combos of size=3 trips from that.
			How many times? They need to have FOUR (k) pairwise combos of trips of size 3 (k-1) with overlap 2 (k-2).
			eg: (12, 23, 13) or (123, 124, 134, 234) or (1234, 1235, 1245, 1345, 2345) or (12345, 12346, 12356, 12456, 13456, 23456)
			k=3: 1 has to be in 2
			k=4: 12 & 23 & 13 & 14 & 24 & 34 have to be in 2, 1 & 2 & 3 have to each be in 3, 2 has to be in 3
			k=5: 123 has to be in 2, 12 has to be in 3, 1 has to be in 4
            */
            vector<tripCandidate> twoSmaller(thesePotentialTrips[k - 3].begin(), thesePotentialTrips[k - 3].end());
            const int twoSmallerSize = twoSmaller.size();
            int m;
            #pragma omp parallel for private(m) shared(twoSmaller, k, smallerTrips, newTrips)
            for (m = 0; m < twoSmallerSize; m++) {
                const tripCandidate& dependency = twoSmaller[m];
                const vector<int>& dependentTrips = dependency.dependentTrips;
                int numDependentTrips = dependentTrips.size();
                if (numDependentTrips < 2) continue;
                for (int i = 0; i < numDependentTrips; i++) {
                    int indexI = dependentTrips[i];
                    const uos& trip1 = smallerTrips[indexI].requests;

                    //uos2 tripUnion(trip1.begin(), trip1.end());
                    for (int j = i + 1; j < numDependentTrips; j++) {
                        int indexJ = dependentTrips[j];
                        const uos& trip2 = smallerTrips[indexJ].requests;
                        uos tripUnion(trip1);
                        for (auto it = trip2.begin(); it != trip2.end(); it++) {
                            if (tripUnion.emplace(*it).second) break;
                        } 
                        map_of_uos::iterator it2;
                        #pragma omp critical (updateNewTrips)
                        {
                        auto& it = newTrips[tripUnion];
                        it.first++;
                        it.second.insert(indexI);
                        it.second.insert(indexJ);
                        }
                    }
                }
            }
        }

        int thisSizeCounter = 0;
		std::vector<std::pair<uos, pair<int, uos>>> vecNewTrips{ newTrips.begin(), newTrips.end() };
		const int vntsize = vecNewTrips.size();
		std::pair<set<int>, pair<int, uos>>* it;
        #pragma omp parallel for default(none) private(it) shared(vecNewTrips, thesePotentialTrips, requests, k, dist,thisSizeCounter)
		for(int j = 0; j < vntsize; j++){
            vector<Request> copiedRequests = requests;
            Vehicle virtualCar = Vehicle();
            //int id = omp_get_thread_num();
            it = &(vecNewTrips[j]);
            if (k > 2 && it->second.first < k) continue; //complete clique requirement: all k subsets of size k-1 must be in here
            std::vector<int> inThisTrip(it->first.begin(), it->first.end());
            Request* reqs[max_trip_size];
            for (int i = 0; i < k; i++) {
                reqs[i] = &copiedRequests[inThisTrip[i]];
            }
            bool pathFound = false;
            //NOTE: as of now, start location is irrelevant BUT ONLY BECAUSE time starts at -9999 so delay time is zero
            for (int i = 0; i < k; i++) {
                virtualCar.set_location(reqs[i]->start);
                TravelHelper th;
                if (th.travel(virtualCar, reqs, k, dist, false, true, false) >= 0) {
                    pathFound = true;
                    break;
                }
            }
            if (pathFound == true) {
                #pragma omp critical (updateprior1)
                thesePotentialTrips[k - 1].push_back(tripCandidate(it->first));
                for (auto dependentIter = it->second.second.begin(); dependentIter != it->second.second.end(); dependentIter++) {
                    int temp = *dependentIter;
                    #pragma omp critical (updateprior2)
                    thesePotentialTrips[k - 2][temp].dependentTrips.push_back(thisSizeCounter);
                }
                #pragma omp critical (updateprior2)
                thisSizeCounter++;
            }
        }
		
		std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-startOfSizeK;
		print_line(outDir,logFile,string_format("Potential %d-trips built (%f seconds, %d/%d potential trips kept).", 
			k,
			elapsed_seconds.count(),
			thisSizeCounter,
			vecNewTrips.size()
			));
        lastSizeSize = thisSizeCounter;
    }
    allPotentialTrips = thesePotentialTrips;
}



void RTVGraph::build_single_vehicle(int vehicleId, int vIdx, vector<Vehicle>& vehicles, RVGraph* rvGraph, vector<Request>& requests, map_of_pairs& dist, vector<vector<tripCandidate>>& potentialTrips) {
    clock_t beginClock = clock();

    map<int, int> req_costs;
    rvGraph->get_vehicle_edges(vehicleId, req_costs); //map of req to cost

    int numPreviousSize = 0;
    vector<tripCandidate>& singleTrips = potentialTrips[0];
    bool bNextSizeExists = potentialTrips.size() > 1 && potentialTrips[1].size() > 0;
    for (auto it = singleTrips.begin(); it != singleTrips.end(); it++) {
        tripCandidate& thisCandidate = *it;
        int reqIdx = *(thisCandidate.requests.begin());
        auto req_cost_it = req_costs.find(reqIdx);
        if (req_cost_it == req_costs.end()) {
            thisCandidate.ruledOut = true;
            if (bNextSizeExists) {
                for (int i = 0; i < thisCandidate.dependentTrips.size(); i++) {
                    potentialTrips[1][thisCandidate.dependentTrips[i]].ruledOut = true;
                }
            }
        } else {
            numPreviousSize++;
        }
    }

    // FIRST, TODO: check and see how/whether this code actually uses the in-progress trips?    
    Vehicle& vehicle = vehicles[vehicleId];
    for (int k = 2; k <= max_trip_size; k++) {
        // Ex: to combine 4 requests, previous entry is 3-way combos, of which we need at least 4: 1-2-3, 1-2-4, 1-3-4, 2-3-4
        if (numPreviousSize < k || !bNextSizeExists) break;
        numPreviousSize = 0;
        bNextSizeExists = potentialTrips.size() > k && potentialTrips[k].size() > 0;
        vector<tripCandidate>& newTrips = potentialTrips[k - 1];
        for (int j = 0; j < newTrips.size(); j++){
            if (!newTrips[j].ruledOut) {
                std::vector<int> inThisTrip(newTrips[j].requests.begin(), newTrips[j].requests.end());
				Request* reqs[max_trip_size];
				int itridx = 0;
				for (auto itr = newTrips[j].requests.begin(); itr != newTrips[j].requests.end(); itr++) {
					reqs[itridx++] = &requests[*itr];
				}
                TravelHelper th;
                int cost = th.travel(vehicle, reqs, k, dist, false); // TODO make this return the distance in some cases
                if (cost >= 0) {
					add_edge_trip_vehicle(newTrips[j].requests, vIdx, cost);
					numPreviousSize++;
                } else {
                    newTrips[j].ruledOut = true;
                }
            }
			if (newTrips[j].ruledOut && bNextSizeExists) {
                for (int i = 0; i < newTrips[j].dependentTrips.size(); i++) {
                    potentialTrips[k][newTrips[j].dependentTrips[i]].ruledOut = true;
                }
                continue;
            }
        }
    }
}

void RTVGraph::greedy_assign_same_trip_size(vector<vector<pair<int, pair<int,int>>>::iterator>& edgeIters, vector<vector<pair<int, pair<int,int>>>::iterator>& edgeEnds, vector<int>& tIdxes, set<int>& assignedRIds, set<int>& assignedVIdxes, GRBVar** epsilon, std::map<int, map<int, int>>& lookupRtoT) {
    int numEdgeVectors = edgeIters.size();
    int numNotEnded = numEdgeVectors;

    while (numNotEnded > 0) {
        // get minimal cost edge
        int minCost = 0x7fffffff, argMin = -1;
        int i = 0;
        //#pragma omp parallel for private(i) shared(minCost, argMin)
        for (i = 0; i < numEdgeVectors; i++) {
            if (edgeIters[i] != edgeEnds[i]) {
                int tmpCost = edgeIters[i]->first;
                //#pragma omp critical(edgevecloop)
                //{
                if (tmpCost < minCost)
                    minCost = tmpCost;
                argMin = i;
                //}
            }
        }

        int tIdx = tIdxes[argMin];
        int vIdx = edgeIters[argMin]->second.second;

        // check if the edge can be assigned
        bool allReqsUnassigned = true;
        for (auto iterRId = trips[tIdx].begin(); iterRId != trips[tIdx].end(); iterRId++) {
            if (assignedRIds.find(*iterRId) != assignedRIds.end()) {
                allReqsUnassigned = false;
                break;
            }
        }
        if (allReqsUnassigned && assignedVIdxes.find(vIdx) == assignedVIdxes.end()) {
            // assign the edge
            try {
                int vIdxIntoCosts = lookupRtoT[vIdx][tIdx];
				epsilon[tIdx][vIdxIntoCosts].set(GRB_DoubleAttr_Start, 1.0);
                for (auto iterRId = trips[tIdx].begin(); iterRId != trips[tIdx].end(); iterRId++) {
                    assignedRIds.insert(*iterRId);
                }
                assignedVIdxes.insert(vIdx);
            }
            catch (GRBException& e) {
                print_line(outDir, logFile,string_format("Gurobi exception code: %d.",e.getErrorCode()));
				print_line(outDir, logFile,"Gurobi exception message: "+e.getMessage());
            }
        }

        // move the argMin-th iterator
        edgeIters[argMin]++;
        if (edgeIters[argMin] == edgeEnds[argMin]) {
            numNotEnded--;
        }
    }
}

RTVGraph::RTVGraph(RVGraph* rvGraph, vector<Vehicle>& vehicles, vector<Request>& requests, map_of_pairs& dist) {
	gen1 = std::mt19937(rd());
	gen2 = std::mt19937(rd());
	distribOfCars = std::uniform_int_distribution<>(1, 1000000);
	distribOfTrips = std::uniform_int_distribution<>(1, 1000000); 
			
    numRequests = requests.size();
    numTrips = 0;
    numVehicles = 0;
    for (int i = 1; i <= max_trip_size; i++) {
        allPotentialTrips.push_back(vector<tripCandidate>{});
    }

	auto thisTime = std::chrono::system_clock::now();
    build_potential_trips(rvGraph, requests, dist);
	std::chrono::duration<double> elapsed_seconds = (std::chrono::system_clock::now()-thisTime);
	print_line(outDir,logFile,string_format("Potential trips build time = %f.", elapsed_seconds.count()));
	thisTime = std::chrono::system_clock::now();
    TIdxComparable::rtvGraph = this;
    map<int, int> vehIDToVehIdx;	
	for (int i = 0; i < vehicles.size(); i++) {
		if (rvGraph->has_vehicle(i)) {
			vehIDToVehIdx[i] = addVehicleId(i);
		}
		map<int, int> edges;
		rvGraph->get_vehicle_edges(i, edges); //req, cost
		for(auto it = edges.begin(); it != edges.end(); it++){
            uos tempUOS{ it->first };
			add_edge_trip_vehicle(tempUOS, vehIDToVehIdx[i], it->second);
		}
    }

    vector<vector<tripCandidate>> thesePotentialTrips(allPotentialTrips);
    int m;
	const int vehsize = vehicles.size();
	#pragma omp parallel for private(m) shared(thesePotentialTrips, dist, vehicles, rvGraph, vehIDToVehIdx)
    for (m=0; m < vehsize; m++) {
        if (rvGraph->has_vehicle(m)) {
            vector<vector<tripCandidate>> potentialTripsToUse(thesePotentialTrips);
            build_single_vehicle(m, vehIDToVehIdx[m], vehicles, rvGraph, requests, dist, potentialTripsToUse);
        }
    }
	elapsed_seconds = (std::chrono::system_clock::now()-thisTime);
	print_line(outDir,logFile,string_format("Vehicle-specific RTV graph build time = %f.", elapsed_seconds.count()));
	thisTime = std::chrono::system_clock::now();
    // printf("begin to sort edges\n");
    sort_edges();
	elapsed_seconds = std::chrono::system_clock::now()-thisTime;
	print_line(outDir,logFile,string_format("RTV edge sorting time = %f.", 
		elapsed_seconds.count()));
}

void RTVGraph::rebalance(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved, map_of_pairs& dist) {

    GRBModel model = GRBModel(*env);
    vector<int> idleVIds;
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].get_num_passengers() == 0) {
            idleVIds.push_back(i);
        }
    }
    int idleCnt = (int)idleVIds.size();
    int unservedCnt = (int)unserved.size();

    GRBVar** y = new GRBVar * [idleCnt];
    for (int i = 0; i < idleCnt; i++) {
        y[i] = model.addVars(unservedCnt, GRB_BINARY);
    }

    GRBLinExpr objective = 0;
    GRBLinExpr totalEdgesCnt = 0;
    GRBLinExpr* rEdgesCnt = new GRBLinExpr[unservedCnt];
    for (int j = 0; j < unservedCnt; j++) {
        rEdgesCnt[j] = 0;
    }
    for (int i = 0; i < idleCnt; i++) {
        GRBLinExpr vEdgesCnt = 0;
        for (int j = 0; j < unservedCnt; j++) {
            objective += y[i][j] * get_dist(vehicles[i].get_location(), unserved[j].start, dist);
            totalEdgesCnt += y[i][j];
            vEdgesCnt += y[i][j];
            rEdgesCnt[j] += y[i][j];
        }
        model.addConstr(vEdgesCnt <= 1.0 + minimal);
    }
    for (int j = 0; j < unservedCnt; j++) {
        model.addConstr(rEdgesCnt[j] <= 1.0 + minimal);
    }
    model.addConstr(totalEdgesCnt == min(idleCnt, unservedCnt));
    model.setObjective(objective, GRB_MINIMIZE);

    model.set("TimeLimit", "300.0");
    model.set("OutputFlag", "1");
    model.set("LogToConsole", "0");
    std::string grbLogName = outDir + "GurobiLogs/" + "rebalance_" + std::to_string(now_time);
    model.set("LogFile", grbLogName + ".txt");
    model.optimize();

    std::string part1 = std::to_string(model.get(GRB_IntAttr_Status));
    std::string part2 = std::to_string((int)std::round(model.get(GRB_DoubleAttr_Runtime)));
//    std::rename(std::string(grbLogName + ".txt").c_str(), std::string(grbLogName + "_" + part1 + "_" + part2 + ".txt").c_str());

    for (int i = 0; i < idleCnt; i++) {
        for (int j = 0; j < unservedCnt; j++) {
            double val = y[i][j].get(GRB_DoubleAttr_X);
            if (val < 1.0 + minimal && val > 1.0 - minimal) {
                vehicles[i].head_for(unserved[j].start, dist);
                break;
            }
        }
    }

    delete[] rEdgesCnt;
    for (int i = 0; i < idleCnt; i++) {
        delete[] y[i];
    }
    delete[] y;
}

void RTVGraph::sort_edges() {
	print_line(outDir,logFile,string_format("Starting to sort edges = %f.", 1));
	auto thisTime = std::chrono::system_clock::now();
	std::vector<TIdxComparable> indices;
	indices.reserve(tIdx_vCostIdxes.size());
	std::transform(begin(tIdx_vCostIdxes), end(tIdx_vCostIdxes), std::back_inserter(indices), [](auto const& pair) { return pair.first; });
	std::chrono::duration<double> elapsed_seconds = (std::chrono::system_clock::now()-thisTime);
	print_line(outDir,logFile,string_format("tIdx_vCostIdxes transform time = %f.", elapsed_seconds.count()));
	thisTime = std::chrono::system_clock::now();
	int i;
	#pragma omp parallel for private(i)
	for (i = 0; i < indices.size(); i++) {
		auto iter = tIdx_vCostIdxes.find(indices[i]);
		if (iter != tIdx_vCostIdxes.end()) {
			sort(iter->second.begin(), iter->second.end());
		}
		iter->second.resize(std::min(static_cast<int>(iter->second.size()),max_v_per_req));
	}
	elapsed_seconds = (std::chrono::system_clock::now()-thisTime);
	print_line(outDir,logFile,string_format("tIdx_vCostIdxes sorting time = %f.", elapsed_seconds.count()));
	thisTime = std::chrono::system_clock::now();

	#pragma omp parallel for private(i)
	for (i = 0; i < vIdx_tIdxes.size(); i++) {
		sort(vIdx_tIdxes[i].begin(), vIdx_tIdxes[i].begin());
		vIdx_tIdxes[i].resize(std::min(static_cast<int>(vIdx_tIdxes[i].size()),min_req_per_v));
	}
	elapsed_seconds = (std::chrono::system_clock::now()-thisTime);
	print_line(outDir,logFile,string_format("vIdx_tIdxes sorting time = %f.", elapsed_seconds.count()));
}

void RTVGraph::prune() {
    map_of_pairs tvCombos;
    tvCombos.reserve(max_v_per_req * tIdx_vCostIdxes.size() + min_req_per_v * vIdx_tIdxes.size());
    for (auto iter = tIdx_vCostIdxes.begin(); iter != tIdx_vCostIdxes.end(); iter++) {
        //tvCombos.insert(iter->second.begin(), iter->second.end());
        for (int j = 0; j < (*iter).second.size(); j++) {
            tvCombos.emplace(std::pair<int, int>{
                (*iter).first.tIdx, (*iter).second[j].second.second}, (*iter).second[j].first); //{trip, vehicle}, cost
        }
    }
    for (int i = 0; i < vIdx_tIdxes.size(); i++) {
        for (int j = 0; j < vIdx_tIdxes[i].size(); j++) {
            tvCombos.emplace(std::pair<int, int>{vIdx_tIdxes[i][j].second.second, i}, vIdx_tIdxes[i][j].first); //{trip, vehicle}, cost
        }
    }

    std::vector<TIdxComparable> indices;
    indices.reserve(tIdx_vCostIdxes.size());
    std::transform(begin(tIdx_vCostIdxes), end(tIdx_vCostIdxes), std::back_inserter(indices), [](auto const& pair) { return pair.first; });
    int i;
    #pragma omp parallel for private(i)
    for (i = 0; i < indices.size(); i++) {
        auto iter = tIdx_vCostIdxes.find(indices[i]);
        if (iter != tIdx_vCostIdxes.end()) {
            iter->second.clear();
            iter->second.reserve(max_v_per_req+ min_req_per_v);
        }
    }

    #pragma omp parallel for private(i)
    for (i = 0; i < vIdx_tIdxes.size(); i++) {
        vIdx_tIdxes[i].clear();
        vIdx_tIdxes[i].reserve(max_v_per_req + min_req_per_v);
    }

    for (auto iter = tvCombos.begin(); iter != tvCombos.end(); iter++) {
        int tidx = (*iter).first.first;
        int vidx = (*iter).first.second;
        int cost = (*iter).second;
        tIdx_vCostIdxes[TIdxComparable(tidx)].push_back(pair<int, pair<int, int>>{cost, pair<int, int>{0, vidx}});
        vIdx_tIdxes[vidx].push_back(pair<int, pair<int, int>>{cost, pair<int, int>{0, tidx}});
    }
}

void RTVGraph::solve(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& requests, vector<Request>& unservedCollector, map_of_pairs& dist) {
	auto startOfPrune = std::chrono::system_clock::now(); 
    int prevSize = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); ++i)
    {
        for (int j = 0; j < vIdx_tIdxes[i].size(); ++j)
        {
            prevSize++;
        }
    }
    prune();
    int prunedSize = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); ++i)
    {
        for (int j = 0; j < vIdx_tIdxes[i].size(); ++j)
        {
            prunedSize++;
        }
    }
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - startOfPrune;
    print_line(outDir, logFile, string_format("RTV size trimmed from %d to %d in %f seconds.",
        prevSize, prunedSize, elapsed_seconds.count()));
		
    // printf("Defining variables...\n");
    GRBModel model = GRBModel(*env);
    GRBVar** epsilon = new GRBVar * [numTrips];
    for (int i = 0; i < numTrips; i++) {
        epsilon[i] = model.addVars(tIdx_vCostIdxes[i].size(), GRB_BINARY);
    }

    // default initial values
    // printf("Initializing...\n");
	std::map<int, std::map<int, int>> tempLookupRtoT;
    for (int i = 0; i < numTrips; i++) {
        for (int j = 0; j < tIdx_vCostIdxes[i].size(); j++) {
            epsilon[i][j].set(GRB_DoubleAttr_Start, 0.0);
			tempLookupRtoT[(tIdx_vCostIdxes[i])[j].second.second][i] = j;
        }
    }

    // add constraints
    /* Constraint #1: Assign each vehicle to no more than one trip */
    // printf("Adding constraint #1 ...\n");
    for (int vIdx = 0; vIdx < numVehicles; vIdx++) {
        GRBLinExpr constr = 0;
		const map<int, int>& thisMap = tempLookupRtoT[vIdx];
		for (auto iter = thisMap.begin(); iter != thisMap.end(); iter++) {
			constr += epsilon[iter->first][iter->second];
		}
        model.addConstr(constr <= 1.0 + minimal);
    }
    /* Constraint #2: Each request is served on exactly one trip*/
    // printf("Adding constraint #2 ...\n");
    for (auto iterRV = rId_tIdxes.begin(); iterRV != rId_tIdxes.end(); iterRV++) {
        GRBLinExpr constr = 0;
        int rId = iterRV->first;
        auto iterTIdx = iterRV->second.begin();
        while (iterTIdx != iterRV->second.end()) {
            int tIdx = *iterTIdx;
            vector<pair<int, pair<int,int>> >& vCostIdxes = tIdx_vCostIdxes[TIdxComparable(tIdx)];
			for (int j = 0; j < vCostIdxes.size(); j++) {
				constr += epsilon[tIdx][j];
            }
            iterTIdx++;
        }
        model.addConstr(constr <= 1.0 + minimal);
    }

    // greedy assignment
    // printf("Greedy assignment...\n");
    set<int> assignedRIds, assignedVIdxes;
    vector<int> tIdxes;
    int tripSize = max_trip_size;
    /*
    printf("size of trips: %d %d\n", tIdx_vCostIdxes.size(), trips.size());
    for (iterTV = tIdx_vCostIdxes.begin(); iterTV != tIdx_vCostIdxes.end(); iterTV++) {
    printf("%d, ", trips[iterTV->first.tIdx].size());
    }
    printf("\n");
    */
    vector<vector<pair<int, pair<int,int>> >::iterator> edgeIters, edgeEnds;
    for (auto iterTV = tIdx_vCostIdxes.begin(); iterTV != tIdx_vCostIdxes.end(); iterTV++) {
        if (tripSize > trips[iterTV->first.tIdx].size()) {
            greedy_assign_same_trip_size(
                edgeIters, edgeEnds, tIdxes, assignedRIds, assignedVIdxes,
                epsilon, tempLookupRtoT
            );
            tripSize = trips[iterTV->first.tIdx].size();
            edgeIters.clear();
            edgeEnds.clear();
            tIdxes.clear();
        }
        edgeIters.push_back(iterTV->second.begin());
        edgeEnds.push_back(iterTV->second.end());
        tIdxes.push_back(iterTV->first.tIdx);
    }
    greedy_assign_same_trip_size(
        edgeIters, edgeEnds, tIdxes, assignedRIds, assignedVIdxes,
        epsilon, tempLookupRtoT
    );

    // build objective expression
    GRBLinExpr objective = 0;
    // printf("Generating objective expression...\n");
    for (int tIdx = 0; tIdx < numTrips; tIdx++) {
        vector<pair<int, pair<int,int>> >& vCostIdxes = tIdx_vCostIdxes[TIdxComparable(tIdx)];
		int reqsInTrip = trips[tIdx].size();
		for(int vehIdx = 0; vehIdx < vCostIdxes.size(); vehIdx++){
			objective += epsilon[tIdx][vehIdx] * (vCostIdxes[vehIdx].first - reqsInTrip * penalty);
		}
    }

    model.set("TimeLimit", "1500.0");
    model.set("OutputFlag", "1");
    model.set("LogToConsole", "0");
	model.set("Method","4");
	//model.set("Presolve","1");
	//model.set("VarBranch","3");
	//model.set("MIPFocus","3");
	std::string nt = std::to_string(now_time);
    std::string grbLogName = outDir + "GurobiLogs/" + "rtv_solve_" + nt;
    model.set("LogFile", grbLogName + ".txt");
    model.setObjective(objective, GRB_MINIMIZE);

    // printf("Solving..
    // solve.\n");
	try{
		model.optimize();
	} catch(GRBException e) {
	    print_line(outDir, logFile,string_format("Gurobi exception code: %d.",e.getErrorCode()));
	    print_line(outDir, logFile,"Gurobi exception message: "+e.getMessage());
	} 

    std::string part1 = std::to_string(model.get(GRB_IntAttr_Status));
    std::string part2 = std::to_string((int)std::round(model.get(GRB_DoubleAttr_Runtime)));
    std::string part3 = std::to_string((int)std::round(100 * model.get(GRB_DoubleAttr_MIPGap)));
//    std::rename(std::string(grbLogName + ".txt").c_str(), std::string(grbLogName + "_" + part1 + "_" + part2 + "_" + part3 + ".txt").c_str());

    // printf("numRequests = %d\n", numRequests);
    // printf("Assigned vehicle-trip pairs:\n");
    int cnt = 0;
    utilization.clear();
	uos reqsServed;			   
    for (int vIdx = 0; vIdx < numVehicles; vIdx++) {
        int thisVehicleCount = 0;
		const map<int, int>& thisMap = tempLookupRtoT[vIdx];
		for (auto iter = thisMap.begin(); iter != thisMap.end(); iter++) {
			int tIdx = iter->first;
			double val = epsilon[tIdx][iter->second].get(GRB_DoubleAttr_X);
            if (val < 1.0 + minimal && val > 1.0 - minimal) {
                // printf("Vehicle #%d: [", vIds[vIdx]);
                Vehicle& vehicle = vehicles[vIds[vIdx]];
                Request* reqs[max_trip_size];
                int tripSize = 0;
                for (auto iter = trips[tIdx].begin(); iter != trips[tIdx].end(); iter++) {
                    // printf(", %d", requests[*iter].unique);
                    reqs[tripSize++] = &requests[*iter];
					reqsServed.insert(*iter);						 
                    cnt++;
                    these_served_reqs++;
                    thisVehicleCount++;
                }
                // printf("]\n");

                // update passengers of vehicle
                TravelHelper th;
                th.travel(vehicle, reqs, tripSize, dist, true);

                break;
            }
        }
        utilization[thisVehicleCount] += 1;
    }
    //printf("Number of served requests: %d\n\n", cnt);

    unservedCollector.clear();
    cnt = 0;
    for (int rId = 0; rId < numRequests; rId++) {
        if (rId_tIdxes.find(rId) == rId_tIdxes.end()
            || reqsServed.find(rId) == reqsServed.end()) {
            unservedCollector.push_back(requests[rId]);
            // printf("%d, ", requests[rId].unique);
            cnt++;
        }
    }
    // printf("]\n");
    //printf("Number of unserved requests: %d\n", cnt);

    for (int i = 0; i < numTrips; i++) {
        delete[] epsilon[i];
    }
    delete[] epsilon;
}

