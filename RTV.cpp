#define _HAS_STD_BYTE 0
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
	#pragma omp critical (addetv2)
    tIdx_vCostIdxes[tIdxComparable].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfCars(gen1), vIdx}));
    //#pragma omp critical (addetv3)
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
    auto iter = trip_tIdx.find(trip);
    if (iter != trip_tIdx.end()) {
        return iter->second;
    }
    trip_tIdx.emplace(trip, numTrips);
    trips.push_back(trip);
    for (auto iterRIdx = trip.begin(); iterRIdx != trip.end(); iterRIdx++) {
        rId_tIdxes[*iterRIdx].insert(numTrips);
    }
    return numTrips++;
}

void RTVGraph::build_potential_trips(RVGraph* rvGraph, vector<Request>& requests, vector<Vehicle>& vehicles) {
    //Enumerate trips of size=1
    allPotentialTrips[0].reserve(requests.size());
    for (int i = 0; i < requests.size(); i++) {
        allPotentialTrips[0].push_back(tripCandidate(uos{ i }));
    }

    const int nVeh = vehicles.size();
    const int nReq = requests.size();
    int fullyConnectedVeh = 0;
    int partlyConnectedVeh = 0;
    int totalConnections = 0;
    int maxConnections = 0;
    int vIdx = 0;
    set<pair<int, int>> vehConnex;
    #pragma omp parallel for default(none) private(vIdx) shared(rvGraph, vehConnex, fullyConnectedVeh, partlyConnectedVeh, totalConnections, maxConnections)
    for (vIdx = 0; vIdx < nVeh; vIdx++) {
        auto it = rvGraph->car_req_cost.find(vIdx);
        if (it != rvGraph->car_req_cost.end()) {
            int nConnex = it->second.size();
            if (nConnex > 0) {
                #pragma omp atomic
                partlyConnectedVeh++;
                #pragma omp atomic
                totalConnections += nConnex;
				if (nConnex == nReq){
					#pragma omp atomic
                    fullyConnectedVeh++;
                }
                #pragma omp critical(insertVehConnex)
                vehConnex.insert(make_pair(-nConnex, vIdx));
            }
        }
    }

    const bool bFullyConnectedVeh = fullyConnectedVeh > 0 ? true : false;
	const double bFractionConnected = fullyConnectedVeh/nVeh;
	const double sparsity = 1.0*totalConnections / (1.0*(partlyConnectedVeh * nReq ));
	print_line(outDir,logFile,string_format("%d vehicles, %d requests, %d fully connected vehicles, %d most connected (%f), %f percent dense.", nVeh, nReq, fullyConnectedVeh, maxConnections, maxConnections*1.0/nReq, sparsity));
    const bool bSparseTwo = (!bFullyConnectedVeh) && (sparsity < 0.8);
    const bool bSparseThree = (!bFullyConnectedVeh) && (sparsity < 0.001);

    auto time2 = std::chrono::system_clock::now();
    int travelCounter = 0;
    int lastSizeSize = requests.size();
    int test1counter = 0, test2counter = 0, test3counter = 0;
    for (int k = 2; k <= max_trip_size; k++) {
        /*  Ex: to combine 4 requests, previous entry is 3-way combos,
        of which we need at least 4: 1-2-3, 1-2-4, 1-3-4, 2-3-4 */
        if (lastSizeSize < k) break;
        auto startOfSizeK = std::chrono::system_clock::now();
        int completeCliques = 0;
        map_of_uos newTrips; //key: set of requests (union of two trips); 
        //print_line(outDir,logFile,string_format("Starting to build potential %d-request combinations.", k));                                             
        //value: indices within allPotentialTrips[k-2]
        if (k == 2) {
            if (bSparseTwo) {
				print_line(outDir,logFile,std::string("Enumerating sparse 2"));
                set_of_pairs sop;
				sop.reserve(lastSizeSize* (lastSizeSize - 1) / 2);
				print_line(outDir,logFile,std::string("sparse 2 space reserved"));
                int vIdx2 = 0;
				#pragma omp parallel for default(none) private(vIdx2) shared(sop, k, rvGraph)
                for (vIdx2 = 0; vIdx2 < nVeh; vIdx2++) {
					auto itv2 = rvGraph->car_req_cost.find(vIdx2);
					if (itv2 == rvGraph->car_req_cost.end()) continue;
                    vector<pair<int, int>> vPair;
					vPair.reserve(itv2->second.size() * (itv2->second.size() - 1) / 2);
					map<int, int>::iterator i, j, end = (*itv2).second.end();
					for (i = (*itv2).second.begin(); i != end; ++i)
                    {
                        j = i;
                        j++;
                        for (; j != end; ++j)
                            vPair.push_back(std::make_pair(i->first, j->first));
                    }
					#pragma omp critical(insertTripPair)
                    sop.insert(vPair.begin(), vPair.end());
                }
				
				print_line(outDir,logFile,std::string("sparse 2 loop finished"));
				std::vector<pair<uos,pair<int, uos>>> allCombosOfTwo;
                allCombosOfTwo.reserve(sop.size());
                for (auto it = sop.begin(); it != sop.end(); it++) {
                    allCombosOfTwo.push_back(pair<uos, pair<int, uos>>{uos{ it->first, it->second }, pair<int, uos>{1, uos{ it->first, it->second }}});
                }
                set_of_pairs().swap(sop);
                newTrips.insert(allCombosOfTwo.begin(), allCombosOfTwo.end());
                std::vector<pair<uos, pair<int, uos>>>().swap(allCombosOfTwo);
				
				print_line(outDir,logFile,std::string("sparse 2 section finished"));
            }
            else {
				print_line(outDir,logFile,std::string("Enumerating dense 2"));
                std::vector<pair<uos, pair<int, uos>>> allCombosOfTwo;
                allCombosOfTwo.reserve(lastSizeSize * (lastSizeSize - 1) / 2);
                for (int i = 0; i < lastSizeSize; i++) {
                    for (int j = i + 1; j < lastSizeSize; j++) {
                        allCombosOfTwo.push_back(pair<uos, pair<int, uos>>{uos{ i, j }, pair<int, uos>{1, uos{ i,j }}});
                    }
                }
                newTrips.insert(allCombosOfTwo.begin(), allCombosOfTwo.end());
                std::vector<pair<uos, pair<int, uos>>>().swap(allCombosOfTwo);
            }
			if(bSparseTwo){
				print_line(outDir,logFile,std::string("sparse 2 section part 2 finished"));
			}
        }
        else {
            if (bSparseThree && (k == 3)) {
				print_line(outDir,logFile,std::string("Enumerating sparse 3"));
                uos_of_uos sop;
                sop.reserve((lastSizeSize * (lastSizeSize - 1) * (lastSizeSize - 2) / 3)*sparsity*2);
                int vIdx = 0;
                const int nVeh = vehicles.size();
                #pragma omp parallel for default(none) private(vIdx) shared(sop, k, rvGraph)
                for (vIdx = 0; vIdx < nVeh; vIdx++) {
                    auto it = rvGraph->car_req_cost.find(vIdx);
                    if (it == rvGraph->car_req_cost.end()) continue;
                    vector<uos> vUos;
                    vUos.reserve(it->second.size() * (it->second.size() - 1) * (it->second.size() - 2) / 3);
                    map<int, int>::iterator i, j, l, end = (*it).second.end();
                    for (i = (*it).second.begin(); i != end; ++i)
                    {
                        j = i;
                        j++;
                        for (; j != end; ++j) {
                            l = j;
                            l++;
                            for (; l != end; ++l)
                                vUos.push_back(uos{ i->first, j->first, l->first });
                        }
                    }
                    #pragma omp critical(insertTripPair)
                    sop.insert(vUos.begin(), vUos.end());
                    vector<uos>().swap(vUos);
                }
                std::vector<pair<uos, pair<int, uos>>> allCombosOfThree;
                allCombosOfThree.reserve(sop.size());
                for (auto it = sop.begin(); it != sop.end(); it++) {
                    auto itSop = it->begin();
                    allCombosOfThree.push_back(pair<uos, pair<int, uos>>{*it, pair<int, uos>{0, uos{ *itSop, *(++itSop), *(++itSop) }}});
                }
                uos_of_uos().swap(sop);
                newTrips.insert(allCombosOfThree.begin(), allCombosOfThree.end());
                std::vector<pair<uos, pair<int, uos>>>().swap(allCombosOfThree);
            }
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
            k=3: for 123 to work, 12 & 13 & 23 have to be in 1, 1 & 2 & 3 have to be in 2
            k=4: for 1234 to work, 123 & 124 & 134 & 234 have to be in 1, 12 & 23 & 13 & 14 & 24 & 34 have to be in 2, 1 & 2 & 3 & 4 have to each be in 3
            k=5: 123 has to be in 2, 12 has to be in 3, 1 has to be in 4
            for 12345 to work, 1234, 1235
            for 123456 to work: 1234, 1235, 1236
            */
			print_line(outDir,logFile,string_format("Enumerating potential %d-trips.",k));
            const int twoSmallerSize = allPotentialTrips[k - 3].size();
            int m = 0;
            #pragma omp parallel for default(none) private(m) shared(k, newTrips)
            for (m = 0; m < twoSmallerSize; m++) {
				const vector<int>& dependentTrips = allPotentialTrips[k - 3][m].dependentTrips;
                int numDependentTrips = dependentTrips.size();
                if (numDependentTrips < 2) continue;
                for (int i = 0; i < numDependentTrips; i++) {
                    int indexI = dependentTrips[i];
                    const uos& trip1 = allPotentialTrips[k - 2][indexI].requests;
                    for (int j = i + 1; j < numDependentTrips; j++) {
                        int indexJ = dependentTrips[j];
                        const uos& trip2 = allPotentialTrips[k - 2][indexJ].requests;
                        uos tripUnion(trip1);
                        for (auto it = trip2.begin(); it != trip2.end(); it++) {
                            if (tripUnion.emplace(*it).second) break;
                        }
                        if (bSparseThree && (k == 3)) {
                            auto it = newTrips.find(tripUnion);
                            if (it != newTrips.end()) {
                                    #pragma omp atomic
                                (*it).second.first++;
                                    #pragma omp critical(updateThree2)
                                (*it).second.second.insert({ indexI, indexJ });
                            }
                        }
                        else {
                            #pragma omp critical (updateNewTrips)
                            {
                                auto& it = newTrips[tripUnion];
                                it.first++;
                                it.second.insert({ indexI, indexJ });
                            }
                        }
                    }
                }
            }
        }
        //put the pointers in the vector

		print_line(outDir,logFile,string_format("Enumerated potential %d-trips.",k));

        //print_line(outDir,logFile,string_format("Starting to check feasibility of %d %d-request combinations.", vntsize, k));    

		int thisSizeCounter = 0;
        if (completeCliques >= 0) {
			vector<pair<const uos, pair<int, uos>>*> elements;
			elements.reserve(newTrips.size());
			for(auto it = newTrips.begin(); it != newTrips.end(); it++){
				if(2==k || (it->second.second.size()==k && it->second.first>=k*(k-1)/2)){ //complete clique requirement: all k subsets of size k-1 must be in here
                        elements.push_back(&(*it));
                    }
			}
			elements.shrink_to_fit();
			const int elementSize = elements.size();
            //std::vector<std::pair<uos, pair<int, uos>>> vecNewTrips{ newTrips.begin(), newTrips.end() };
            #pragma omp parallel for default(none) shared(newTrips, elements, requests, k,thisSizeCounter, vehConnex, rvGraph, treeCost)
            for (int j = 0; j < elementSize; j++) {
                if (!bFullyConnectedVeh && !(bSparseTwo && k == 2)) {
                    const auto endRC = rvGraph->req_car.end();
                    bool bVehicleIncludes = false;
                    for (auto itVeh = vehConnex.begin(); itVeh != vehConnex.end(); itVeh++) {
                        bool bMatch = true;
                        for (auto itr = elements[j]->first.begin(); itr != elements[j]->first.end(); itr++) {
                            if (rvGraph->req_car.find(make_pair(*itr, itVeh->second)) == endRC) {
                                bMatch = false;
                                break;
                            }
                        }
                        if (bMatch == true) {
                            bVehicleIncludes = true;
                            break;
                        }
                    }

                    if (bVehicleIncludes = false) {
                        continue;
                    }
                }

                vector<Request> copiedRequests;
                for (auto itr = elements[j]->first.begin(); itr != elements[j]->first.end(); itr++) {
                    copiedRequests.push_back(requests[*itr]);
                }

                Request* reqs[max_trip_size];
                for (int i = 0; i < k; i++) {
                    reqs[i] = &copiedRequests[i];
                }

                bool pathFound = false;
                //NOTE: as of now, start location is irrelevant BUT ONLY BECAUSE time starts at -9999 so delay time is zero
                Vehicle virtualCar = Vehicle();
                auto itRNCNend = rvGraph->req_nearest_car_node.end();
                for (int i = 0; i < k; i++) {
                    auto it = rvGraph->req_nearest_car_node.find(i);
                    if (it != itRNCNend) {
                        virtualCar.set_location(it->second);
                    }
                    else {
                        virtualCar.set_location(reqs[i]->start);
                    }
                    TravelHelper th;
                    if (th.travel(virtualCar, reqs, k, false, true, false) >= 0) {
                        pathFound = true;
                        break;
                    }
                }
                if (pathFound == true) {
					int thisSizeIdx = -1;
					#pragma omp critical (updateprior1)
					{
                    allPotentialTrips[k - 1].push_back(tripCandidate(elements[j]->first));
                    thisSizeIdx = thisSizeCounter++;
					}
                    for (auto dependentIter = elements[j]->second.second.begin(); dependentIter != elements[j]->second.second.end(); dependentIter++) {
                        int temp = *dependentIter;
                        #pragma omp critical (updateprior2)
                        allPotentialTrips[k - 2][temp].dependentTrips.push_back(thisSizeIdx);
                    }
                }
            }
        vector<pair<const uos, pair<int, uos>>*>().swap(elements);
        }
		std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-startOfSizeK;
		print_line(outDir,logFile,string_format("Potential %d-trips built (%f seconds, %d/%d potential trips kept).", 
            k,
            elapsed_seconds.count(),
            thisSizeCounter,
            newTrips.size()
        ));
        lastSizeSize = thisSizeCounter;
        allPotentialTrips[k-1].shrink_to_fit();
    }
}



void RTVGraph::build_single_vehicle(int vehicleId, int vIdx, vector<Vehicle>& vehicles, RVGraph* rvGraph, vector<Request>& requests) {


    const map<int, int>&  req_costs = rvGraph->get_vehicle_edges(vehicleId); //map of req to cost

    int numPreviousSize = 0;
    vector<tripCandidate> thisSizeVec = allPotentialTrips[0];
    vector<tripCandidate> nextSizeVec = allPotentialTrips[1];
    bool bNextSizeExists = nextSizeVec.size() > 0;
    if (!bNextSizeExists) {
        return;
    }
    for (auto it = thisSizeVec.begin(); it != thisSizeVec.end(); it++) {
        tripCandidate& thisCandidate = *it;
        int reqIdx = *(thisCandidate.requests.begin());
        auto req_cost_it = req_costs.find(reqIdx);
        if (req_cost_it == req_costs.end()) {
            thisCandidate.ruledOut = true;
            for (int i = 0; i < thisCandidate.dependentTrips.size(); i++) {
                nextSizeVec[thisCandidate.dependentTrips[i]].ruledOut = true;
            }
        } else {
            numPreviousSize++;
        }
    }

    // FIRST, TODO: check and see how/whether this code actually uses the in-progress trips?    
    Vehicle& vehicle = vehicles[vehicleId];
	const auto endRC = rvGraph->req_car.end();
    for (int k = 2; k <= max_trip_size; k++) {
        // Ex: to combine 4 requests, previous entry is 3-way combos, of which we need at least 4: 1-2-3, 1-2-4, 1-3-4, 2-3-4
        if (numPreviousSize < k || !bNextSizeExists) break;
        thisSizeVec.swap(nextSizeVec);
        nextSizeVec = allPotentialTrips[k];
        numPreviousSize = 0;
        bNextSizeExists = allPotentialTrips.size() > k && allPotentialTrips[k].size() > 0;
        for (int j = 0; j < thisSizeVec.size(); j++){
            if (!thisSizeVec[j].ruledOut) {
 				bool bRuledOut = false;
                vector<Request> copiedRequests;		 
                for (auto itr = thisSizeVec[j].requests.begin(); itr != thisSizeVec[j].requests.end(); itr++) {
                    copiedRequests.push_back(requests[*itr]);
                }

                Request* reqs[max_trip_size];
                for (int i = 0; i < k; i++) {
                    reqs[i] = &copiedRequests[i];
                }

				if(bRuledOut){
					thisSizeVec[j].ruledOut = true;
				}
				else{
                TravelHelper th;
                int cost = th.travel(vehicle, reqs, k, false); // TODO make this return the distance in some cases
                if (cost >= 0) {
                    add_edge_trip_vehicle(thisSizeVec[j].requests, vIdx, th.getTravelCost());
                    numPreviousSize++;
                } else {
                    thisSizeVec[j].ruledOut = true;
                }
            }
            }
            if (thisSizeVec[j].ruledOut && bNextSizeExists) {
                for (int i = 0; i < thisSizeVec[j].dependentTrips.size(); i++) {
                    nextSizeVec[thisSizeVec[j].dependentTrips[i]].ruledOut = true;
                }
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

RTVGraph::RTVGraph(RVGraph* rvGraph, vector<Vehicle>& vehicles, vector<Request>& requests) {
    gen1 = std::mt19937(rd());
    gen2 = std::mt19937(rd());
    distribOfCars = std::uniform_int_distribution<>(1, 1000000);
    distribOfTrips = std::uniform_int_distribution<>(1, 1000000);
	vector<vector<tripCandidate>> ().swap(allPotentialTrips);
    numRequests = requests.size();
    numTrips = 0;
    numVehicles = 0;
    for (int i = 1; i <= max_trip_size; i++) {
        allPotentialTrips.push_back(vector<tripCandidate>{});
    }

    omp_set_num_threads(8);//omp_get_max_threads());///2);// omp_get_max_threads());
    auto thisTime = std::chrono::system_clock::now();
    build_potential_trips(rvGraph, requests, vehicles);
	std::chrono::duration<double> elapsed_seconds = (std::chrono::system_clock::now()-thisTime);
	print_line(outDir,logFile,string_format("Potential trips build time = %f.", elapsed_seconds.count()));
    thisTime = std::chrono::system_clock::now();
    TIdxComparable::rtvGraph = this;
    map<int, int> vehIDToVehIdx;
    for (int i = 0; i < vehicles.size(); i++) {
        if (rvGraph->has_vehicle(i)) {
            vehIDToVehIdx[i] = addVehicleId(i);
        const map<int, int>& edges = rvGraph->get_vehicle_edges(i); //req, cost
			for(auto it = edges.begin(); it != edges.end(); it++){
            uos tempUOS{ it->first };
            add_edge_trip_vehicle(tempUOS, vehIDToVehIdx[i], it->second);
        }
    }
    }

    int m;
    const int vehsize = vehicles.size();
	print_line(outDir,logFile,std::string("Starting parallel section."));
	#pragma omp parallel for default(none) private(m) shared(vehicles, rvGraph, requests, vehIDToVehIdx, treeCost)
    for (m=0; m < vehsize; m++) {
        if (rvGraph->has_vehicle(m)) {
            build_single_vehicle(m, vehIDToVehIdx[m], vehicles, rvGraph, requests);
        }
    }
	vector<vector<tripCandidate>> ().swap(allPotentialTrips);
	elapsed_seconds = (std::chrono::system_clock::now()-thisTime);
	print_line(outDir,logFile,string_format("Vehicle-specific RTV graph build time = %f.", elapsed_seconds.count()));
}

void RTVGraph::rebalance(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved) {

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
            objective += y[i][j] * treeCost.get_dist(vehicles[idleVIds[i]].get_location(), unserved[j].start).first;
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
    model.set("MIPGap", "0.001");
    model.set("OutputFlag", "1");
    model.set("LogToConsole", "0");
	model.set("NodeFileStart","1.0");
    std::string grbLogName = outDir + "GurobiLogs/" + "rebalance_" + std::to_string(now_time);
    model.set("LogFile", grbLogName + ".txt");
    model.optimize();

    std::string part1 = std::to_string(model.get(GRB_IntAttr_Status));
    std::string part2 = std::to_string((int)std::round(model.get(GRB_DoubleAttr_Runtime)));

    for (int i = 0; i < idleCnt; i++) {
        for (int j = 0; j < unservedCnt; j++) {
            double val = y[i][j].get(GRB_DoubleAttr_X);
            if (val < 1.0 + minimal && val > 1.0 - minimal) {
                Request* reqs[1];
                reqs[0] = &unserved[j];
                //vehicles[idleVIds[i]].head_for(unserved[j].start, unserved[j].reqTime, dist);
                TravelHelper th;
                th.travel(vehicles[idleVIds[i]], reqs, 1, true);
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
			iter->second.resize(min(static_cast<int>(iter->second.size()),max_v_per_req));
        iter->second.shrink_to_fit();
    }
	}
	std::vector<TIdxComparable>().swap(indices);
	elapsed_seconds = (std::chrono::system_clock::now()-thisTime);
	print_line(outDir,logFile,string_format("tIdx_vCostIdxes sorting time = %f.", elapsed_seconds.count()));
    thisTime = std::chrono::system_clock::now();

	#pragma omp parallel for private(i)
    for (i = 0; i < vIdx_tIdxes.size(); i++) {
        sort(vIdx_tIdxes[i].begin(), vIdx_tIdxes[i].begin());
		vIdx_tIdxes[i].resize(min(static_cast<int>(vIdx_tIdxes[i].size()),min_req_per_v));
        vIdx_tIdxes[i].shrink_to_fit();
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
    int i = 0;
    #pragma omp parallel for private(i)
    for (i = 0; i < indices.size(); i++) {
        auto iter = tIdx_vCostIdxes.find(indices[i]);
        if (iter != tIdx_vCostIdxes.end()) {
            iter->second.clear();
            iter->second.reserve(max_v_per_req+ min_req_per_v);
        }
    }

	i=0;
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
	
	map_of_pairs().swap(tvCombos);
  
	i=0;
    #pragma omp parallel for private(i)
    for (i = 0; i < indices.size(); i++) {
        auto iter = tIdx_vCostIdxes.find(indices[i]);
        if (iter != tIdx_vCostIdxes.end()) {
            iter->second.shrink_to_fit();
        }
    }
	std::vector<TIdxComparable>().swap(indices);

	i=0;
    #pragma omp parallel for private(i)
    for (i = 0; i < vIdx_tIdxes.size(); i++) {
        vIdx_tIdxes[i].shrink_to_fit();
    }
}

void RTVGraph::solve(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& requests, vector<Request>& unservedCollector) {
    auto startOfPrune = std::chrono::system_clock::now();
    // printf("begin to sort edges\n");
    sort_edges();
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - startOfPrune;
    print_line(outDir, logFile, string_format("RTV edge sorting time = %f.",
        elapsed_seconds.count()));
    int prevSize = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); ++i)
    {
        prevSize = prevSize + vIdx_tIdxes[i].size();
    }
    prune();
    int prunedSize = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); ++i)
    {
        prunedSize = prunedSize + vIdx_tIdxes[i].size();
    }
    elapsed_seconds = std::chrono::system_clock::now() - startOfPrune;
    print_line(outDir, logFile, string_format("RTV size trimmed from %d to %d in a total of %f seconds.",
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
    int maxCostTrip = 0;
    vector<vector<pair<int, pair<int, int>> >::iterator> edgeIters, edgeEnds;
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
        maxCostTrip = max(maxCostTrip, iterTV->second[iterTV->second.size() - 1].first);
    }
    greedy_assign_same_trip_size(
        edgeIters, edgeEnds, tIdxes, assignedRIds, assignedVIdxes,
        epsilon, tempLookupRtoT
    );

    int thisPenalty = maxCostTrip * 10; //penalty also defined in globals.h/.cpp

    // build objective expression
    GRBLinExpr objective = 0;
    // printf("Generating objective expression...\n");
    for (int tIdx = 0; tIdx < numTrips; tIdx++) {
        vector<pair<int, pair<int,int>> >& vCostIdxes = tIdx_vCostIdxes[TIdxComparable(tIdx)];
        int reqsInTrip = trips[tIdx].size();
        for (int vehIdx = 0; vehIdx < vCostIdxes.size(); vehIdx++) {
            objective += epsilon[tIdx][vehIdx] * (vCostIdxes[vehIdx].first - reqsInTrip * thisPenalty);
        }
    }
    model.set("TimeLimit", "1500.0");
    model.set("MIPGap", "0.001");
    model.set("OutputFlag", "1");
    model.set("LogToConsole", "0");
	model.set("Method","3");
	model.set("NodeFileStart","1.0");
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
                    reqsServed.emplace(*iter);
                    cnt++;
                    these_served_reqs++;
                    thisVehicleCount++;
                }
                // update passengers of vehicle
                TravelHelper th;
                th.travel(vehicle, reqs, tripSize, true);
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

