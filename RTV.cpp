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
#include <iomanip>
#include <sstream>
#include <cstring>
#include <fstream>
#include <filesystem>
#include "hps_src/hps.h"
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
    vIdx_tIdxes.push_back(vector<pair<int, pair<int, int>> >());
    return numVehicles++;
}

void RTVGraph::add_edge_trip_vehicle(const uos& reqsInTrip, int vIdx, int cost) {
    int tIdx;
    tIdx = getTIdx(reqsInTrip);
    TIdxComparable tIdxComparable(tIdx);
    #pragma omp critical (addetv2)
    tIdx_vCostIdxes[tIdxComparable].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfCars(gen1), vIdx}));
    //#pragma omp critical (addetv3)
    vIdx_tIdxes[vIdx].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfTrips(gen2), tIdx}));
}

void RTVGraph::add_edge_trip_vehicle(int tIdx, int vIdx, int cost) {
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

    int toReturn = 0;
    #pragma omp critical (addetv1)
    {
        trip_tIdx.emplace(trip, numTrips);
        trips.push_back(trip);
        toReturn = numTrips++;
    }
    return toReturn;
}

void RTVGraph::build_potential_trips(RVGraph* rvGraph, vector<Request>& requests, vector<Vehicle>& vehicles) {
    //Enumerate trips of size=1
    const int nVeh = vehicles.size();
    const int nReq = requests.size();
    int fullyConnectedVeh = 0;
    int partlyConnectedVeh = 0;
    int totalConnections = 0;
    int maxConnections = 0;
    set<pair<int, int>> vehConnex;
    map<int, int> vehIDToVehIdx;
    bool bUnusedVehicle = false;
    std::vector<std::pair<int, uos>> prevInclusions;
    prevInclusions.reserve(nVeh);
    int addedToPrevInclusions = 0;
    for (int i = 0; i < nVeh; i++) {
        if (rvGraph->has_vehicle(i)) {
            int vIdx = addVehicleId(i);
            if (vehicles[vIdx].getAvailableSince() == -9999) {
                bUnusedVehicle = true;
            }
            vehIDToVehIdx[i] = vIdx;
            const map<int, int>& edges = rvGraph->get_vehicle_edges(i); //req, cost
            int nConnex = edges.size();
            if (nConnex > 1) {
                prevInclusions.push_back(std::pair<int, uos>(i, uos()));
                vehConnex.insert(make_pair(-nConnex, addedToPrevInclusions));
                addedToPrevInclusions++;
            }
            for (auto it = edges.begin(); it != edges.end(); ++it) {
                uos tempUOS{ it->first };
                add_edge_trip_vehicle(tempUOS, vIdx, it->second);
                if (nConnex > 1) prevInclusions[prevInclusions.size() - 1].second.insert(it->first);
            }
            if (nConnex > 0) {
                maxConnections = max(maxConnections, nConnex);
                partlyConnectedVeh++;
                totalConnections += nConnex;
                if (nConnex == nReq) {
                    fullyConnectedVeh++;
                }
            }
        }
    }

    for (int m = 0; m < numVehicles; m++) ;
    allPotentialTrips[0].reserve(requests.size());
    int reqsAdded = 0;
    for (int i = 0; i < requests.size(); i++) {
        allPotentialTrips[0].push_back(tripCandidate(uos{ i }));
    }

    serialize_current_combos(); 

    const bool bFullyConnectedVeh = fullyConnectedVeh > 0 ? true : false;
    const double bFractionConnected = fullyConnectedVeh / nVeh;
    const double sparsity = totalConnections / (1.0 * (partlyConnectedVeh * nReq));
    print_line(outDir, logFile, string_format("%d vehicles, %d requests, %d fully connected vehicles, %d most connected (%f), %d percent dense.", nVeh, nReq, fullyConnectedVeh, maxConnections, maxConnections * 100.0 / nReq, 100.0 * sparsity));
    const bool bSparseTwo = (!bFullyConnectedVeh) && (sparsity < 0.8);
    const bool bSparseThree =  (!bFullyConnectedVeh) && (sparsity < 0.001);
    const bool bTesting = true;
    map<int, int> adjustedTripIdxes;
    auto time2 = std::chrono::system_clock::now();
    int travelCounter = 0;
    int lastSizeSize = requests.size();
    int test1counter = 0, test2counter = 0, test3counter = 0;
    for (int k = 2; k <= max_trip_size; k++) {
        /*  Ex: to combine 4 requests, previous entry is 3-way combos,
        of which we need at least 4: 1-2-3, 1-2-4, 1-3-4, 2-3-4 */
        if (allPotentialTrips[k - 2].size() < k) break;
        allPotentialTrips.push_back(vector<tripCandidate>{});
        auto startOfSizeK = std::chrono::system_clock::now();
        int completeCliques = 0;
        map_of_uos newTrips; //key: set of requests (union of two trips);                                      
        //value: indices within allPotentialTrips[k-2]
        //print_line(outDir,logFile,string_format("Starting to build potential %d-request combinations.", k));        
        if (k == 2) {
            if (bSparseTwo) {
                print_line(outDir, logFile, std::string("Enumerating sparse 2"));
                set_of_pairs sop;
                sop.reserve(lastSizeSize * (lastSizeSize - 1) / 2);
                print_line(outDir, logFile, std::string("sparse 2 space reserved"));
                int vIdx2 = 0;
                #pragma omp parallel for default(none) private(vIdx2) shared(sop, k, rvGraph)
                for (vIdx2 = 0; vIdx2 < numVehicles; vIdx2++) {
                    auto itv2 = rvGraph->car_req_cost.find(vIds[vIdx2]);
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

                print_line(outDir, logFile, std::string("sparse 2 loop finished"));
                std::vector<pair<uos, pair<int, uosTBB>>> allCombosOfTwo;
                allCombosOfTwo.reserve(sop.size());
                for (auto it = sop.begin(); it != sop.end(); ++it) {
                    allCombosOfTwo.push_back(pair<uos, pair<int, uosTBB>>{uos{ it->first, it->second }, pair<int, uosTBB>{1, uosTBB{ it->first, it->second }}});
                }
                set_of_pairs().swap(sop);
                newTrips.insert(allCombosOfTwo.begin(), allCombosOfTwo.end());
                std::vector<pair<uos, pair<int, uosTBB>>>().swap(allCombosOfTwo);

                print_line(outDir, logFile, std::string("sparse 2 section finished"));
            }
            else {
                print_line(outDir, logFile, std::string("Enumerating dense 2"));
                std::vector<pair<uos, pair<int, uosTBB>>> allCombosOfTwo;
                allCombosOfTwo.reserve(lastSizeSize * (lastSizeSize - 1) / 2);
                for (int i = 0; i < lastSizeSize; i++) {
                    for (int j = i + 1; j < lastSizeSize; j++) {
                        allCombosOfTwo.push_back(pair<uos, pair<int, uosTBB>>{uos{ i, j }, pair<int, uosTBB>{1, uosTBB{ i,j }}});
                    }
                }
                newTrips.insert(allCombosOfTwo.begin(), allCombosOfTwo.end());
                std::vector<pair<uos, pair<int, uosTBB>>>().swap(allCombosOfTwo);
            }
            if (bSparseTwo) {
                print_line(outDir, logFile, std::string("sparse 2 section part 2 finished"));
            }
        }
        else {
            if (bSparseThree && (k == 3)) {
                print_line(outDir, logFile, std::string("Enumerating sparse 3"));
                uos_of_uos sop;
                sop.reserve((lastSizeSize * (lastSizeSize - 1) * (lastSizeSize - 2) / 3) * sparsity * 2);
                int vIdx = 0;
                #pragma omp parallel for default(none) private(vIdx) shared(sop, k, rvGraph)
                for (vIdx = 0; vIdx < numVehicles; vIdx++) {
                    auto it = rvGraph->car_req_cost.find(vIds[vIdx]);
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
                    #pragma omp critical(insertTripPair2)
                    sop.insert(vUos.begin(), vUos.end());
                    vector<uos>().swap(vUos);
                }
                std::vector<pair<uos, pair<int, uosTBB>>> allCombosOfThree;
                allCombosOfThree.reserve(sop.size());
                for (auto it = sop.begin(); it != sop.end(); ++it) {
                    auto itSop = it->begin();
                    allCombosOfThree.push_back(pair<uos, pair<int, uosTBB>>{*it, pair<int, uosTBB>{0, uosTBB{ *itSop, *(++itSop), *(++itSop) }}}); //TODO: 0 or 1 here???
                }
                uos_of_uos().swap(sop);
                newTrips.insert(allCombosOfThree.begin(), allCombosOfThree.end());
                std::vector<pair<uos, pair<int, uosTBB>>>().swap(allCombosOfThree);
            }
            else if (bTesting || (k >2 && lastSizeSize > 100000)) {
                int m = 0;
                std::set<std::pair<int, int>> allCombos;
                //#pragma omp parallel for default(none) private(m) shared(k, newTrips, allCombos, adjustedTripIdxes)
                for (m = 0; m < prevInclusions.size(); m++) {
                    std::vector<std::pair<int,int>> theseCombos;
                    theseCombos.reserve(prevInclusions[m].second.size() * (prevInclusions[m].second.size() - 1) / 2);
                    uos::iterator i, j, end = prevInclusions[m].second.end();
                    for (i = prevInclusions[m].second.begin(); i != end; ++i)
                    {
                        j = i;
                        j++;
                        for (; j != end; ++j)
                            theseCombos.push_back(std::make_pair(*i, *j));
                    }
                    //#pragma omp critical(insertSetThree)
                    allCombos.insert(theseCombos.begin(), theseCombos.end());
                }
                std::vector<std::pair<int, int>> allCombosVec(allCombos.begin(), allCombos.end());
                std::set<std::pair<int, int>>().swap(allCombos);
                #pragma omp parallel for default(none) private(m) shared(k, newTrips, allCombos, adjustedTripIdxes)
                for (m = 0; m < allCombosVec.size(); m++) {
                    auto itI = adjustedTripIdxes.find(allCombosVec[m].first);
                    auto itJ = adjustedTripIdxes.find(allCombosVec[m].second);
                    if (itI == adjustedTripIdxes.end() || itJ == adjustedTripIdxes.end()) {
                        int x = 5; 
                        continue;
                    }
                    const uos& trip1 = allPotentialTrips[k - 2][itI->second].requests;
                    const uos& trip2 = allPotentialTrips[k - 2][itJ->second].requests;
                    pair<int, int> dj = getDisjunction(trip1, trip2);
                    if (dj.first == -1) {
                        std::vector<int> test;
                        std::set_union(trip1.begin(), trip1.end(), trip2.begin(), trip2.end(), std::inserter(test, test.begin()));
                        if (test.size() == k) {
                            int x = 5;
                        }
                        continue;
                    }
                    uos tripUnion = dj.first == 0 ? trip1 : trip2;
                    tripUnion.insert(dj.second);
                    std::pair<int, uosTBB>* it = NULL;
                    #pragma omp critical(updateNewTrips)
                    it = &(newTrips[tripUnion]);
                    #pragma omp atomic
                    it->first++;
                    #pragma omp critical(insertInto2nd)
                    it->second.insert({ itI->second, itJ->second });
                }
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
            print_line(outDir, logFile, string_format("Enumerating potential %d-trips.", k));
            const int twoSmallerSize = allPotentialTrips[k - 3].size();
            auto adjustedTripEnd = adjustedTripIdxes.end();
            int m = 0;
            #pragma omp parallel for default(none) private(m) shared(k, newTrips, adjustedTripEnd, adjustedTripIdxes)
            for (m = 0; m < twoSmallerSize; m++) {
                const vector<int>& dependentTrips = allPotentialTrips[k - 3][m].dependentTrips;
                int numDependentTrips = dependentTrips.size();
                if (numDependentTrips < 2) continue;
                for (int i = 0; i < numDependentTrips; i++) {
                    auto itI = adjustedTripIdxes.find(dependentTrips[i]);
                    if (itI == adjustedTripEnd) continue;
                    int indexI = itI->second;
                    const uos& trip1 = allPotentialTrips[k - 2][indexI].requests;
                    for (int j = i + 1; j < numDependentTrips; j++) {
                        auto itJ = adjustedTripIdxes.find(dependentTrips[j]);
                        if (itJ == adjustedTripEnd) continue;
                        int indexJ = itJ->second;
                        const uos& trip2 = allPotentialTrips[k - 2][indexJ].requests;
                        pair<int, int> dj = getDisjunction(trip1, trip2);
                        if (dj.first == -1) continue;
                        uos tripUnion = dj.first == 0 ? trip1 : trip2;
                        tripUnion.insert(dj.second);
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
                            std::pair<int, uosTBB>* it = NULL;
                            #pragma omp critical(updateNewTrips)
                            it = &(newTrips[tripUnion]);
                            #pragma omp atomic
                            it->first++;
                            #pragma omp critical(insertInto2nd)
                            it->second.insert({ indexI, indexJ });
                        }
                    }
                }
            }
        }
        //put the pointers in the vector


        //print_line(outDir,logFile,string_format("Starting to check feasibility of %d %d-request combinations.", vntsize, k));    

        int thisSizeCounter = 0;
        int newTripsSize = newTrips.size();
        int elementSize = 0;
		vector<pair<const uos, pair<int, uosTBB>>*> elements;
        if (k > 2) {
            for (auto it = newTrips.begin(); it != newTrips.end(); )
            {   //complete clique requirement: all k subsets of size k-1 must be in here
                if (it->second.second.size() == k && it->second.first >= k * (k - 1) / 2) { ++it; }
                else { newTrips.erase(it++); }
            }
            newTrips.rehash(0);
        }
        elements.reserve(newTrips.size());
		for(auto it = newTrips.begin(); it != newTrips.end(); it++){
            elements.push_back(&(*it));
        }
        print_line(outDir, logFile, string_format("Enumerated potential %d-trips.", k));
        print_ram(outDir, string_format("%d_after_potential_trip_enumeration_%d", now_time, k));
        elementSize = elements.size();
        //#pragma omp parallel for default(none) schedule(guided) shared(prevInclusions, newTrips, elementSize, elements, requests, k, thisSizeCounter, vehConnex, rvGraph, treeCost, adjustedTripIdxes)
        for (int j = 0; j < elementSize; j++) {
            if (!bFullyConnectedVeh && !(bSparseTwo && k == 2)) {
                bool bVehicleIncludes = false;
                for (auto itVeh = vehConnex.begin(); itVeh != vehConnex.end(); ++itVeh) {
                    bool bMatch = true;
                    for (auto itr = elements[j]->second.second.begin(); itr != elements[j]->second.second.end(); ++itr) {
                        if (prevInclusions[itVeh->second].second.find(*itr) == prevInclusions[itVeh->second].second.end()) {
                            bMatch = false;
                            break;
                        }
                    }
                    if (bMatch == true) {
                        bVehicleIncludes = true;
                        break;
                    }
                }

                if (bVehicleIncludes == false) {
					//#pragma omp atomic
					//noVeh++;
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
            Vehicle virtualCar;
            virtualCar.set_location(reqs[0]->start);
            TravelHelper th;
            if (th.travel(virtualCar, reqs, k, false, true, true) >= 0) {
                pathFound = true;
            }
			//print_line(outDir, logFile, string_format("AnsTravvelled = %f, allPotentialTrips size = %d, elements size = %d, j = %d", th.ansCost, allPotentialTrips[k - 2].size(), elements.size(), j ));
            if (pathFound == true) {
                int thisSizeIdx = -1;
                #pragma omp critical (updateprior1)
                {
                    allPotentialTrips[k - 1].push_back(tripCandidate(elements[j]->first));
                    thisSizeIdx = thisSizeCounter++;
                }
				for (auto dependentIter = elements[j]->second.second.begin(); dependentIter != elements[j]->second.second.end(); dependentIter++) {
					
                    #pragma omp critical (updateprior2)
                    allPotentialTrips[k - 2][*dependentIter].dependentTrips.push_back(thisSizeIdx);
                }
            }
        }
        //print_line(outDir, logFile, string_format("%d elements had no vehicle", noVeh));
        vector<pair<const uos, pair<int, uosTBB>>*>().swap(elements);
        map_of_uos().swap(newTrips);
        allPotentialTrips[k - 1].shrink_to_fit();
        std::vector<int> addedTrips(allPotentialTrips[k - 1].size(), 0);
        vector<std::pair<int,uos>> theseInclusions;
        theseInclusions.reserve(prevInclusions.size());
        for (int m = 0; m < prevInclusions.size(); m++) {
            theseInclusions.push_back(std::pair<int,uos>(prevInclusions[m].first,uos()));
        }
        print_line(outDir, logFile, "Build_single_vehicles starting.");
        build_single_vehicles(requests, vehicles, vehIDToVehIdx, k, adjustedTripIdxes, addedTrips, prevInclusions, theseInclusions);
        theseInclusions.swap(prevInclusions);
        vector<std::pair<int, uos>>().swap(theseInclusions);

        print_line(outDir, logFile, "Build_single_vehicles done.");
        print_line(outDir, logFile, "Trips labeled.");
        adjustedTripIdxes.clear();
        int oldIdx = 0;
        int newIdx = 0;
        int m = 0;
        int sizeBeforeShrinking = allPotentialTrips[k - 1].size();
        auto toCheckIt = allPotentialTrips[k - 1].begin();
        //When k-trips are invalidated, delete them (reducing # of links for k-1-trips)
        for (m = 0; m < sizeBeforeShrinking; m++) {
            if (addedTrips[m] == 1) {
                adjustedTripIdxes[oldIdx] = newIdx++;
            }
            oldIdx++;
        }

        vector<tripCandidate>& toShrink = allPotentialTrips[k - 1];
        toShrink.erase(std::remove_if(toShrink.begin(), toShrink.end(),
            [&addedTrips, &toShrink](const tripCandidate& o) { return !addedTrips[&o - &*(toShrink.begin())]; }),
            toShrink.end());
        allPotentialTrips[k - 1].shrink_to_fit();

        /* When building 3-trips, want to make a bimap between 2- & 3-trips
        * When 3-trips are invalidated, delete them (reducing # of links for 2-trips)
        * Then, in anticipation of 4-trips:
        *   Delete 2-trips with fewer than 2 3-trips
        *   Delete 3-trips with fewer than 3 2-trips
        *   Delete 2-trips with fewer than 2 3-trips
        *   etc.
        * When building 4-trips, want to make a bimap between 3- & 4-trips
        * When 4-trips are invalidated, delete them (reducing # of links for 3-trips)
        * Then, in anticipation of 5-trips:
        *   Delete 3-trips with fewer than 2 4-trips
        *   Delete 4-trips with fewer than 3 3-trips
        *   Delete 3-trips with fewer than 2 4-trips
        *   etc.
        */
        //Erase removed k-trip dependencies from k-1-trips
        for (int i = 0; i < allPotentialTrips[k - 2].size(); i++) { //
            std::vector<int>& theseDependents = allPotentialTrips[k - 2][i].dependentTrips;
            auto it = theseDependents.begin();
            while ( it != theseDependents.end()) {
                if (adjustedTripIdxes.find(*it) != adjustedTripIdxes.end()) { ++it; }
                else { it = theseDependents.erase(it); }
            }
        }
        //Remove k-1-trips with fewer than 2 k-trip dependencies
        int temp1 = allPotentialTrips[k - 2].size();
        allPotentialTrips[k - 2].erase(std::remove_if(allPotentialTrips[k - 2].begin(), allPotentialTrips[k - 2].end(),
            [](const tripCandidate& o) { return o.dependentTrips.size() < 0; }),
            allPotentialTrips[k - 2].end());
        int temp2 = temp1 - allPotentialTrips[k - 2].size();
        allPotentialTrips[k - 2].erase(std::remove_if(allPotentialTrips[k - 2].begin(), allPotentialTrips[k - 2].end(),
            [](const tripCandidate& o) { return o.dependentTrips.size() < 2; }),
            allPotentialTrips[k - 2].end());
        int temp3 = temp1 - allPotentialTrips[k - 2].size() - temp2;
        print_line(outDir, logFile, string_format("%d-trips: of %d, removed %d with 0 dependents and %d with <2 dependents.", k-1, temp1, temp2, temp3));
        //Remove k-trips with fewer than 3 k-1-trips
        map<int, int> reverseDependentCounts;
        for (int i = 0; i < allPotentialTrips[k - 2].size(); i++) {
            for (int j = 0; j < allPotentialTrips[k - 2][i].dependentTrips.size(); j++) {
                reverseDependentCounts[allPotentialTrips[k - 2][i].dependentTrips[j]]++;
            }
        }
        int b4adj = adjustedTripIdxes.size();
        auto it = adjustedTripIdxes.begin();
        while (it != adjustedTripIdxes.end()) {
            auto it2 = reverseDependentCounts.find(it->first);
            if (it2 == reverseDependentCounts.end() || it2->second < k) { adjustedTripIdxes.erase(it++); }
            else { ++it; };
        }
        int afteradj = adjustedTripIdxes.size();
        print_line(outDir, logFile, string_format("k-trips: of %d, removed %d with <2 antecedents.", b4adj, b4adj-afteradj));

        if (k >= 3) {
            vector<tripCandidate>().swap(allPotentialTrips[k - 3]);
            allPotentialTrips[k - 3].clear();
            allPotentialTrips[k-3].shrink_to_fit();
        }
        lastSizeSize = allPotentialTrips[k - 1].size();

        vehConnex.clear();
        auto itPrev = prevInclusions.begin();
        int idx = 0;
        while (itPrev != prevInclusions.end()) {
            for (auto it2 = itPrev->second.begin(); it2 != itPrev->second.end(); )
            {
                if (adjustedTripIdxes.find(*it2) != adjustedTripIdxes.end()) { ++it2; }
                else {it2 = itPrev->second.erase(it2); }
            }
            int connex = itPrev->second.size();
            if (connex == 0) {
                itPrev = prevInclusions.erase(itPrev);
            }
            else {
                if(connex > k) vehConnex.insert(make_pair(-connex, idx));
                idx++;
                ++itPrev;
            }
        }

        std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - startOfSizeK;
        print_line(outDir, logFile, string_format("Potential %d-trips built (%f seconds, %d/%d met clique reqmt, %d after ideal vehicle, %d after single vehicles).",
            k,
            elapsed_seconds.count(),
            elementSize,
            newTripsSize,
            sizeBeforeShrinking,
            lastSizeSize
        ));
    }
}

void RTVGraph::build_single_vehicles(vector<Request>& requests, vector<Vehicle>& vehicles,
    const map<int, int>& vehIDToVehIdx, int tripSize,
    const map<int, int>& adjustedTripIdxes, std::vector<int>& addedTrips,
    const std::vector<std::pair<int,uos>>& prevInclusions, std::vector<std::pair<int, uos>>& theseInclusions) {
    if (tripSize < 2) return;
    if (allPotentialTrips.size() < tripSize) return;;
    const std::vector<tripCandidate>& lastSizeVec = allPotentialTrips[tripSize - 2];
    const std::vector<tripCandidate>& thisSizeVec = allPotentialTrips[tripSize - 1];
    if (lastSizeVec.size() < tripSize || thisSizeVec.size() == 0) return;

    std::vector<std::pair<int, std::pair<int, int>>> allValidTripCosts;
    int m = 0;
    #pragma omp parallel for default(none) private(m) schedule(guided) shared(allValidTripCosts, addedTrips, adjustedTripIdxes, requests, vehicles, vehIDToVehIdx, tripSize, treeCost, lastSizeVec, thisSizeVec, prevInclusions, theseInclusions)
    for (m = 0; m < prevInclusions.size(); m++) {
        int vId = prevInclusions[m].first; //index into vehicles, not RTV vehicle indices
        const uos& carPrev = prevInclusions[m].second;
        if (carPrev.size() < tripSize) continue;

        int vIdx = -1;
        auto itVeh = vehIDToVehIdx.find(vId);
        if (itVeh != vehIDToVehIdx.end()) {
            vIdx = itVeh->second;
        }
        else {
            continue;
        }

        std::vector<std::pair<int,std::pair<int,int>>> validTripCosts;
        validTripCosts.reserve(carPrev.size() * (carPrev.size() - 1) / 2);
        if (carPrev.size() == lastSizeVec.size()) {
            for (int i = 0; i < thisSizeVec.size(); i++) {
                vector<Request> copiedRequests;
                copiedRequests.reserve(thisSizeVec[i].requests.size());
                for (auto itr = thisSizeVec[i].requests.begin(); itr != thisSizeVec[i].requests.end(); ++itr) {
                    copiedRequests.push_back(requests[*itr]);
                }

                Request* reqs[max_trip_size];
                for (int j = 0; j < tripSize; j++) {
                    reqs[j] = &copiedRequests[j];
                }

                Vehicle& vehicle = vehicles[vId];
                TravelHelper th;
                int cost = th.travel(vehicle, reqs, tripSize, false);
                if (cost >= 0) {
                    int tripIdx = getTIdx(thisSizeVec[i].requests);
                    validTripCosts.push_back(make_pair(vIdx,make_pair(tripIdx, cost)));
                    addedTrips[i] = 1;
                    //add_edge_trip_vehicle(thisSizeVec[i].requests, vIdx, cost);
                    theseInclusions[m].second.insert(i);
                }
            }
        }
        else {
            int sparseComplexity = carPrev.size() * (carPrev.size() - 1);
            int denseComplexity = thisSizeVec.size() * tripSize;
            unordered_map<int, int> carThis;
            if (tripSize == 2) {
                for (auto it = carPrev.begin(); it != carPrev.end(); ++it) {
                    int idx = *it;
                    for (int i = 0; i < lastSizeVec[idx].dependentTrips.size(); i++) {
                        carThis[lastSizeVec[idx].dependentTrips[i]]++;
                    }
                }
            }
            else {
                auto itEnd = adjustedTripIdxes.end();
                for (auto it = carPrev.begin(); it != carPrev.end(); ++it) {
                    auto itr = adjustedTripIdxes.find(*it);
                    if (itr == itEnd) continue;
                    int idx = itr->second;
                    for (int i = 0; i < lastSizeVec[idx].dependentTrips.size(); i++) {
                        carThis[lastSizeVec[idx].dependentTrips[i]]++;
                    }
                }
            }
            for (auto it = carThis.begin(); it != carThis.end(); ++it) {
                if (it->second != tripSize) continue; //TODO just remove these indices?
                vector<Request> copiedRequests;
                copiedRequests.reserve(thisSizeVec[it->first].requests.size());
                for (auto itr = thisSizeVec[it->first].requests.begin(); itr != thisSizeVec[it->first].requests.end(); ++itr) {
                    copiedRequests.push_back(requests[*itr]);
                }

                Request* reqs[max_trip_size];
                for (int j = 0; j < tripSize; j++) {
                    reqs[j] = &copiedRequests[j];
                }

                Vehicle& vehicle = vehicles[m];
                TravelHelper th;
                int cost = th.travel(vehicle, reqs, tripSize, false);
                if (cost >= 0) {
                    //add_edge_trip_vehicle(thisSizeVec[it->first].requests, vIdx, cost);
                    int tripIdx = getTIdx(thisSizeVec[it->first].requests);
                    validTripCosts.push_back(make_pair(vIdx, make_pair(tripIdx, cost)));
                    addedTrips[it->first] = 1;
                    theseInclusions[m].second.insert(it->first);
                }
            }
        }
        #pragma omp critical(pushingToVec)
        allValidTripCosts.insert(allValidTripCosts.end(), validTripCosts.begin(), validTripCosts.end());
    }
    std::string vehTripsFile = outDir + "Misc/Trips/trips_valid_" + to_string(tripSize) + ".hps";
    std::ofstream out_file(vehTripsFile, std::ofstream::binary | std::ios_base::app);
    hps::to_stream(allValidTripCosts, out_file);
    out_file.close();
    std::vector<std::pair<int, std::pair<int, int>>>().swap(allValidTripCosts);
}

void RTVGraph::serialize_current_combos() {
    std::string tvCombosFile = outDir + "Misc/Trips/tvCombos.hps";
    std::ofstream out_tv(tvCombosFile, std::ofstream::binary | std::ios_base::app);
    hps::to_stream(tIdx_vCostIdxes, out_tv);
    out_tv.close();
    map<TIdxComparable, vector<pair<int, pair<int, int>> > >().swap(tIdx_vCostIdxes);
    std::string vtCombosFile = outDir + "Misc/Trips/vtCombos.hps";
    std::ofstream out_vt(tvCombosFile, std::ofstream::binary | std::ios_base::app);
    hps::to_stream(vIdx_tIdxes, out_vt);
    out_vt.close();
    vector<vector<pair<int, pair<int, int>> > >().swap(vIdx_tIdxes);
}

void RTVGraph::deserialize_current_combos() {
    std::string tvCombosFile = outDir + "Misc/Trips/tvCombos.hps";
    if (!std::filesystem::exists(tvCombosFile)) {
        std::ifstream in_tv(tvCombosFile, std::ofstream::binary);
        tIdx_vCostIdxes = hps::from_stream <std::map<TIdxComparable, vector<pair<int, pair<int, int>>>>>(in_tv);
    }
    std::string vtCombosFile = outDir + "Misc/Trips/vtCombos.hps";
    if (!std::filesystem::exists(vtCombosFile)) {
        std::ifstream in_tv(vtCombosFile, std::ofstream::binary);
        vIdx_tIdxes = hps::from_stream<std::vector<std::vector<std::pair<int, std::pair<int, int>>>>>(in_tv);
    }
}

void RTVGraph::deserialize_valid_trips() {
    deserialize_current_combos();
    std::string vehTripsPath = outDir + "Misc/Trips";
    for (int tripSize = 2; tripSize < max_trip_size; tripSize++) {
        std::string vehTripsFile = vehTripsPath + "/trips_valid_" + to_string(tripSize) + ".hps";
//       #pragma omp parallel for default(none) private(thread) shared(tripSize, treeCost)
        if (!std::filesystem::exists(vehTripsFile)) continue;
        std::ifstream in_file(vehTripsFile, std::ofstream::binary);
        auto parsed = hps::from_stream<std::vector<std::pair<int, std::pair<int, int>>>>(in_file);
        if (parsed.size() == 0) break;
        for (int i = 0; i < parsed.size(); i++) {
            add_edge_trip_vehicle(parsed[i].second.first, parsed[i].first, parsed[i].second.second);
        }
    }
    std::filesystem::remove_all(vehTripsPath);
    std::filesystem::create_directories(vehTripsPath + "/");
    for (auto it = trip_tIdx.begin(); it != trip_tIdx.end(); ++it) {
        for (auto it2 = it->first.begin(); it2 != it->first.end(); ++it2) {
            rId_tIdxes[*it2].insert(it->second);
        }
    }
}

void RTVGraph::greedy_assign_same_trip_size(vector<vector<pair<int, pair<int, int>>>::iterator>& edgeIters, vector<vector<pair<int, pair<int, int>>>::iterator>& edgeEnds, vector<int>& tIdxes, set<int>& assignedRIds, set<int>& assignedVIdxes, GRBVar** epsilon, std::map<int, map<int, int>>& lookupRtoT) {
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
        for (auto iterRId = trips[tIdx].begin(); iterRId != trips[tIdx].end(); ++iterRId) {
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
                for (auto iterRId = trips[tIdx].begin(); iterRId != trips[tIdx].end(); ++iterRId) {
                    assignedRIds.insert(*iterRId);
                }
                assignedVIdxes.insert(vIdx);
            }
            catch (GRBException& e) {
                print_line(outDir, logFile, string_format("Gurobi exception code: %d.", e.getErrorCode()));
                print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
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
    vector<vector<tripCandidate>>().swap(allPotentialTrips);
    allPotentialTrips.push_back(vector<tripCandidate>{});
    numRequests = requests.size();
    numTrips = 0;
    numVehicles = 0;

    TIdxComparable::rtvGraph = this;

    auto thisTime = std::chrono::system_clock::now();
    build_potential_trips(rvGraph, requests, vehicles);
    std::chrono::duration<double> elapsed_seconds = (std::chrono::system_clock::now() - thisTime);
    print_line(outDir, logFile, string_format("Potential trips build and serialization time = %f.", elapsed_seconds.count()));
    thisTime = std::chrono::system_clock::now();
    deserialize_valid_trips();
    elapsed_seconds = (std::chrono::system_clock::now() - thisTime);
    print_line(outDir, logFile, string_format("Potential trips deserialization time = %f.", elapsed_seconds.count()));
    thisTime = std::chrono::system_clock::now();

    /*    int m;
        const int vehsize = vehicles.size();
        print_line(outDir,logFile,std::string("Starting parallel section."));
        #pragma omp parallel for default(none) private(m) shared(vehicles, rvGraph, requests, vehIDToVehIdx, allPotentialTrips, treeCost)
        for (m=0; m < vehsize; m++) {
            if (rvGraph->has_vehicle(m)) {
                build_single_vehicle(m, vehIDToVehIdx[m], vehicles, rvGraph, requests);
            }
        } */
    vector<vector<tripCandidate>>().swap(allPotentialTrips);
    elapsed_seconds = (std::chrono::system_clock::now() - thisTime);
    print_line(outDir, logFile, string_format("Vehicle-specific RTV graph build time = %f.", elapsed_seconds.count()));
}

void RTVGraph::rebalance(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved, bool bEquityVersion) {
    vector<int> idleVIds;
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].get_num_passengers() == 0)  idleVIds.push_back(i);
    }
    /*
    map<int, vector<pair<int, int>>> relocCosts;
    set<int> servableUnserved;
    for (int i = 0; i < idleVIds.size(); i++) {
        for (int j = 0; j < unserved.size(); j++) {
            Request* reqs[1];
            reqs[0] = &unserved[j];
            TravelHelper th;
            int cost = th.travel(vehicles[idleVIds[i]], reqs, 1, false, false);
            if (cost != -1) {
                relocCosts[i].push_back(make_pair(j, cost));
                servableUnserved.insert(j);
            }
        }
    }

    vector<int> reqIndices(servableUnserved.begin(), servableUnserved.end());
    */
    uos newlyServed;
    int assignedCnt = 0;

    int idleCnt = (int)idleVIds.size();
    int unservedCnt = (int)unserved.size();
    set<int> servableUnserved;
    if (idleCnt > 0 && unservedCnt > 0) {
        try {
            GRBModel model = GRBModel(*env);
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
                bool bAdded = false;
                for (int j = 0; j < unservedCnt; j++) {
                    Request* reqs[1];
                    reqs[0] = &unserved[j];
                    TravelHelper th;
                    int cost = th.travel(vehicles[idleVIds[i]], reqs, 1, false, false);
                    if (cost != -1) {
                        bAdded = true;
                        objective += y[i][j] * cost;
                        servableUnserved.insert(j);
                    }
                    else {
                        y[i][j].set(GRB_DoubleAttr_UB, 0.0);
                        objective += y[i][j] * 0;
                    }
                    totalEdgesCnt += y[i][j];
                    vEdgesCnt += y[i][j];
                    rEdgesCnt[j] += y[i][j];
                }
                if (bAdded) model.addConstr(vEdgesCnt <= 1.0 + minimal);
                else idleCnt--;
            }
            for (int j = 0; j < unservedCnt; j++) {
                model.addConstr(rEdgesCnt[j] <= 1.0 + minimal);
            }
            unservedCnt = servableUnserved.size();
            model.addConstr(totalEdgesCnt == min(idleCnt, unservedCnt));
            model.setObjective(objective, GRB_MINIMIZE);

            model.set("Threads", to_string(omp_get_max_threads() - 2));
            model.set("Method", "1");
            model.set("TimeLimit", "300.0");
            model.set("MIPGap", "0.001");
            model.set("NodeFileStart", "0.5");
            std::string grbLogName = outDir + "GurobiLogs/" + "rebalance1_" + std::to_string(now_time);
            model.set("LogFile", grbLogName + ".txt");
            model.set("ResultFile", grbLogName + ".ilp");
            model.optimize();
            std::string part1 = std::to_string(model.get(GRB_IntAttr_Status));
            std::string part2 = std::to_string((int)std::round(model.get(GRB_DoubleAttr_Runtime)));
            for (int i = 0; i < idleCnt; i++) {
                bool idle = true;
                for (int j = 0; j < unservedCnt; j++) {
                    double val = y[i][j].get(GRB_DoubleAttr_X);
                    if (val < 1.0 + minimal && val > 1.0 - minimal) {
                        Request* reqs[1];
                        reqs[0] = &unserved[j];
                        TravelHelper th;
                        if (-1 != th.travel(vehicles[idleVIds[i]], reqs, 1, true, false)) {
                            for (auto it = vehicles[idleVIds[i]].passengers.begin(); it != vehicles[idleVIds[i]].passengers.end(); ++it) {
                                if (it->unique == reqs[0]->unique) {
                                    int delay = it->scheduledOffTime - it->expectedOffTime;
                                    int wait = it->scheduledOnTime - it->reqTime;
                                    if (delay > max_delay_sec || wait > max_wait_sec) {
                                        it->allowedDelay = delay + max_delay_sec;
                                        it->allowedWait = wait + max_wait_sec;
                                    }
                                    break;
                                }
                            }
                            newlyServed.emplace(j);
                            idle = false;
                            assignedCnt++;
                        }
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
        catch (GRBException& e) {
            print_line(outDir, logFile, string_format("Gurobi exception code: %d.", e.getErrorCode()));
            print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
        }
    }

    std::vector<Request> stillUnserved;
    stillUnserved.reserve(unservedCnt - assignedCnt);
    for (int i = 0; i < unservedCnt; i++) {
        if (newlyServed.find(i) == newlyServed.end()) {
            stillUnserved.push_back(unserved[i]);
        }
    }
    stillUnserved.swap(unserved);
    std::vector<Request>().swap(stillUnserved);

    //Loosen definition of "idle" to those dropping off a passenger before the next time window
    std::vector<int>().swap(idleVIds);
    idleVIds.reserve(vehicles.size() - assignedCnt); //this is reserving too much, but that's OK
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].get_num_passengers() == 0) {
            idleVIds.push_back(i);
            continue;
        }

        for (int j = 0; j < vehicles[i].get_num_passengers(); j++) {
            if (vehicles[i].passengers[j].scheduledOffTime < now_time + time_step) {
                idleVIds.push_back(i);
                break;
            }
        }
    }
    idleVIds.shrink_to_fit();

    idleCnt = (int)idleVIds.size();
    unservedCnt = (int)unserved.size();

    if (idleCnt > 0 && unservedCnt > 0) {
        GRBModel model2 = GRBModel(*env);
        GRBVar** y = new GRBVar * [idleCnt];
        for (int i = 0; i < idleCnt; i++) {
            y[i] = model2.addVars(unservedCnt, GRB_BINARY);
        }

        GRBLinExpr objectiveMain = 0;
        GRBLinExpr objectiveCnt = 0;
        GRBLinExpr totalEdgesCnt = 0;
        GRBLinExpr* rEdgesCnt = new GRBLinExpr[unservedCnt];
        for (int j = 0; j < unservedCnt; j++) {
            rEdgesCnt[j] = 0;
        }
        set<int> servableUnserved;
        for (int i = 0; i < idleCnt; i++) {
            GRBLinExpr vEdgesCnt = 0;
            bool bAdded = false;
            for (int j = 0; j < unservedCnt; j++) {
                Request* reqs[1];
                Request reqCopy = unserved[j];
                Vehicle vehCopy = vehicles[idleVIds[i]];
                reqs[0] = &unserved[j];
                TravelHelper th;
                int cost = th.travel(vehicles[idleVIds[i]], reqs, 1, false, false);
                if (cost != -1) {
                    bAdded = true;
                    objectiveMain += y[i][j] * cost;
                    servableUnserved.insert(j);
                }
                else {
                    y[i][j].set(GRB_DoubleAttr_UB, 0.0);
                    objectiveMain += y[i][j] * 0;
                }
                objectiveCnt += y[i][j];
                totalEdgesCnt += y[i][j];
                vEdgesCnt += y[i][j];
                rEdgesCnt[j] += y[i][j];
            }
            if (bAdded) model2.addConstr(vEdgesCnt <= 1.0 + minimal);
            else idleCnt--;
        }
        for (int j = 0; j < unservedCnt; j++) {
            model2.addConstr(rEdgesCnt[j] <= 1.0 + minimal);
        }
        unservedCnt = servableUnserved.size();
        model2.setObjective(objectiveCnt, GRB_MAXIMIZE);

        model2.set("Threads", to_string(omp_get_max_threads() - 2));
        model2.set("Method", "1");
        model2.set("TimeLimit", "300.0");
        model2.set("MIPGap", "0.001");
        model2.set("NodeFileStart", "0.5");
        std::string grbLogName = outDir + "GurobiLogs/" + "rebalance2_" + std::to_string(now_time);
        model2.set("LogFile", grbLogName + ".txt");
        model2.set("ResultFile", grbLogName + ".ilp");
        model2.optimize();
        double maxAssignments = model2.get(GRB_DoubleAttr_ObjVal);
        model2.addConstr(totalEdgesCnt == maxAssignments);
        model2.setObjective(objectiveMain, GRB_MINIMIZE);
        model2.optimize();

        vector<int> stillIdleVIDs;
        newlyServed.clear();
        assignedCnt = 0;
        for (int i = 0; i < idleCnt; i++) {
            bool idle = true;
            for (int j = 0; j < unservedCnt; j++) {
                double val = y[i][j].get(GRB_DoubleAttr_X);
                if (val < 1.0 + minimal && val > 1.0 - minimal) {
                    Request* reqs[1];
                    reqs[0] = &unserved[j];
                    TravelHelper th;
                    if (-1 != th.travel(vehicles[idleVIds[i]], reqs, 1, true, false)) {
                        for (auto it = vehicles[idleVIds[i]].passengers.begin(); it != vehicles[idleVIds[i]].passengers.end(); ++it) {
                            if (it->unique == reqs[0]->unique) {
                                int delay = it->scheduledOffTime - it->expectedOffTime;
                                int wait = it->scheduledOnTime - it->reqTime;
                                if (delay > max_delay_sec || wait > max_wait_sec) {
                                    it->allowedDelay = delay + max_delay_sec;
                                    it->allowedWait = wait + max_wait_sec;
                                }
                                break;
                            }
                        }
                        newlyServed.emplace(j);
                        idle = false;
                        assignedCnt++;
                    }
                    break;
                }
            }
            if (idle == true) {
                stillIdleVIDs.push_back(idleVIds[i]);
            }
        }

        delete[] rEdgesCnt;
        for (int i = 0; i < idleCnt; i++) {
            delete[] y[i];
        }
        delete[] y;
    }

    if (unservedCnt - assignedCnt > 0) stillUnserved.reserve(unservedCnt - assignedCnt);
    for (int i = 0; i < unservedCnt; i++) {
        if (newlyServed.find(i) == newlyServed.end()) {
            stillUnserved.push_back(unserved[i]);
        }
    }
    stillUnserved.swap(unserved);
    std::vector<Request>().swap(stillUnserved);

    int unlocatedFreeVehicles = 0;
    int totalFreeVehicles = 0;
    int totalDemand = 0;
    int areaCount = treeCost.area_medoids.size();
    vector<int> area_demand(areaCount, 0);
    vector<double> scaled_demand(areaCount, 0.0);
    vector<double> area_rewards(areaCount, 0.0);
    vector<int> area_shortages(areaCount, 0);
    vector<int> area_vehicles(areaCount, 0);
    //Count forecasted demand and unserved trips
    for (int i = 0; i < unserved.size(); i++) {
        if (newlyServed.find(i) != newlyServed.end()) continue;
        int thisArea = treeCost.node_areas[unserved[i].start - 1];
        area_shortages[thisArea - 1]++;
    }
    for (int i = 0; i < areaCount; i++) {
        auto it = treeCost.area_forecasts.find(make_pair(now_time, i + 1));
        if (it == treeCost.area_forecasts.end()) continue;
        const vector<int>& thisForecast = it->second;
        area_demand[i] = thisForecast[3] + area_shortages[i]; //30-minute forecast
        totalDemand += area_demand[i];
    }
    //Count available cars
    idleVIds.clear();
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].get_num_passengers() > 0 && vehicles[i].scheduledPath.size() == 0) {
            int  x = 5;
        }
        if (!vehicles[i].isAvailable() || 
            (vehicles[i].get_num_passengers() > 0 && vehicles[i].scheduledPath.back().first >= now_time + time_step)) continue;

        idleVIds.push_back(i);
        totalFreeVehicles++;
        if (vehicles[i].getAvailableSince() == -9999 && vehicles[i].get_num_passengers() == 0) {
            unlocatedFreeVehicles++;
        }
        else {
            int thisArea = -1;
            if (vehicles[i].get_num_passengers() == 0) {
                thisArea = treeCost.node_areas[vehicles[i].get_location() - 1];
            }
            else if(vehicles[i].scheduledPath.back().first < now_time + time_step) {
                thisArea = treeCost.node_areas[vehicles[i].scheduledPath.back().second - 1];
            }
            area_vehicles[thisArea - 1]++;
        }
    }
    //Enough unrestricted vehicles OR no free vehicles to relocate
    if (unlocatedFreeVehicles >= totalDemand || unlocatedFreeVehicles == totalFreeVehicles) {
        return;
    }
    //Scale demand down if the forecast is larger than available vehicles
    double scaler = (totalFreeVehicles - unlocatedFreeVehicles) * 1.0 / (totalDemand - unlocatedFreeVehicles);
    bool scaleDown = scaler < 1 && scaler > 0;
    for (int i = 0; i < scaled_demand.size(); i++) {
        scaled_demand[i] = scaleDown ? static_cast<double>(area_demand[i] * 1.0 * scaler) : static_cast<double>(area_demand[i]);
        area_rewards[i] = bEquityVersion ? 1.0 / scaled_demand[i] : 1.0;
    }

    for (int i = 0; i < area_shortages.size(); i++) {
        area_shortages[i] = area_vehicles[i] - area_demand[i];
    }
    idleCnt = idleVIds.size();
    try{
        GRBModel model3 = GRBModel(*env);
        GRBVar** y = new GRBVar * [idleCnt+1];
        for (int i = 0; i < idleCnt; i++) {
            y[i] = model3.addVars(areaCount, GRB_BINARY);
        }

        GRBLinExpr objectiveMain = 0;
        GRBLinExpr objectiveCnt = 0;
        GRBLinExpr totalEdgesCnt = 0;
        GRBLinExpr* aEdgesCnt = new GRBLinExpr[areaCount];
        GRBLinExpr* aEdgesReward = new GRBLinExpr[areaCount];
        for (int j = 0; j < areaCount; j++) {
            aEdgesCnt[j] = 0;
            aEdgesReward[j] = 0;
        }
        for (int i = 0; i < idleCnt; i++) {
            int thisId = idleVIds[i];
            GRBLinExpr vEdgesCnt = 0;
            bool bAdded = false;
            int startNode = vehicles[thisId].get_num_passengers() == 0 ? vehicles[thisId].get_location() : vehicles[thisId].scheduledPath.back().second;
            int startArea = treeCost.node_areas[startNode - 1];
            bool bUnlocated = vehicles[i].getAvailableSince() == -9999 && vehicles[i].get_num_passengers() == 0; 
            for (int j = 0; j < areaCount; j++) {
                int cost = 0;
                if (startArea != j + 1) {
                    cost = treeCost.get_dist(startNode, treeCost.area_medoids[j]).first;
                }
                objectiveMain += y[i][j] * cost;
                totalEdgesCnt += y[i][j];
                vEdgesCnt += y[i][j];
                aEdgesCnt[j] += y[i][j];
            }
            model3.addConstr(vEdgesCnt <= 1.0 + minimal);
        }
        for (int j = 0; j < areaCount; j++) {
            if (bEquityVersion) {
                model3.addConstr(objectiveCnt <= aEdgesReward[j] + minimal);
                model3.addConstr(aEdgesReward[j] <= aEdgesCnt[j]/area_demand[j] + minimal);
                model3.addConstr(aEdgesReward[j] <= 1.0 + minimal);
            }
            else {
                //Efficiency: just serve as many as possible, at lowest cost
                objectiveCnt += aEdgesReward[j];
                model3.addConstr(aEdgesReward[j] <= area_demand[j] + minimal);
                model3.addConstr(aEdgesReward[j] <= aEdgesCnt[j] + minimal);
            }
        }
        model3.setObjective(objectiveCnt, GRB_MAXIMIZE);

        model3.set("Threads", to_string(omp_get_max_threads() - 2));
        model3.set("Method", "1");
        model3.set("TimeLimit", "300.0");
        model3.set("MIPGap", "0.001");
        model3.set("NodeFileStart", "0.5");
        std::string grbLogName = outDir + "GurobiLogs/" + "rebalance3_" + std::to_string(now_time);
        model3.set("LogFile", grbLogName + ".txt");
        model3.set("ResultFile", grbLogName + ".ilp");
        model3.optimize();

        double maxReward = model3.get(GRB_DoubleAttr_ObjVal);
        model3.addConstr(totalEdgesCnt == maxReward);
        model3.setObjective(objectiveMain, GRB_MINIMIZE);
        model3.optimize();

        assignedCnt = 0;
        for (int i = 0; i < idleCnt; i++) {
            int thisId = idleVIds[i];
            if (vehicles[thisId].get_num_passengers() == 0 && vehicles[thisId].getAvailableSince() == -9999) continue;
            for (int j = 0; j < areaCount; j++) {
                double val = y[i][j].get(GRB_DoubleAttr_X);
                if (val < 1.0 + minimal && val > 1.0 - minimal) {
                    int startNode = vehicles[thisId].get_num_passengers() == 0 ? vehicles[thisId].get_location() : vehicles[thisId].scheduledPath.back().second;
                    int startArea = treeCost.node_areas[startNode - 1];
                    if (startArea != j + 1) {
                        assignedCnt++;
                        int beginTime = vehicles[thisId].scheduledPath.size() == 0 ? vehicles[thisId].getAvailableSince() : vehicles[thisId].scheduledPath.back().first;
                        int passedDist = 0;
                        vector<pair<int, int> > finalPath;
                        vector<int> order;
                        treeCost.find_path(startNode - 1, treeCost.area_medoids[j] - 1, order);
                        order[0] += 1;
                        for (int m = 1; m < order.size(); m++) {
                            order[i] += 1;
                            passedDist += treeCost.get_dist(order[i - 1], order[i]).second;
                            vehicles[thisId].scheduledPath.push(make_pair(beginTime + ceil(static_cast<double>(passedDist) / velocity * 1.0), order[i]));
                        }
                    }
                    break;
                }
            }
        }

        delete[] aEdgesCnt;
        for (int i = 0; i < idleCnt; i++) {
            delete[] y[i];
        }
        delete[] y;
    }
    catch (GRBException& e) {
        print_line(outDir, logFile, string_format("Gurobi exception code: %d.", e.getErrorCode()));
        print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
    }
}

void RTVGraph::sort_edges() {
    print_line(outDir, logFile, string_format("Starting to sort edges = %f.", 1));
    auto thisTime = std::chrono::system_clock::now();
    std::vector<TIdxComparable> indices;
    indices.reserve(tIdx_vCostIdxes.size());
    std::transform(begin(tIdx_vCostIdxes), end(tIdx_vCostIdxes), std::back_inserter(indices), [](auto const& pair) { return pair.first; });
    std::chrono::duration<double> elapsed_seconds = (std::chrono::system_clock::now() - thisTime);
    print_line(outDir, logFile, string_format("tIdx_vCostIdxes transform time = %f.", elapsed_seconds.count()));
    thisTime = std::chrono::system_clock::now();
    int i;
#pragma omp parallel for private(i)
    for (i = 0; i < indices.size(); i++) {
        auto iter = tIdx_vCostIdxes.find(indices[i]);
        if (iter != tIdx_vCostIdxes.end()) {
            sort(iter->second.begin(), iter->second.end());
            iter->second.resize(min(static_cast<int>(iter->second.size()), max_v_per_req));
            iter->second.shrink_to_fit();
        }
    }
    std::vector<TIdxComparable>().swap(indices);
    elapsed_seconds = (std::chrono::system_clock::now() - thisTime);
    print_line(outDir, logFile, string_format("tIdx_vCostIdxes sorting time = %f.", elapsed_seconds.count()));
    thisTime = std::chrono::system_clock::now();

#pragma omp parallel for private(i)
    for (i = 0; i < vIdx_tIdxes.size(); i++) {
        sort(vIdx_tIdxes[i].begin(), vIdx_tIdxes[i].end());
        vIdx_tIdxes[i].resize(min(static_cast<int>(vIdx_tIdxes[i].size()), min_req_per_v));
        vIdx_tIdxes[i].shrink_to_fit();
    }
    elapsed_seconds = (std::chrono::system_clock::now() - thisTime);
    print_line(outDir, logFile, string_format("vIdx_tIdxes sorting time = %f.", elapsed_seconds.count()));
}

void RTVGraph::prune() {
    map_of_pairs tvCombos;
    tvCombos.reserve(max_v_per_req * tIdx_vCostIdxes.size() + min_req_per_v * vIdx_tIdxes.size());
    for (auto iter = tIdx_vCostIdxes.begin(); iter != tIdx_vCostIdxes.end(); ++iter) {
        //tvCombos.insert(iter->second.begin(), iter->second.end());
        for (int j = 0; j < (*iter).second.size(); j++) {
            tvCombos.emplace(std::pair<int, int>{(*iter).first.tIdx, (*iter).second[j].second.second}, (*iter).second[j].first); //{trip, vehicle}, cost
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
            iter->second.reserve(max_v_per_req + min_req_per_v);
        }
    }

    i = 0;
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

    i = 0;
#pragma omp parallel for private(i)
    for (i = 0; i < indices.size(); i++) {
        auto iter = tIdx_vCostIdxes.find(indices[i]);
        if (iter != tIdx_vCostIdxes.end()) {
            iter->second.shrink_to_fit();
        }
    }
    std::vector<TIdxComparable>().swap(indices);

    i = 0;
#pragma omp parallel for private(i)
    for (i = 0; i < vIdx_tIdxes.size(); i++) {
        vIdx_tIdxes[i].shrink_to_fit();
    }
    print_ram(outDir, string_format("%d_after_prune", now_time));
}

void RTVGraph::solve(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& requests, vector<Request>& unservedCollector) {

    int prevSize = 0, prevSize2 = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); i++)
    {
        prevSize += vIdx_tIdxes[i].size();
    }
    for (int i = 0; i < tIdx_vCostIdxes.size(); i++) {
        prevSize2 += tIdx_vCostIdxes[i].size();
    }
    auto startOfPrune = std::chrono::system_clock::now();
    // printf("begin to sort edges\n");
    sort_edges();
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - startOfPrune;
    print_line(outDir, logFile, string_format("RTV edge sorting time = %f.", elapsed_seconds.count()));
    int sortedSize = 0, sortedSize2 = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); i++)
    {
        sortedSize = sortedSize + vIdx_tIdxes[i].size();
    }
    for (int i = 0; i < tIdx_vCostIdxes.size(); i++) {
        sortedSize2 += tIdx_vCostIdxes[i].size();
    }
    prune();
    int prunedSize = 0, prunedSize2 = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); i++)
    {
        prunedSize = prunedSize + vIdx_tIdxes[i].size();
    }
    for (int i = 0; i < tIdx_vCostIdxes.size(); i++) {
        prunedSize2 += tIdx_vCostIdxes[i].size();
    }
    elapsed_seconds = std::chrono::system_clock::now() - startOfPrune;
    print_line(outDir, logFile, string_format("RTV size trimmed from %d & %d -> %d & %d -> %d & %d in a total of %f seconds.",
        prevSize, prevSize2, sortedSize, sortedSize2, prunedSize, prunedSize2, elapsed_seconds.count()));

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
        for (auto iter = thisMap.begin(); iter != thisMap.end(); ++iter) {
            constr += epsilon[iter->first][iter->second];
        }
        model.addConstr(constr <= 1.0 + minimal);
    }
    /* Constraint #2: Each request is served on exactly one trip*/
    // printf("Adding constraint #2 ...\n");
    for (auto iterRV = rId_tIdxes.begin(); iterRV != rId_tIdxes.end(); ++iterRV) {
        GRBLinExpr constr = 0;
        int rId = iterRV->first;
        auto iterTIdx = iterRV->second.begin();
        while (iterTIdx != iterRV->second.end()) {
            int tIdx = *iterTIdx;
            vector<pair<int, pair<int, int>> >& vCostIdxes = tIdx_vCostIdxes[TIdxComparable(tIdx)];
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
    int maxCostTrip = 0;
    vector<vector<pair<int, pair<int, int>> >::iterator> edgeIters, edgeEnds;
    for (auto iterTV = tIdx_vCostIdxes.begin(); iterTV != tIdx_vCostIdxes.end(); ++iterTV) {
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
        vector<pair<int, pair<int, int>> >& vCostIdxes = tIdx_vCostIdxes[TIdxComparable(tIdx)];
        int reqsInTrip = trips[tIdx].size();
        for (int vehIdx = 0; vehIdx < vCostIdxes.size(); vehIdx++) {
            objective += epsilon[tIdx][vehIdx] * (vCostIdxes[vehIdx].first - reqsInTrip * thisPenalty);
        }
    }
    model.set("TimeLimit", "1500.0");
    model.set("MIPGap", "0.001");
    model.set("Method", "3");
    model.set("NodeFileStart", "1.0");
    //model.set("Presolve","1");
    //model.set("VarBranch","3");
    //model.set("MIPFocus","3");
    std::string nt = std::to_string(now_time);
    std::string grbLogName = outDir + "GurobiLogs/" + "rtv_solve_" + nt;
    model.set("LogFile", grbLogName + ".txt");
    model.setObjective(objective, GRB_MINIMIZE);

    // printf("Solving..
    // solve.\n");
    try {
        model.optimize();
    }
    catch (GRBException e) {
        print_line(outDir, logFile, string_format("Gurobi exception code: %d.", e.getErrorCode()));
        print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
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
        for (auto iter = thisMap.begin(); iter != thisMap.end(); ++iter) {
            int tIdx = iter->first;
            double val = epsilon[tIdx][iter->second].get(GRB_DoubleAttr_X);
            if (val < 1.0 + minimal && val > 1.0 - minimal) {
                // printf("Vehicle #%d: [", vIds[vIdx]);
                Vehicle& vehicle = vehicles[vIds[vIdx]];
                thisVehicleCount = vehicle.get_num_passengers();
                Request* reqs[max_trip_size];
                int tripSize = 0;
                for (auto iter2 = trips[tIdx].begin(); iter2 != trips[tIdx].end(); ++iter2) {
                    // printf(", %d", requests[*iter].unique);
                    reqs[tripSize++] = &requests[*iter2];
                    reqsServed.emplace(*iter2);
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

