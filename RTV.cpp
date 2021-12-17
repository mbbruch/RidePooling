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
    vIdx_tIdxes.push_back(vector<pair<int, pair<int, int>> >());
    return numVehicles++;
}

void RTVGraph::add_edge_trip_vehicle(const uos& reqsInTrip, int vIdx, int cost) {
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

    int toReturn = 0;
    trip_tIdx.emplace(trip, numTrips);
    trips.push_back(trip);
    for (auto iterRIdx = trip.begin(); iterRIdx != trip.end(); ++iterRIdx) {
        rId_tIdxes[*iterRIdx].insert(numTrips);
    }
    toReturn = numTrips++;
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
    for (int i = 0; i < nVeh; i++) {
        if (rvGraph->has_vehicle(i)) {
            int vIdx = addVehicleId(i);
            if (vehicles[vIdx].getAvailableSince() == -9999) {
                bUnusedVehicle = true;
            }
            vehIDToVehIdx[i] = vIdx;
            const map<int, int>& edges = rvGraph->get_vehicle_edges(i); //req, cost
            for (auto it = edges.begin(); it != edges.end(); ++it) {
                uos tempUOS{ it->first };
                add_edge_trip_vehicle(tempUOS, vIdx, it->second);
            }
            int nConnex = edges.size();
            if (nConnex > 0) {
                maxConnections = max(maxConnections, nConnex);
                partlyConnectedVeh++;
                totalConnections += nConnex;
                if (nConnex == nReq) {
                    fullyConnectedVeh++;
                }
                if (nConnex > 1) vehConnex.insert(make_pair(-nConnex, vIdx));
            }
        }
    }

    vector<set<int>> prevInclusions;
    prevInclusions.reserve(numVehicles);
    for (int m = 0; m < numVehicles; m++) prevInclusions.push_back(set<int>());
    allPotentialTrips[0].reserve(requests.size());
    int reqsAdded = 0;
    for (int i = 0; i < requests.size(); i++) {
        auto it = rId_tIdxes.find(i);
        if (it != rId_tIdxes.end()) {
            allPotentialTrips[0].push_back(tripCandidate(uos{ i }, *(it->second.begin())));
        }
        else {
            allPotentialTrips[0].push_back(tripCandidate(uos{ i }));
        }

        auto it2 = rvGraph->req_cost_car.find(i);
        if (it2 != rvGraph->req_cost_car.end()) {
            const std::vector<pair<int, int> >& thisReqCosts = it2->second;
            for (int j = 0; j < thisReqCosts.size(); j++) {
                prevInclusions[vehIDToVehIdx[thisReqCosts[j].second]].insert(i);
            }
        }
    }

    const bool bFullyConnectedVeh = fullyConnectedVeh > 0 ? true : false;
    const double bFractionConnected = fullyConnectedVeh / nVeh;
    const double sparsity = totalConnections / (1.0 * (partlyConnectedVeh * nReq));
    print_line(outDir, logFile, string_format("%d vehicles, %d requests, %d fully connected vehicles, %d most connected (%f), %f percent dense.", nVeh, nReq, fullyConnectedVeh, maxConnections, maxConnections * 100 / nReq, 100*sparsity));
    const bool bSparseTwo = (!bFullyConnectedVeh) && (sparsity < 0.8);
    const bool bSparseThree = (!bFullyConnectedVeh) && (sparsity < 0.001);

    map<int, int> adjustedTripIdxes;
    auto time2 = std::chrono::system_clock::now();
    int travelCounter = 0;
    int lastSizeSize = requests.size();
    int test1counter = 0, test2counter = 0, test3counter = 0;
    for (int k = 2; k <= max_trip_size; k++) {
        /*  Ex: to combine 4 requests, previous entry is 3-way combos,
        of which we need at least 4: 1-2-3, 1-2-4, 1-3-4, 2-3-4 */
        if (allPotentialTrips[k - 2].size() == 0) break;
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
                std::vector<pair<uos, pair<int, uos>>> allCombosOfTwo;
                allCombosOfTwo.reserve(sop.size());
                for (auto it = sop.begin(); it != sop.end(); ++it) {
                    allCombosOfTwo.push_back(pair<uos, pair<int, uos>>{uos{ it->first, it->second }, pair<int, uos>{1, uos{ it->first, it->second }}});
                }
                set_of_pairs().swap(sop);
                newTrips.insert(allCombosOfTwo.begin(), allCombosOfTwo.end());
                std::vector<pair<uos, pair<int, uos>>>().swap(allCombosOfTwo);

                print_line(outDir, logFile, std::string("sparse 2 section finished"));
            }
            else {
                print_line(outDir, logFile, std::string("Enumerating dense 2"));
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
                std::vector<pair<uos, pair<int, uos>>> allCombosOfThree;
                allCombosOfThree.reserve(sop.size());
                for (auto it = sop.begin(); it != sop.end(); ++it) {
                    auto itSop = it->begin();
                    allCombosOfThree.push_back(pair<uos, pair<int, uos>>{*it, pair<int, uos>{0, uos{ *itSop, *(++itSop), *(++itSop) }}}); //TODO: 0 or 1 here???
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
                                #pragma omp critical(updateThree22)
                                (*it).second.first++;
                                #pragma omp critical(updateThree2)
                                (*it).second.second.insert({ indexI, indexJ });
                            }
                        }
                        else {
                            std::pair<int, uos>* it = NULL;
                            #pragma omp critical(updateNewTrips)
                            it = &(newTrips[tripUnion]);
                            #pragma omp critical(updateNewTrips123)
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
        vector<pair<const uos, pair<int, uos>>*> elements;
        if (k > 2) {
            for (auto it = newTrips.begin(); it != newTrips.end(); )
            {   //complete clique requirement: all k subsets of size k-1 must be in here
                if (it->second.second.size() == k && it->second.first >= k * (k - 1) / 2) { ++it; }
                else { newTrips.erase(it++); }
            }
            newTrips.rehash(0);
        }
        elements.reserve(newTrips.size());
        for (auto it = newTrips.begin(); it != newTrips.end(); ++it) {
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
                        if (prevInclusions[itVeh->second].find(*itr) == prevInclusions[itVeh->second].end()) {
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
                    continue;
                }
            }

            vector<Request> copiedRequests;
            for (auto itr = elements[j]->first.begin(); itr != elements[j]->first.end(); ++itr) {
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
            /*            if (!bUnusedVehicle) virtualCar.setAvailableSince(now_time);
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
                            if (th.travel(virtualCar, reqs, k, false, true, true) >= 0) {
                                pathFound = true;
                                break;
                            }
                        } */
            if (pathFound == true) {
                int thisSizeIdx = -1;
#pragma omp critical (updateprior1)
                {
                    allPotentialTrips[k - 1].push_back(tripCandidate(elements[j]->first));
                    thisSizeIdx = thisSizeCounter++;
                }
                for (auto dependentIter = elements[j]->second.second.begin(); dependentIter != elements[j]->second.second.end(); ++dependentIter) {
#pragma omp critical (updateprior2)
                    allPotentialTrips[k - 2][*dependentIter].dependentTrips.push_back(thisSizeIdx);
                }
            }
        }
        vector<uos> dependencies;
        dependencies.reserve(elements.size());

        vector<pair<const uos, pair<int, uos>>*>().swap(elements);
        map_of_uos().swap(newTrips);
        allPotentialTrips[k - 1].shrink_to_fit();
        vector<set<int>> theseInclusions;
        theseInclusions.reserve(numVehicles);
        for (int m = 0; m < numVehicles; m++) {
            theseInclusions.push_back(set<int>());
        }
        print_line(outDir, logFile, "Build_single_vehicles starting.");
        build_single_vehicles(rvGraph, requests, vehicles, vehIDToVehIdx, k, adjustedTripIdxes, prevInclusions, theseInclusions);
        theseInclusions.swap(prevInclusions);
        vector<set<int>>().swap(theseInclusions);

        print_line(outDir, logFile, "Build_single_vehicles done.");
        int m = 0;
        auto itEnd = trip_tIdx.end();
        #pragma omp parallel for default(none) private(m) shared(k, itEnd)
        for (m = 0; m < allPotentialTrips[k - 1].size(); m++) {
            auto it = trip_tIdx.find(allPotentialTrips[k - 1][m].requests);
            if (it != itEnd) {
                allPotentialTrips[k - 1][m].tIdx = it->second;
            }
        }

        print_line(outDir, logFile, "Trips labeled.");
        adjustedTripIdxes.clear();
        int oldIdx = 0;
        int newIdx = 0;
        for (auto it = allPotentialTrips[k - 1].begin(); it != allPotentialTrips[k - 1].end(); ++it)
        {
            if (it->tIdx >= 0) {
                adjustedTripIdxes[oldIdx] = newIdx++;
            }
            oldIdx++;
        }
        int sizeBeforeShrinking = allPotentialTrips[k - 1].size();
        vehConnex.clear();
        for (int i = 0; i < numVehicles; i++) {
            for (auto it = prevInclusions[i].begin(); it != prevInclusions[i].end(); )
            {
                if (adjustedTripIdxes.find(*it) != adjustedTripIdxes.end()) { ++it; }
                else { prevInclusions[i].erase(it++); }
            }
            int connex = prevInclusions[i].size();
            if (connex > k) vehConnex.insert(make_pair(-connex, i));
        } //TODO: remove some vehicles?

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
        //When k-trips are invalidated, delete them (reducing # of links for k-1-trips)
        allPotentialTrips[k - 1].erase(std::remove_if(allPotentialTrips[k - 1].begin(), allPotentialTrips[k - 1].end(),
            [](const tripCandidate& o) { return o.tIdx < 0; }),
            allPotentialTrips[k - 1].end());
        allPotentialTrips[k - 1].shrink_to_fit();
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
        print_line(outDir, logFile, string_format("k-1 trips: of %d, removed %d with 0 dependents and %d with <2 dependents.", temp1, temp2, temp3));
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
            if (it2 == reverseDependentCounts.end() || it2->second < 2) { adjustedTripIdxes.erase(it++); }
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

void RTVGraph::build_single_vehicles(RVGraph* rvGraph, vector<Request>& requests, vector<Vehicle>& vehicles,
    const map<int, int>& vehIDToVehIdx, int tripSize,
    const map<int, int>& adjustedTripIdxes,
    const std::vector<std::set<int>>& prevInclusions, std::vector<std::set<int>>& theseInclusions) {
    if (tripSize < 2) return;
    if (allPotentialTrips.size() < tripSize) return;;
    const std::vector<tripCandidate>& lastSizeVec = allPotentialTrips[tripSize - 2];
    const std::vector<tripCandidate>& thisSizeVec = allPotentialTrips[tripSize - 1];
    if (lastSizeVec.size() < tripSize || thisSizeVec.size() == 0) return;

    int m = 0;
    //#pragma omp parallel for default(none) private(m) schedule(guided) shared(rvGraph, requests, vehicles, vehIDToVehIdx, tripSize, treeCost, lastSizeVec, thisSizeVec, prevInclusions, theseInclusions)
    for (m = 0; m < numVehicles; m++) {
        const set<int>& carPrev = prevInclusions[m];
        if (carPrev.size() < tripSize) continue;
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

                Vehicle& vehicle = vehicles[m];
                TravelHelper th;
                int cost = th.travel(vehicle, reqs, tripSize, false);
                if (cost >= 0) {
                    add_edge_trip_vehicle(thisSizeVec[i].requests, m, cost);
                    theseInclusions[m].insert(i);
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
                    add_edge_trip_vehicle(thisSizeVec[it->first].requests, m, cost);
                    theseInclusions[m].insert(it->first);
                }
            }
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
    print_line(outDir, logFile, string_format("Potential trips build time = %f.", elapsed_seconds.count()));
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
    vector<int> area_demand(treeCost.area_medoids.size(), 0);
    vector<double> scaled_demand(treeCost.area_medoids.size(), 0.0);
    vector<double> area_rewards(treeCost.area_medoids.size(), 0.0);
    vector<int> area_shortages(treeCost.area_medoids.size(), 0);
    vector<int> area_vehicles(treeCost.area_medoids.size(), 0);
    //Count forecasted demand and unserved trips
    for (int i = 0; i < unserved.size(); i++) {
        if (newlyServed.find(i) != newlyServed.end()) continue;
        int thisArea = treeCost.node_areas[unserved[i].start - 1];
        area_shortages[thisArea - 1]++;
    }
    for (int i = 0; i < area_demand.size(); i++) {
        auto it = treeCost.area_forecasts.find(make_pair(now_time, i + 1));
        if (it == treeCost.area_forecasts.end()) continue;
        const vector<int>& thisForecast = it->second;
        area_demand[i] = thisForecast[3] + area_shortages[i]; //30-minute forecast
        totalDemand += area_demand[i];
    }
    //Count available cars
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].get_num_passengers() > 0 || !vehicles[i].isAvailable()) continue;
        totalFreeVehicles++;
        if (vehicles[i].getAvailableSince() == -9999) {
            unlocatedFreeVehicles++;
        }
        else {
            int thisArea = treeCost.node_areas[vehicles[i].get_location() - 1];
            area_vehicles[thisArea - 1]++;
        }
    }
    //Enough unrestricted vehicles OR no free vehicles to relocate
    if (unlocatedFreeVehicles >= totalDemand || totalFreeVehicles == unlocatedFreeVehicles) {
        return;
    }
    //Scale demand down if the forecast is larger than available vehicles
    double scaler = (totalFreeVehicles - unlocatedFreeVehicles) * 1.0 / (totalDemand - unlocatedFreeVehicles);
    bool scaleDown = scaler < 1 && scaler > 0;
    for (int i = 0; i < scaled_demand.size(); i++) {
        scaled_demand[i] = scaleDown ? static_cast<double>(area_demand[i] * 1.0 * scaler) : static_cast<double>(area_demand[i]);
        area_rewards[i] = bEquityVersion ? 1.0 / scaled_demand[i] : 1.0;
    }
    //Efficiency: just serve as many as possible, at lowest cost
    int x = 5;
    if (bEquityVersion) {

    }
    else {

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
    /*
    printf("size of trips: %d %d\n", tIdx_vCostIdxes.size(), trips.size());
    for (iterTV = tIdx_vCostIdxes.begin(); iterTV != tIdx_vCostIdxes.end(); iterTV++) {
    printf("%d, ", trips[iterTV->first.tIdx].size());
    }
    printf("\n");
    */
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

