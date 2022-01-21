#define _HAS_STD_BYTE 0
#include <map>
#include <unordered_set>
#include "util.h"
#include <algorithm>
#include <string>
#include <cstdio>
#include <tuple>
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


std::unordered_set<int> floyd_sampling(const int& k, const int& N) {
    std::default_random_engine gen;
    std::unordered_set<int> elems(k);
    for (int r = N - k; r < N; ++r) {
        int v = std::uniform_int_distribution<>(1, r)(gen);
        if (!elems.insert(v).second) elems.insert(r);
    }
    return elems;
}

void mapAdd(map_of_uos& inout, map_of_uos& in) {
    for (auto initer = in.begin(); initer != in.end(); ++initer) {
        auto it = inout.try_emplace(initer->first, initer->second);
        if (it.second == false) {
            it.first->second.first += initer->second.first;
            it.first->second.second.insert(initer->second.second.begin(), initer->second.second.end());
        }
    }
}

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
    int tIdx = getTIdx(reqsInTrip);
#pragma omp critical (addetv2)
    tIdx_vCostIdxes[tIdx].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfCars(gen1), vIdx}));
    //#pragma omp critical (addetv3)
    vIdx_tIdxes[vIdx].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfTrips(gen2), tIdx}));
}

void RTVGraph::add_edge_trip_vehicle(int tIdx, int vIdx, int cost) {
#pragma omp critical (addetv2)
    tIdx_vCostIdxes[tIdx].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfCars(gen1), vIdx}));
    //#pragma omp critical (addetv3)
    vIdx_tIdxes[vIdx].push_back(pair<int, pair<int, int>>(cost, pair<int, int>{distribOfTrips(gen2), tIdx}));
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
    std::vector<std::pair<int, uos>> prevInclusions;
    prevInclusions.reserve(nVeh);
    int addedToPrevInclusions = 0;
    for (int i = 0; i < nVeh; i++) {
        if (rvGraph->has_vehicle(i)) {
            int vIdx = addVehicleId(i);
            vehIDToVehIdx[i] = vIdx;
            const vector<std::pair<int, int>>& edges = rvGraph->get_vehicle_edges(i); //req, cost
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

    allPotentialTrips[0].reserve(requests.size());
    int reqsAdded = 0;
    for (int i = 0; i < requests.size(); i++) {
        allPotentialTrips[0].push_back(tripCandidate(uos{ i }));
    }

    serialize_current_combos();

    const bool bFullyConnectedVeh = fullyConnectedVeh > 0 ? true : false;
    const double bFractionConnected = fullyConnectedVeh / numVehicles;
    const double sparsity = totalConnections / (1.0 * (partlyConnectedVeh * nReq));
    print_line(outDir, logFile, string_format("%d vehicles, %d requests, %d fully connected vehicles, %d most connected (%f), %f percent dense.", numVehicles, nReq, fullyConnectedVeh, maxConnections, maxConnections * 100.0 / nReq, 100.0 * sparsity));
    const bool bSparseTwo = (!bFullyConnectedVeh) && (sparsity < 0.8);
    //const bool bTesting = true;
    unordered_map<int, int> adjustedTripIdxes;
    auto time2 = std::chrono::system_clock::now();
    int travelCounter = 0;
    int lastSizeSize = requests.size();
    double lastSizeMaxConnectedness = 0;
    double lastSizeAvgConnectedness = 0;
    int test1counter = 0, test2counter = 0, test3counter = 0;
    for (int k = 2; k <= max_trip_size; k++) {
        /*  Ex: to combine 4 requests, previous entry is 3-way combos,
        of which we need at least 4: 1-2-3, 1-2-4, 1-3-4, 2-3-4 */
        if (allPotentialTrips[k - 2].size() < k) break;
        allPotentialTrips.push_back(vector<tripCandidate>{});
        auto startOfSizeK = std::chrono::system_clock::now();
        bool bAltMethod = false;
        int completeCliques = 0;
        map_of_uos newTrips; //key: set of requests (union of two trips);                                      
        //value: indices within allPotentialTrips[k-2]
        //print_line(outDir,logFile,string_format("Starting to build potential %d-request combinations.", k));        
        if (k == 2) {
            if (bSparseTwo) {
                print_line(outDir, logFile, std::string("Enumerating sparse 2"));
                set_of_pairs sop;
                sop.reserve(lastSizeSize * (lastSizeSize - 1) / 2);
                int vIdx2 = 0;
#pragma omp parallel for default(none) private(vIdx2) shared(sop, k, rvGraph)
                for (vIdx2 = 0; vIdx2 < numVehicles; vIdx2++) {
                    auto itv2 = rvGraph->car_req_cost.find(vIds[vIdx2]);
                    if (itv2 == rvGraph->car_req_cost.end()) continue;
                    const vector<std::pair<int, int>>& thisVehVec = itv2->second;
                    int thisSize = thisVehVec.size();
                    std::vector<std::pair<int, int>> vPair;
                    vPair.reserve(itv2->second.size() * (itv2->second.size() - 1) / 2);
                    int i, j = 0;
                    std::pair<int, int> thisSet;
                    for (i = 0; i < thisSize; i++)
                    {
                        thisSet.first = thisVehVec[i].first;
                        for (j = i + 1; j < thisSize; j++) {
                            thisSet.second = thisVehVec[j].first;
                            vPair.push_back(thisSet);
                        }
                    }
#pragma omp critical(insertTripPair)
                    sop.insert(vPair.begin(), vPair.end());
                }

                std::vector<pair<uos, pair<int, uosTBB>>> allCombosOfTwo;
                allCombosOfTwo.reserve(sop.size());
                for (auto it = sop.begin(); it != sop.end(); ++it) {
                    uos thisSet{ { it->first, it->second } };
                    allCombosOfTwo.push_back(pair<uos, pair<int, uosTBB>>{thisSet, pair<int, uosTBB>{1, thisSet}});
                }
                set_of_pairs().swap(sop);
                newTrips.insert(allCombosOfTwo.begin(), allCombosOfTwo.end());
                std::vector<pair<uos, pair<int, uosTBB>>>().swap(allCombosOfTwo);
            }
            else {
                print_line(outDir, logFile, std::string("Enumerating dense 2"));
                std::vector<pair<uos, pair<int, uosTBB>>> allCombosOfTwo;
                allCombosOfTwo.reserve(lastSizeSize * (lastSizeSize - 1) / 2);
                for (int i = 0; i < lastSizeSize; i++) {
                    for (int j = i + 1; j < lastSizeSize; j++) {
                        uos thisSet{ { i, j } };
                        allCombosOfTwo.push_back(pair<uos, pair<int, uosTBB>>{thisSet, pair<int, uosTBB>{1, thisSet}});
                    }
                }
                newTrips.insert(allCombosOfTwo.begin(), allCombosOfTwo.end());
                std::vector<pair<uos, pair<int, uosTBB>>>().swap(allCombosOfTwo);
            }
        }
        else {
            const bool bSparseThree = (!bFullyConnectedVeh) && lastSizeAvgConnectedness < 0.01 && lastSizeMaxConnectedness < 0.75;
            print_line(outDir, logFile, string_format("Starting enumeration: avg connectedness %f, max %f.", lastSizeAvgConnectedness, lastSizeMaxConnectedness));
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
            print_line(outDir, logFile, string_format("Enumerating potential %d-trips, default way.", k));
            const int twoSmallerSize = allPotentialTrips[k - 3].size();
            auto adjustedTripEnd = adjustedTripIdxes.end();
            int m = 0;
#pragma omp parallel for default(none) private(m) shared(k, newTrips, adjustedTripEnd, adjustedTripIdxes)
            for (m = 0; m < twoSmallerSize; m++) {
                const vector<int>& dependentTrips = allPotentialTrips[k - 3][m].dependentTrips;
                int numDependentTrips = dependentTrips.size();
                if (numDependentTrips < 2) continue;
                pair<int, int> dj(-1, -1);
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
                        getDisjunction(trip1, trip2, dj);
                        if (dj.first == -1) continue;
                        uos tripUnion = dj.first == 0 ? trip1 : trip2;
                        tripUnion.insert(dj.second);
#pragma omp critical(updateItNT)
                        {
                            auto itNT = newTrips.try_emplace(tripUnion, make_pair<int, uos>(1, { indexI, indexJ }));
                            if (itNT.second == false) {
                                itNT.first->second.first++;
                                itNT.first->second.second.insert({ indexI, indexJ });
                            }
                        }
                    }
                }
            }
        }
        //put the pointers in the vector


        int newTripsSize = newTrips.size();
        print_line(outDir, logFile, string_format("Starting to check feasibility of %d %d-request combinations.", newTripsSize, k));

        int thisSizeCounter = 0;
        int elementSize = 0;
        vector<pair<const uos, pair<int, uosTBB>>*> elements;
        if (k > 2 && !bAltMethod) {
            for (auto it = newTrips.begin(); it != newTrips.end(); )
            {   //complete clique requirement: all k subsets of size k-1 must be in here
                if (it->second.second.size() == k && it->second.first >= k * (k - 1) / 2) { ++it; }
                else { newTrips.erase(it++); }
            }
            newTrips.rehash(0);
        }
        elements.reserve(newTrips.size());
        for (auto it = newTrips.begin(); it != newTrips.end(); it++) {
            elements.push_back(&(*it));
        }
        elementSize = elements.size();
        print_line(outDir, logFile, string_format("Enumerated %d potential %d-trips.", elementSize, k));
        print_ram(outDir, string_format("%d_after_potential_trip_enumeration_%d", now_time, k));
        int noVeh = 0;
        int j = 0;
//#pragma omp parallel for default(none) private(j) shared(noVeh, bAltMethod, prevInclusions, newTrips, elementSize, elements, requests, k, thisSizeCounter, vehConnex, rvGraph, treeCost, adjustedTripIdxes)
        for (j = 0; j < elementSize; j++) {
            if (!bFullyConnectedVeh && !(bSparseTwo && k == 2) && !bAltMethod) {
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
#pragma omp atomic
                    noVeh++;
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
        print_line(outDir, logFile, string_format("%d elements had no vehicle", noVeh));
        vector<pair<const uos, pair<int, uosTBB>>*>().swap(elements);
        map_of_uos().swap(newTrips);
        allPotentialTrips[k - 1].shrink_to_fit();
        std::vector<int> addedTrips(allPotentialTrips[k - 1].size(), 0);
        vector<std::pair<int, uos>> theseInclusions;
        theseInclusions.reserve(prevInclusions.size());
        for (int m = 0; m < prevInclusions.size(); m++) {
            theseInclusions.push_back(std::pair<int, uos>(prevInclusions[m].first, uos()));
        }
        print_line(outDir, logFile, string_format("Build_single_vehicles starting for %d trips and %d vehicles of prev size (%d of this size).", (int)allPotentialTrips[k - 1].size(), (int)prevInclusions.size(), (int)theseInclusions.size()));
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
        //When k-trips are invalidated, delete them (reducing # of links for k-1-trips)
        for (m = 0; m < sizeBeforeShrinking; m++) {
            if (addedTrips[m] == 1) {
                adjustedTripIdxes[oldIdx] = newIdx++;
            }
            oldIdx++;
        }

        print_line(outDir, logFile, "Adjusted trip idxes created.");

        vector<tripCandidate>& toShrink = allPotentialTrips[k - 1];
        toShrink.erase(std::remove_if(toShrink.begin(), toShrink.end(),
            [&addedTrips, &toShrink](const tripCandidate& o) { return !addedTrips[&o - &*(toShrink.begin())]; }),
            toShrink.end());
        allPotentialTrips[k - 1].shrink_to_fit();

        print_line(outDir, logFile, "allPotentialTrips[k-1] shrunk where no trips were found.");
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
        int temp1 = allPotentialTrips[k - 2].size();
        int apt_i = 0;
#pragma omp parallel for default(none) num_threads(omp_get_max_threads()) private(apt_i) shared(noVeh, prevInclusions, newTrips, elementSize, elements, requests, k, thisSizeCounter, vehConnex, rvGraph, treeCost, adjustedTripIdxes)
        for (apt_i = 0; apt_i < allPotentialTrips[k - 2].size(); apt_i++) { //
            std::vector<int>& theseDependents = allPotentialTrips[k - 2][apt_i].dependentTrips;
            theseDependents.erase(std::remove_if(theseDependents.begin(), theseDependents.end(),
                [&adjustedTripIdxes](const int& o) { return adjustedTripIdxes.find(o) == adjustedTripIdxes.end(); }),
                theseDependents.end());
        }
        print_line(outDir, logFile, "allPotentialTrips[k-2] orphaned dependencies removed.");
        //Remove k-1-trips with fewer than 2 k-trip dependencies
        int temp2 = temp1 - allPotentialTrips[k - 2].size();
        allPotentialTrips[k - 2].erase(std::remove_if(allPotentialTrips[k - 2].begin(), allPotentialTrips[k - 2].end(),
            [](const tripCandidate& o) { return o.dependentTrips.size() < 2; }),
            allPotentialTrips[k - 2].end());
        int temp3 = temp1 - allPotentialTrips[k - 2].size() - temp2;
        print_line(outDir, logFile, string_format("%d-trips: of %d, removed %d with 0 dependents and %d with <2 dependents.", k - 1, temp1, temp2, temp3));
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
        print_line(outDir, logFile, string_format("k-trips: of %d, removed %d with <2 antecedents.", b4adj, b4adj - afteradj));

        if (k >= 3) {
            vector<tripCandidate>().swap(allPotentialTrips[k - 3]);
            allPotentialTrips[k - 3].clear();
            allPotentialTrips[k - 3].shrink_to_fit();
        }
        lastSizeSize = allPotentialTrips[k - 1].size();

        vehConnex.clear();
        auto itPrev = prevInclusions.begin();
        int idx = 0;
        int totalConnex = 0;
        while (itPrev != prevInclusions.end()) {
            for (auto it2 = itPrev->second.begin(); it2 != itPrev->second.end(); )
            {
                if (adjustedTripIdxes.find(*it2) != adjustedTripIdxes.end()) { ++it2; }
                else { it2 = itPrev->second.erase(it2); }
            }
            int connex = itPrev->second.size();
            if (connex == 0) {
                itPrev = prevInclusions.erase(itPrev);
            }
            else {
                if (connex > k) {
                    vehConnex.insert(make_pair(-connex, idx));
                    totalConnex += connex;
                }
                idx++;
                ++itPrev;
            }
        }
        lastSizeMaxConnectedness = vehConnex.size() > 0 ? (-1.0 * (vehConnex.begin()->first)) / (1.0 * lastSizeSize) : 0.0;
        lastSizeAvgConnectedness = vehConnex.size() > 0 ? 1.0 * totalConnex / (1.0 * vehConnex.size()) / (1.0 * lastSizeSize) : 0.0;
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
    const unordered_map<int, int>& adjustedTripIdxes, std::vector<int>& addedTrips,
    const std::vector<std::pair<int, uos>>& prevInclusions, std::vector<std::pair<int, uos>>& theseInclusions) {
    if (tripSize < 2) return;
    if (allPotentialTrips.size() < tripSize) return;;
    const std::vector<tripCandidate>& lastSizeVec = allPotentialTrips[tripSize - 2];
    const std::vector<tripCandidate>& thisSizeVec = allPotentialTrips[tripSize - 1];
    if (lastSizeVec.size() < tripSize || thisSizeVec.size() == 0) return;

    std::vector<std::tuple<int,int,int>> validTripCosts;
    int m = 0;
#pragma omp parallel for default(none) num_threads(omp_get_max_threads()) private(m) schedule(guided) shared(validTripCosts, addedTrips, adjustedTripIdxes, requests, vehicles, vehIDToVehIdx, tripSize, treeCost, lastSizeVec, thisSizeVec, prevInclusions, theseInclusions)
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
        std::vector<std::tuple<int, int, int>> theseValidTripCosts;
        theseValidTripCosts.reserve(carPrev.size() * (carPrev.size() - 1) / 2);
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

                Vehicle vCopy = vehicles[vId];
                TravelHelper th;
                int cost = th.travel(vCopy, reqs, tripSize, false);
                if (cost >= 0) theseValidTripCosts.push_back(std::tuple<int,int,int>(cost, m, i));
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
                if (it->second != tripSize) continue;
                vector<Request> copiedRequests;
                copiedRequests.reserve(thisSizeVec[it->first].requests.size());
                for (auto itr = thisSizeVec[it->first].requests.begin(); itr != thisSizeVec[it->first].requests.end(); ++itr) {
                    copiedRequests.push_back(requests[*itr]);
                }

                Request* reqs[max_trip_size];
                for (int j = 0; j < tripSize; j++) {
                    reqs[j] = &copiedRequests[j];
                }

                Vehicle vCopy = vehicles[vId];
                TravelHelper th;
                int cost = th.travel(vCopy, reqs, tripSize, false);
                if (cost >= 0) theseValidTripCosts.push_back(std::tuple<int, int, int>(cost, m, it->first));
            }
        }
#pragma omp critical(pushingToVec)
        validTripCosts.insert(validTripCosts.end(), theseValidTripCosts.begin(), theseValidTripCosts.end());
    }
    validTripCosts.shrink_to_fit();
    std::sort(validTripCosts.begin(), validTripCosts.end());
    std::vector<int> vAssignments(prevInclusions.size(), -1); //indexed as m above
    std::vector<int> tAssignments(thisSizeVec.size(), -1); //indexed as i above
    std::vector<int> tAssignmentCounts(thisSizeVec.size(), 0);
    std::vector<int> vAssignmentCounts(prevInclusions.size(), 0);
    for (int i = 0; i < validTripCosts.size(); i++) {
        int vIdx = std::get<1>(validTripCosts[i]);
        int tIdx = std::get<2>(validTripCosts[i]);
        if (vAssignmentCounts[vIdx] >= min_req_per_v && tAssignmentCounts[tIdx] >= max_v_per_req && (-1 != vAssignments[vIdx] || -1 != tAssignments[tIdx])) {
            std::get<0>(validTripCosts[i]) = -1;
            continue;
        }
        vAssignmentCounts[vIdx]++;
        tAssignmentCounts[tIdx]++;
        if (-1 == vAssignments[vIdx] && -1 == tAssignments[tIdx]) {
            vAssignments[vIdx] = tIdx;
            tAssignments[tIdx] = vIdx;
        }
    }
    std::vector<int>().swap(vAssignmentCounts);
    std::vector<int>().swap(tAssignmentCounts);
    std::vector<int>().swap(vAssignments);
    std::vector<int>().swap(tAssignments);
    validTripCosts.erase(std::remove_if(validTripCosts.begin(), validTripCosts.end(),
        [](const std::tuple<int, int, int>& o) { return std::get<0>(o) == -1; }),
        validTripCosts.end());
    std::sort(validTripCosts.begin(), validTripCosts.end(),
        [](const std::tuple<int, int, int>& a, const std::tuple<int, int, int>& b) {
            return std::get<2>(a) < std::get<2>(b) || (std::get<2>(a) == std::get<2>(b) && std::get<1>(a) < std::get<1>(b));
        }); //orders ascending by trip then vehicle
    validTripCosts.shrink_to_fit();
    int prevI = -1;
    int prevM = -1;
    int vIdx = -1;
    int tIdx = -1;
    for (int j = 0; j < validTripCosts.size(); j++) {
        int cost = std::get<0>(validTripCosts[j]);
        int m = std::get<1>(validTripCosts[j]); //vehicle
        int i = std::get<2>(validTripCosts[j]); //trip
        if (i != prevI) {
            tIdx = getTIdx(thisSizeVec[i].requests);
            prevI = i;
        }
        if (m != prevM){
            prevM = m;
            auto itVeh = vehIDToVehIdx.find(prevInclusions[m].first);
            if (itVeh == vehIDToVehIdx.end()) { vIdx = -1; continue; }
            else { vIdx = itVeh->second; };
        }
        if (vIdx == -1 || tIdx == -1 || cost == -1) continue;
        addedTrips[i] = 1;
        theseInclusions[m].second.insert(i);
        std::get<1>(validTripCosts[j]) = vIdx;
        std::get<2>(validTripCosts[j]) = tIdx;
    }
    std::string vehTripsFile = outDir + "Misc/Trips/trips_valid_" + to_string(tripSize) + ".hps";
    std::ofstream out_file(vehTripsFile, std::ofstream::binary | std::ios_base::app);
    hps::to_stream(validTripCosts, out_file);
    out_file.close();
    std::vector<std::tuple<int, int, int>>().swap(validTripCosts);
}

void RTVGraph::serialize_current_combos() {
    {
        std::string tvCombosFile = outDir + "Misc/Trips/tvCombos.hps";
        std::ofstream out_tv(tvCombosFile, std::ofstream::binary | std::ios_base::app);
        hps::to_stream(tIdx_vCostIdxes, out_tv);
        out_tv.close();
        map<TIdxComparable, vector<pair<int, pair<int, int>> > >().swap(tIdx_vCostIdxes);
    }
    {
        std::string vtCombosFile = outDir + "Misc/Trips/vtCombos.hps";
        std::ofstream out_vt(vtCombosFile, std::ofstream::binary | std::ios_base::app);
        hps::to_stream(vIdx_tIdxes, out_vt);
        out_vt.close();
        vector<vector<pair<int, pair<int, int>> > >().swap(vIdx_tIdxes);
    }
}

void RTVGraph::deserialize_current_combos() {
    {
        std::string tvCombosFile = outDir + "Misc/Trips/tvCombos.hps";
        if (std::filesystem::exists(tvCombosFile)) {
            std::ifstream in_tv(tvCombosFile, std::ofstream::binary);
            tIdx_vCostIdxes = hps::from_stream <std::map<TIdxComparable, vector<pair<int, pair<int, int>>>>>(in_tv);
        }
    }
    {
        std::string vtCombosFile = outDir + "Misc/Trips/vtCombos.hps";
        if (std::filesystem::exists(vtCombosFile)) {
            std::ifstream in_vt(vtCombosFile, std::ofstream::binary);
            vIdx_tIdxes = hps::from_stream<std::vector<std::vector<std::pair<int, std::pair<int, int>>>>>(in_vt);
        }
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
        auto parsed = hps::from_stream<std::vector<std::tuple<int,int,int>>>(in_file);
        if (parsed.size() == 0) break;
        for (int i = 0; i < parsed.size(); i++) {
            //cost, v, t -> t, v, cost
            add_edge_trip_vehicle(std::get<2>(parsed[i]), std::get<1>(parsed[i]), std::get<0>(parsed[i]));
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

void RTVGraph::greedy_assign_same_trip_size(vector<vector<pair<int, pair<int, int>>>::iterator>& edgeIters, vector<vector<pair<int, pair<int, int>>>::iterator>& edgeEnds, vector<int>& tIdxes, set<int>& assignedRIds, set<int>& assignedVIdxes, GRBVar** epsilon, std::map<int, map<int, int>>& lookupRtoT, unordered_map<int, int>& validTripReverseLookup) {
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
                int vIdxIntoCosts = lookupRtoT[vIdx][validTripReverseLookup[tIdx]];
                epsilon[validTripReverseLookup[tIdx]][vIdxIntoCosts].set(GRB_DoubleAttr_Start, 1.0);
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

void RTVGraph::update_unserved_from_rebalancing(vector<Request>& unserved, uos& newlyServed) {
    std::vector<Request> stillUnserved;
    if (unserved.size() - newlyServed.size() > 0) stillUnserved.reserve(unserved.size() - newlyServed.size());
    for (int i = 0; i < unserved.size(); i++) {
        if (newlyServed.find(i) == newlyServed.end()) {
            stillUnserved.push_back(unserved[i]);
        }
    }
    stillUnserved.swap(unserved);
}

void RTVGraph::rebalance_for_pruning_fix(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved) {
    vector<int> idleVIds;
    uos newlyServed;
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].get_num_passengers() == 0 && !vehicles[i].offline && vehicles[i].getAvailableSince() < now_time + time_step)  idleVIds.push_back(i);
    }
    int idleCnt = (int)idleVIds.size();
    int unservedCnt = (int)unserved.size();
    std::vector<int> rAssignments(unserved.size(), -1);
    std::vector<int> vAssignments(idleVIds.size(), -1);
    std::vector<int> rAssignmentCounts(unserved.size(), 0);
    std::vector<int> vAssignmentCounts(idleVIds.size(), 0);
    std::vector<std::tuple<int, int, int>> allCosts(idleCnt * unservedCnt, std::make_tuple<int, int, int>(-1, -1, -1));
    int i = 0;
    #pragma omp parallel for default(none) num_threads(omp_get_max_threads()) private(i) shared(unserved, vehicles, idleVIds, allCosts, idleCnt, unservedCnt)
    for (i = 0; i < idleCnt; i++) {
        Vehicle vCopy = vehicles[idleVIds[i]];
        for (int j = 0; j < unservedCnt; j++) {
            Request copied = unserved[j];
            TravelHelper th;
            Request* reqs[1];
            reqs[0] = &copied;
            int cost = th.travel(vCopy, reqs, 1, false, false);
            if (cost != -1) {
                int idx = i * unservedCnt + j;
                std::get<0>(allCosts[idx]) = cost;
                std::get<1>(allCosts[idx]) = i;
                std::get<2>(allCosts[idx]) = j;
            }
        }
    }
    allCosts.erase(std::remove_if(allCosts.begin(), allCosts.end(),
        [](const std::tuple<int, int, int>& o) { return std::get<0>(o) == -1; }),
        allCosts.end());
    allCosts.shrink_to_fit();
    std::sort(allCosts.begin(), allCosts.end());
    int assignableCnt = 0;
    for (int i = 0; i < allCosts.size(); i++) {
        int vIdx = std::get<1>(allCosts[i]);
        int rIdx = std::get<2>(allCosts[i]);
        if (vAssignmentCounts[vIdx] >= min_req_per_v && rAssignmentCounts[rIdx] >= max_v_per_req && (-1 != vAssignments[vIdx] || -1 != rAssignments[rIdx])) {
            std::get<0>(allCosts[i]) = -1;
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
    vector<int>().swap(vAssignmentCounts);
    vector<int>().swap(rAssignmentCounts);
    allCosts.erase(std::remove_if(allCosts.begin(), allCosts.end(),
        [](const std::tuple<int, int, int>& o) { return std::get<0>(o) == -1; }),
        allCosts.end());
    std::sort(allCosts.begin(), allCosts.end(),
        [](const std::tuple<int, int, int>& a, const std::tuple<int, int, int>& b) {
            return std::get<1>(a) < std::get<1>(b) || (std::get<1>(a) == std::get<1>(b) && std::get<2>(a) < std::get<2>(b));
        }); //orders ascending by vehicle then request

    print_line(outDir, logFile, string_format("Rebalancing 1: shrunk from %d to %d combos.", idleCnt * unservedCnt, allCosts.size()));
    if (allCosts.size() == 0) return;

    int idleToDelete = idleCnt;
    GRBLinExpr objective = 0;
    GRBLinExpr totalEdgesCnt = 0;
    GRBLinExpr* rEdgesCnt = new GRBLinExpr[unservedCnt];
    GRBLinExpr* vEdgesCnt = new GRBLinExpr[idleCnt];
    GRBModel model = GRBModel(*env);
    GRBVar* y = model.addVars(allCosts.size(), GRB_BINARY); \
        try {
        for (int i = 0; i < unservedCnt; i++) { rEdgesCnt[i] = 0; }
        for (int i = 0; i < idleCnt; i++) { vEdgesCnt[i] = 0; }
        for (int i = 0; i < allCosts.size(); i++) {
            //Order: cost, vIdx, rIdx
            int cost = std::get<0>(allCosts[i]);
            int vIdx = std::get<1>(allCosts[i]);
            int rIdx = std::get<2>(allCosts[i]);
            vEdgesCnt[vIdx] += y[i];
            rEdgesCnt[rIdx] += y[i];
            totalEdgesCnt += y[i];
            objective += y[i] * cost;
            if (rAssignments[rIdx] == vIdx) { y[i].set(GRB_DoubleAttr_Start, 1.0); }
            else { y[i].set(GRB_DoubleAttr_Start, 0.0); }
        }
        for (int i = 0; i < unservedCnt; i++) {
            model.addConstr(rEdgesCnt[i] <= 1.0 + minimal);
        }
        for (int i = 0; i < idleCnt; i++) {
            model.addConstr(vEdgesCnt[i] <= 1.0 + minimal);
        }
        model.addConstr(totalEdgesCnt == assignableCnt);
        model.setObjective(objective, GRB_MINIMIZE);
        model.set("Threads", to_string(omp_get_max_threads() - 2));
        model.set("Method", "1");
        model.set("TimeLimit", "300.0");
        model.set("MIPGap", "0.01");
        model.set("NodeFileStart", "0.5");
        std::string grbLogName = outDir + "GurobiLogs/" + "rebalance1_" + std::to_string(now_time);
        model.set("LogFile", grbLogName + ".txt");
        model.set("ResultFile", grbLogName + ".ilp");
        model.optimize();
        std::string part1 = std::to_string(model.get(GRB_IntAttr_Status));
        std::string part2 = std::to_string((int)std::round(model.get(GRB_DoubleAttr_Runtime)));
        std::ofstream assignments;
        for (int i = 0; i < allCosts.size(); i++) {
            double val = y[i].get(GRB_DoubleAttr_X);
            if (val > 1.0 + minimal || val < 1.0 - minimal) continue;
            int vIdx = idleVIds[std::get<1>(allCosts[i])];
            int rIdx = std::get<2>(allCosts[i]);
            Request* reqs[1];
            reqs[0] = &unserved[rIdx];
            assignments.open(outDir + "Misc/Rebalance_Prune_Fix_Vehs.csv", std::ofstream::out | std::ofstream::app);
            assignments << to_string(now_time) + "," + to_string(vIdx) + "," + to_string(reqs[0]->unique) + "\n";
            assignments.close();
            TravelHelper th;
            if (-1 == th.travel(vehicles[vIdx], reqs, 1, true, false)) continue;
            newlyServed.emplace(rIdx);
            for (auto it = vehicles[vIdx].passengers.begin(); it != vehicles[vIdx].passengers.end(); ++it) {
                if (it->unique != reqs[0]->unique) continue;
                int delay = it->scheduledOffTime - it->expectedOffTime;
                int wait = it->scheduledOnTime - it->reqTime;
                if (delay > max_delay_sec || wait > max_wait_sec) {
                    it->allowedDelay = delay + max_delay_sec;
                    it->allowedWait = wait + max_wait_sec;
                }
                break;
            }
        }
    }
    catch (GRBException& e) {
        print_line(outDir, logFile, string_format("Gurobi exception code in rebalancing 1: %d.", e.getErrorCode()));
        print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
    }
    delete[] rEdgesCnt;
    delete[] vEdgesCnt;
    delete[] y;

    update_unserved_from_rebalancing(unserved, newlyServed);
}

void RTVGraph::rebalance_for_finishing_cars(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved) {
    std::vector<int> idleVIds;
    uos newlyServed;
    //Loosen definition of "idle" to those dropping off a passenger before the next time window
    idleVIds.reserve(vehicles.size()); //this is reserving too much, but that's OK
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].offline || vehicles[i].getAvailableSince() >= now_time + time_step) continue;

        if (vehicles[i].get_num_passengers() == 0) {
            idleVIds.push_back(i);
            continue;
        }

        bool bIdleDropoff = true;
        for (int j = 0; j < vehicles[i].get_num_passengers(); j++) {
            if (vehicles[i].passengers[j].scheduledOffTime >= now_time + time_step) {
                bIdleDropoff = false;
                break;
            }
        }
        if (bIdleDropoff) idleVIds.push_back(i);
    }
    idleVIds.shrink_to_fit();

    int idleCnt = (int)idleVIds.size();
    int unservedCnt = (int)unserved.size();
    int assignedCnt = 0;
    if (idleCnt > 0 && unservedCnt > 0) {
        int idleToDelete = idleCnt;
        GRBModel model2 = GRBModel(*env);
        GRBVar** y = new GRBVar * [idleCnt];
        for (int i = 0; i < idleCnt; i++) {
            y[i] = model2.addVars(unservedCnt, GRB_BINARY);
        }
        GRBLinExpr* rEdgesCnt = new GRBLinExpr[unservedCnt];
        try {
            for (int j = 0; j < unservedCnt; j++) {
                rEdgesCnt[j] = 0;
            }
            GRBLinExpr objectiveMain = 0;
            GRBLinExpr objectiveCnt = 0;
            GRBLinExpr totalEdgesCnt = 0;

            set<int> servableUnserved;
            for (int i = 0; i < idleCnt; i++) {
                GRBLinExpr vEdgesCnt = 0;
                bool bAdded = false;
                for (int j = 0; j < unservedCnt; j++) {
                    Request* reqs[1];
                    Request reqCopy = unserved[j];
                    Vehicle vehCopy = vehicles[idleVIds[i]];
                    reqs[0] = &reqCopy;
                    TravelHelper th;
                    int cost = th.travel(vehCopy, reqs, 1, false, false);
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
            model2.set("MIPGap", "0.005");
            model2.set("NodeFileStart", "0.5");
            std::string grbLogName = outDir + "GurobiLogs/" + "rebalance2_" + std::to_string(now_time);
            model2.set("LogFile", grbLogName + ".txt");
            model2.set("ResultFile", grbLogName + ".ilp");
            model2.optimize();
            double maxAssignments = model2.get(GRB_DoubleAttr_ObjVal);
            model2.addConstr(totalEdgesCnt == maxAssignments);
            model2.setObjective(objectiveMain, GRB_MINIMIZE);
            model2.set("MIPGap", "0.01");
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
        }
        catch (GRBException& e) {
            print_line(outDir, logFile, string_format("Gurobi exception code in rebalancing 2: %d.", e.getErrorCode()));
            print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
        }
        delete[] rEdgesCnt;
        for (int i = 0; i < idleToDelete; i++) {
            delete[] y[i];
        }
        delete[] y;
    }
    update_unserved_from_rebalancing(unserved, newlyServed);
}

void RTVGraph::rebalance_online_offline(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved, int nowTime) {
    int totalDemand = 0;
    int areaCount = treeCost.area_medoids.size();
    auto it = treeCost.city_forecasts_max.find(nowTime);
    if (it == treeCost.city_forecasts_max.end()) return;
    int city_demand_30max = it->second;
    int carsNeeded = city_demand_30max * cars_needed_per_trip_per_30;
    vector<int> area_demand(areaCount, 0);
    vector<double> scaled_demand(areaCount, 0.0);
    vector<double> area_rewards(areaCount, 0.0);
    vector<int> area_shortages(areaCount, 0);
    print_line(outDir, logFile, "1044 ");
    //Count forecasted demand and unserved trips
    for (int i = 0; i < unserved.size(); i++) {
        int thisArea = treeCost.node_areas[unserved[i].start - 1];
        area_shortages[thisArea - 1]++;
    }
    for (int i = 0; i < areaCount; i++) {
        auto it = treeCost.area_forecasts.find(make_pair(nowTime, i + 1));
        if (it == treeCost.area_forecasts.end()) continue;
        const vector<int>& thisForecast = it->second;
        area_demand[i] = thisForecast[3] + area_shortages[i]; //30-minute forecast
        totalDemand += area_demand[i];
    }
    carsNeeded = max(carsNeeded, (int)ceil(totalDemand * cars_needed_per_trip_per_30));
    int carsNotNeeded = vehicles.size() - carsNeeded;
    //Count available cars
    int unlocatedFreeVehicles = 0;
    int totalFreeVehicles = 0;
    // carsNotNeeded > offlineCars: exclude offline cars from optimization, bring some more offline, send to depot
    // carsNotNeeded == offlineCars: do nothing: exclude offline cars from optimization, don't bring any more offline 
    // carsNotNeeded < offlineCars: include all that are en route to home, plus (offline - notneeded) that are at home. these may get routed to specific neighborhoods
    // to mark as offline: available == false, no passengers, availableSince = nowTime + time_step
    int offlineDepot = 0;
    int offlineEnRoute = 0;
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].offline == true) {
            if (vehicles[i].get_location() == vehicle_depot) { offlineDepot++; }
            else { offlineEnRoute++; }
        }
    }
    const int toBringOnline = offlineEnRoute + offlineDepot - carsNotNeeded;
    int toBringOnlineCounter = toBringOnline;
    enum vehStatus { UNINITIALIZED, ONLINE_IDLE, OFFLINE_DEPOT, OFFLINE_ENROUTE };
    std::vector<std::pair<int, vehStatus>> idleVehIDs;
    std::vector<std::pair<int, vehStatus>> offlineDepotVIds;
    std::vector<std::pair<int, vehStatus>> offlineEnrouteVIds;
    print_line(outDir, logFile, "1080 ");
    idleVehIDs.reserve(std::count_if(vehicles.begin(), vehicles.end(), [&nowTime](const Vehicle& v) {
        return ((!v.offline || (v.offline && v.get_location() != vehicle_depot)) && !(v.get_num_passengers() > 0 && !(v.scheduledPath.size() > 0 && v.scheduledPath.back().first >= nowTime + time_step))); }));
    //TODO: this shouldn't check getAvailableSince!!!
    offlineDepotVIds.reserve(std::count_if(vehicles.begin(), vehicles.end(), [&nowTime](const Vehicle& v) {
        return (v.offline && v.get_location() == vehicle_depot && !(v.get_num_passengers() > 0 && !(v.scheduledPath.size() > 0 && v.scheduledPath.back().first >= nowTime + time_step))); }));
    print_line(outDir, logFile, "1086 ");
    offlineEnrouteVIds.reserve(std::count_if(vehicles.begin(), vehicles.end(), [&nowTime](const Vehicle& v) {
        return (v.offline && v.get_location() != vehicle_depot && !(v.get_num_passengers() > 0 && !(v.scheduledPath.size() > 0 && v.scheduledPath.back().first >= nowTime + time_step))); }));
    print_line(outDir, logFile, "1089 ");
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].get_num_passengers() > 0 && vehicles[i].scheduledPath.size() == 0) {
            int x = 5;
        }
        if (vehicles[i].get_num_passengers() > 0 && (vehicles[i].scheduledPath.size() > 0 && vehicles[i].scheduledPath.back().first >= nowTime + time_step)) continue;

        if (vehicles[i].offline == true) {
            if (vehicles[i].get_location() == vehicle_depot) {
                if (vehicles[i].getAvailableSince() == -9999) {
                    if (toBringOnlineCounter-- > 0) offlineDepotVIds.push_back(make_pair(i, vehStatus::UNINITIALIZED));
                }
                else {
                    if (toBringOnlineCounter-- > 0) offlineDepotVIds.push_back(make_pair(i, vehStatus::OFFLINE_DEPOT));
                }
            }
            else {
                offlineEnrouteVIds.push_back(make_pair(i, vehStatus::OFFLINE_ENROUTE));
            }
        }
        else {
            totalFreeVehicles++;
            idleVehIDs.push_back(make_pair(i, vehStatus::ONLINE_IDLE));
        }
    }
    print_line(outDir, logFile, "1144 ");
    if (toBringOnline > 0) {
        int broughtOnline = 0;
        for (int i = 0; i < offlineDepotVIds.size() && broughtOnline <= toBringOnline; i++) {
            vehicles[offlineDepotVIds[i].first].bring_online();
            broughtOnline++;
        }
        int left = toBringOnline - broughtOnline;
        if (left >= offlineEnrouteVIds.size()) {
            for (int i = 0; i < offlineEnrouteVIds.size() && broughtOnline <= toBringOnline; i++) {
                vehicles[offlineEnrouteVIds[i].first].bring_online();
            }
        }
        else {
            print_line(outDir, logFile, string_format("1127: sample %d from %d.", left, offlineEnrouteVIds.size()));
            const std::unordered_set<int>& vehToBringOnline = floyd_sampling(left, offlineEnrouteVIds.size());
            print_line(outDir, logFile, "1129 ");
            for (auto it = vehToBringOnline.begin(); it != vehToBringOnline.end(); ++it) {
                vehicles[offlineEnrouteVIds[*it].first].bring_online();
            }

        }
    }
    else if (toBringOnline < 0) {
        int toTakeOffline = -toBringOnline;
        int takenOffline = 0;
        if (idleVehIDs.size() <= toTakeOffline) {
            for (int i = 0; i < idleVehIDs.size(); i++) {
                vehicles[idleVehIDs[i].first].take_offline();
            }
        }
        else {
            print_line(outDir, logFile, string_format("1143: sample %d from %d.", toTakeOffline, idleVehIDs.size()));
            const std::unordered_set<int>& vehToTakeOffline = floyd_sampling(toTakeOffline, idleVehIDs.size());
            print_line(outDir, logFile, "1145 ");
            for (auto it = vehToTakeOffline.begin(); it != vehToTakeOffline.end(); ++it) {
                vehicles[idleVehIDs[*it].first].take_offline();
            }
        }
    }
    print_line(outDir, logFile, "1151 ");
}


void RTVGraph::rebalance_for_demand_forecasts(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved, int nowTime, bool bEquityVersion) {
    int totalDemand = 0;
    int areaCount = treeCost.area_medoids.size();
    auto it = treeCost.city_forecasts_max.find(nowTime);
    assert(it != treeCost.city_forecasts_max.end());
    int city_demand_30max = it->second;
    int carsNeeded = city_demand_30max * cars_needed_per_trip_per_30;
    vector<int> area_demand(areaCount, 0);
    vector<double> scaled_demand(areaCount, 0.0);
    vector<double> area_rewards(areaCount, 0.0);
    vector<int> area_shortages(areaCount, 0);
    //Count forecasted demand and unserved trips
    for (int i = 0; i < unserved.size(); i++) {
        int thisArea = treeCost.node_areas[unserved[i].start - 1];
        area_shortages[thisArea - 1]++;
    }
    for (int i = 0; i < areaCount; i++) {
        auto it = treeCost.area_forecasts.find(make_pair(nowTime, i + 1));
        if (it == treeCost.area_forecasts.end()) continue;
        const vector<int>& thisForecast = it->second;
        area_demand[i] = thisForecast[3] + area_shortages[i]; //30-minute forecast
        totalDemand += area_demand[i];
    }
    carsNeeded = max(carsNeeded, (int)ceil(totalDemand * cars_needed_per_trip_per_30));
    int carsNotNeeded = vehicles.size() - carsNeeded;
    //Count available cars
    int unlocatedFreeVehicles = 0;
    int totalFreeVehicles = 0;
    // carsNotNeeded > offlineCars: exclude offline cars from optimization, bring some more offline, send to depot
    // carsNotNeeded == offlineCars: do nothing: exclude offline cars from optimization, don't bring any more offline 
    // carsNotNeeded < offlineCars: include all that are en route to home, plus (offline - notneeded) that are at home. these may get routed to specific neighborhoods
    // to mark as offline: available == false, no passengers, availableSince = nowTime + time_step
    int offlineDepot = 0;
    int offlineEnRoute = 0;
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].offline == true) {
            if (vehicles[i].get_location() == vehicle_depot) { offlineDepot++; }
            else { offlineEnRoute++; }
        }
    }
    const int toBringOnline = offlineEnRoute + offlineDepot - carsNotNeeded;
    int toBringOnlineCounter = toBringOnline;
    enum vehStatus { UNINITIALIZED, ONLINE_IDLE, OFFLINE_DEPOT, OFFLINE_ENROUTE };
    std::vector<std::pair<int, vehStatus>> idleVehIDs;
    std::vector<std::pair<int, vehStatus>> offlineDepotVIds;
    std::vector<std::pair<int, vehStatus>> offlineEnrouteVIds;
    idleVehIDs.reserve(std::count_if(vehicles.begin(), vehicles.end(), [&nowTime](const Vehicle& v) {
        return ((!v.offline || (v.offline && v.get_location() != vehicle_depot)) && !(v.get_num_passengers() > 0 && !(v.scheduledPath.size() > 0 && v.scheduledPath.back().first >= nowTime + time_step))); }));
    offlineDepotVIds.reserve(std::count_if(vehicles.begin(), vehicles.end(), [&nowTime](const Vehicle& v) {
        return (v.offline && v.get_location() == vehicle_depot && !(v.get_num_passengers() > 0 && !(v.scheduledPath.size() > 0 && v.scheduledPath.back().first >= nowTime + time_step))); }));
    for (int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i].get_num_passengers() > 0 && vehicles[i].scheduledPath.size() == 0) {
            int x = 5;
        }
        if (vehicles[i].get_num_passengers() > 0 && (vehicles[i].scheduledPath.size() > 0 && vehicles[i].scheduledPath.back().first >= nowTime + time_step)) continue;

        if (vehicles[i].offline == true) {
            if (vehicles[i].get_location() == vehicle_depot) {
                if (vehicles[i].getAvailableSince() == -9999) {
                    if (toBringOnlineCounter-- > 0) offlineDepotVIds.push_back(make_pair(i, vehStatus::UNINITIALIZED));
                }
                else {
                    if (toBringOnlineCounter-- > 0) offlineDepotVIds.push_back(make_pair(i, vehStatus::OFFLINE_DEPOT));
                }
            }
            else {
                if (toBringOnline > 0) idleVehIDs.push_back(make_pair(i, vehStatus::OFFLINE_ENROUTE));
            }
        }
        else {
            totalFreeVehicles++;
            idleVehIDs.push_back(make_pair(i, vehStatus::ONLINE_IDLE));
        }
    }

    //Scale demand down if the forecast is larger than available vehicles
    double scaler = 1.0; // (totalFreeVehicles - unlocatedFreeVehicles) * 1.0 / (totalDemand - unlocatedFreeVehicles);
    bool scaleDown = scaler < 1 && scaler > 0;
    for (int i = 0; i < scaled_demand.size(); i++) {
        scaled_demand[i] = scaleDown ? static_cast<double>(area_demand[i] * 1.0 * scaler) : static_cast<double>(area_demand[i]);
        area_rewards[i] = bEquityVersion ? 1.0 / scaled_demand[i] : 1.0;
    }

    print_line(outDir, logFile, string_format("To bring online: %d.", toBringOnline));
    int idleCnt = idleVehIDs.size();
    GRBModel model3 = GRBModel(*env);
    GRBVar** y = new GRBVar * [idleVehIDs.size()];
    for (int i = 0; i < idleVehIDs.size(); i++) {
        y[i] = model3.addVars(areaCount + 1, GRB_BINARY);
    }
    GRBVar* z = model3.addVars(areaCount, GRB_INTEGER);
    GRBLinExpr* aEdgesCnt = new GRBLinExpr[areaCount + 1];
    GRBVar* aEdgesReward = model3.addVars(areaCount + 1, GRB_CONTINUOUS);
    try {
        GRBLinExpr objectiveMain = 0;
        GRBLinExpr objectiveCnt = 0;
        GRBLinExpr offlineDepotToOnlineCnt = 0;
        GRBLinExpr offlineEnRouteToOnlineCnt = 0;
        GRBLinExpr onlineToOfflineCnt = 0;
        const int offlineDepotAreaIdx = areaCount;
        //GRBLinExpr* aEdgesReward = new GRBLinExpr[areaCount + 1];
        for (int j = 0; j < areaCount + 1; j++) {
            aEdgesCnt[j] = 0;
            //aEdgesReward[j] = 0;
        }
        for (int i = 0; i < idleVehIDs.size(); i++) {
            int thisId = idleVehIDs[i].first;
            GRBLinExpr vEdgesCnt = 0;
            bool bAdded = false;
            int startNode = vehicles[thisId].get_num_passengers() == 0 ? vehicles[thisId].get_location() : vehicles[thisId].scheduledPath.back().second;
            int startArea = treeCost.node_areas[startNode - 1];
            for (int j = 0; j < areaCount + 1; j++) {
                int cost = 0;
                vEdgesCnt += y[i][j];
                aEdgesCnt[j] += y[i][j];
                if (j == offlineDepotAreaIdx) {
                    cost = treeCost.get_dist(startNode, vehicle_depot).first;
                    onlineToOfflineCnt += aEdgesCnt[j];
                }
                else {
                    if (startArea != j + 1) {
                        cost = treeCost.get_dist(startNode, treeCost.area_medoids[j]).first;
                    }
                    if (idleVehIDs[i].second == vehStatus::OFFLINE_ENROUTE || idleVehIDs[i].second == vehStatus::OFFLINE_DEPOT) {
                        offlineEnRouteToOnlineCnt += y[i][j];
                    }
                }
                objectiveMain += y[i][j] * cost;
            }
            model3.addConstr(vEdgesCnt <= 1.0 + minimal);
        }
        if (offlineDepotVIds.size() > 0) {
            for (int j = 0; j < areaCount; j++) {
                aEdgesCnt[j] += z[j];
                offlineDepotToOnlineCnt += z[j];
                int cost = treeCost.get_dist(vehicle_depot, treeCost.area_medoids[j]).first;
                objectiveMain += z[j] * cost;
            }
        }
        else {
            for (int j = 0; j < areaCount; j++) {
                z[j].set(GRB_DoubleAttr_UB, 0.0);
            }
        }
        if (toBringOnline > 0) { //Need to take some online
            model3.addConstr(offlineDepotToOnlineCnt <= min(toBringOnline, (int)offlineDepotVIds.size()));
            model3.addConstr(offlineDepotToOnlineCnt >= 0);
            model3.addConstr(offlineEnRouteToOnlineCnt + offlineDepotToOnlineCnt == min(toBringOnline, (int)(offlineDepotVIds.size() + offlineEnrouteVIds.size())));
        }

        if (toBringOnline < 0) {
            //TODO: handle the cars that were offline and need to stay offline
            model3.addConstr(onlineToOfflineCnt == min(-toBringOnline, totalFreeVehicles)); //TODO: idleVehIDs includes offline enroute!!!!
        }
        for (int j = 0; j < areaCount; j++) {
            if (bEquityVersion) {
                //Equity: maximize the minimum ratio of cars to demand
                model3.addConstr(objectiveCnt <= aEdgesReward[j] + minimal);
                model3.addConstr(aEdgesReward[j] <= aEdgesCnt[j] / (area_demand[j] * cars_needed_per_trip_per_30 + minimal));
                model3.addConstr(aEdgesReward[j] <= 1.0 + minimal);
            }
            else {
                //Efficiency: just serve as many as possible, at lowest cost
                objectiveCnt += aEdgesReward[j];
                model3.addConstr(aEdgesReward[j] <= area_demand[j] * cars_needed_per_trip_per_30 + minimal);
                model3.addConstr(aEdgesReward[j] <= aEdgesCnt[j] + minimal);
            }
        }

        model3.setObjective(objectiveCnt, GRB_MAXIMIZE);

        model3.set("Threads", to_string(omp_get_max_threads() - 2));
        model3.set("Method", "1");
        model3.set("TimeLimit", "300.0");
        model3.set("MIPGap", "0.005");
        model3.set("NodeFileStart", "0.5");
        std::string grbLogName = outDir + "GurobiLogs/" + "rebalance3_" + std::to_string(now_time);
        model3.set("LogFile", grbLogName + ".txt");
        model3.set("ResultFile", grbLogName + ".ilp");
        model3.setObjectiveN(objectiveCnt, 0, 2, -1.0); //MAXIMIZE reward
        model3.setObjectiveN(objectiveMain, 1, 1, 1.0); //MINIMIZE cost
        model3.set(GRB_IntAttr_ModelSense, 1); //MINIMIZATION problem
        model3.optimize();

        /* TODO: handle routing of going offline or online cars, incl edge cases:
            Assigned to go offline previous timestep, but still dropping off a passenger? (Not possible?)
            Was already en route to a given medoid, don't stop just bc you're in the area now*/
        int assignedCnt = 0;
        for (int i = 0; i < idleVehIDs.size(); i++) {
            int thisId = idleVehIDs[i].first;
            //if (vehicles[thisId].get_num_passengers() == 0 && vehicles[thisId].getAvailableSince() == -9999) continue;
            for (int j = 0; j < areaCount + 1; j++) {
                double val = y[i][j].get(GRB_DoubleAttr_X);
                if (val < 1.0 + minimal && val > 1.0 - minimal) {
                    //Has passenger: look at end of scheduledPath, check if equal to area.
                    // If it is, just keep going.
                    // If it isn't, push onto scheduledPath.
                    //Does not have passenger, has a scheduledPath: check if end of scheduled path is equal to area.
                    //If it is, just keep going.
                    //If it isn't, clear scheduled path then push onto it.
                    int startNode = vehicles[thisId].scheduledPath.size() == 0 ? vehicles[thisId].get_location() : vehicles[thisId].scheduledPath.back().second;
                    int startArea = treeCost.node_areas[startNode - 1];
                    int destNode = -1;
                    if (j == offlineDepotAreaIdx) {
                        //needs to go to depot
                        vehicles[thisId].offline = true;
                        destNode = vehicle_depot;
                        if (destNode == startNode) {
                            break; //already headed where it needs to be
                        }
                    }
                    else {
                        vehicles[thisId].offline = false;
                        if (startArea == j + 1) {
                            break; //already headed where it needs to be
                        }
                        else {
                            destNode = treeCost.area_medoids[j];
                        }
                    }

                    assignedCnt++;
                    if (vehicles[thisId].get_num_passengers() == 0) vehicles[thisId].clear_path();
                    int beginTime = (vehicles[thisId].get_num_passengers() == 0 || vehicles[thisId].scheduledPath.size() == 0) ? vehicles[thisId].getAvailableSince() : vehicles[thisId].scheduledPath.back().first;
                    int passedDist = 0;
                    vector<pair<int, int> > finalPath;
                    vector<int> order;

#pragma omp critical (findpath)
                    treeCost.find_path(startNode - 1, destNode - 1, order);
                    order[0] += 1;
                    for (int m = 1; m < order.size(); m++) {
                        order[m] += 1;
                        passedDist += treeCost.get_dist(order[m - 1], order[m]).second;
                        vehicles[thisId].scheduledPath.push(make_pair(beginTime + ceil(static_cast<double>(passedDist) / velocity * 1.0), order[m]));
                    }
                    break;
                }
            }
        }
        print_line(outDir, logFile, string_format("Offline cars that could be used: %d.", offlineDepotVIds.size()));
        auto offlineIt = offlineDepotVIds.begin();
        for (int j = 0; j < areaCount; j++) {
            int val = round(z[j].get(GRB_DoubleAttr_X));
            if (val == 0) continue;

            int startNode = vehicle_depot;
            int startArea = treeCost.node_areas[startNode - 1];
            int destNode = treeCost.area_medoids[j];
            vector<int> order;
            vector<pair<int, int> > finalPath;
#pragma omp critical (findpath)
            treeCost.find_path(startNode - 1, destNode - 1, order);

            for (int k = 0; k < val; k++) {
                assert(offlineIt != offlineDepotVIds.end());
                if (offlineIt == offlineDepotVIds.end()) {
                    assert(false);
                    break;
                }
                int thisId = offlineIt->first;
                ++offlineIt;
                vehicles[thisId].offline = false;

                vehicles[thisId].clear_path();
                int beginTime = vehicles[thisId].getAvailableSince();
                if (beginTime == -9999) {
                    vehicles[thisId].set_location(destNode);
                    vehicles[thisId].available = true;
                    vehicles[thisId].setAvailableSince(nowTime);
                    continue;
                }
                if (startArea == j + 1) {
                    continue; //already headed where it needs to be
                }
                assignedCnt++;
                int passedDist = 0;
                for (int m = 1; m < order.size(); m++) {
                    passedDist += treeCost.get_dist(order[m - 1] + 1, order[m] + 1).second;
                    vehicles[thisId].scheduledPath.push(make_pair(beginTime + ceil(static_cast<double>(passedDist) / velocity * 1.0), order[m] + 1));
                }
            }
        }
    }
    catch (GRBException& e) {
        print_line(outDir, logFile, string_format("Gurobi exception code in rebalancing 3: %d.", e.getErrorCode()));
        print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
    }
    delete[] aEdgesCnt;
    delete[] aEdgesReward;
    for (int i = 0; i < idleVehIDs.size(); i++) {
        delete[] y[i];
    }
    delete[] y;
    delete[] z;
}


void RTVGraph::rebalance(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved) {
    int unservedCnt = unserved.size();
    int isPositive = (int)(now_time >= 0);
    int timeToUse = ((now_time + isPositive * (300 - 1)) / 300) * 300; //forecasts are imported in 300 second intervals; get the next one
    print_line(outDir, logFile, string_format("Pruning fix rebalancing.", now_time, timeToUse));
    map<int, set<int> >().swap(rId_tIdxes);
    map<TIdxComparable, vector<pair<int, pair<int, int>> > >().swap(tIdx_vCostIdxes);
    vector<vector<pair<int, pair<int, int>> > >().swap(vIdx_tIdxes);
    rebalance_for_pruning_fix(env, vehicles, unserved);
    print_line(outDir, logFile, string_format("Finishing cars rebalancing.", now_time, timeToUse));
    rebalance_for_finishing_cars(env, vehicles, unserved);
    print_line(outDir, logFile, string_format("Forecast rebalancing (now=%d, using forecasts from %d.)", now_time, timeToUse));
    rebalance_online_offline(env, vehicles, unserved, timeToUse);
    print_line(outDir, logFile, string_format("Done rebalancing.", now_time, timeToUse));
}

void RTVGraph::prune() {
    print_line(outDir, logFile, string_format("Starting to sort edges = %f.", 1));
    int origSize = 0, origSize2 = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); i++)
    {
        origSize += vIdx_tIdxes[i].size();
    }
    for (int i = 0; i < tIdx_vCostIdxes.size(); i++) {
        origSize2 += tIdx_vCostIdxes[i].size();
    }

    auto thisTime = std::chrono::system_clock::now();
    std::vector<TIdxComparable> indices;
    indices.reserve(tIdx_vCostIdxes.size());
    std::transform(begin(tIdx_vCostIdxes), end(tIdx_vCostIdxes), std::back_inserter(indices), [](auto const& pair) { return pair.first; });
    int i;
#pragma omp parallel for private(i) num_threads(omp_get_max_threads())
    for (i = 0; i < indices.size(); i++) {
        auto iter = tIdx_vCostIdxes.find(indices[i]);
        if (iter != tIdx_vCostIdxes.end()) {
            sort(iter->second.begin(), iter->second.end());
            iter->second.resize(min(static_cast<int>(iter->second.size()), max_v_per_req));
            iter->second.shrink_to_fit();
        }
    }
    std::vector<int> sizes(indices.size(), -1);
    for (int i = 0; i < indices.size(); i++) {
        sizes[i] = trips[indices[i]].size();
    }
    std::vector<TIdxComparable>().swap(indices);
#pragma omp parallel for private(i) num_threads(omp_get_max_threads())
    for (i = 0; i < vIdx_tIdxes.size(); i++) {
        std::sort(vIdx_tIdxes[i].begin(), vIdx_tIdxes[i].end(),
            [&sizes](const pair<int, pair<int, int>>& a, const pair<int, pair<int, int>>& b) { return (double)a.first / (double)sizes[a.second.second] > (double)b.first / (double)sizes[b.second.second]; });
        vIdx_tIdxes[i].resize(min(static_cast<int>(vIdx_tIdxes[i].size()), min_req_per_v));
        vIdx_tIdxes[i].shrink_to_fit();
    }
    map_of_pairs tvCombos;
    tvCombos.reserve(max_v_per_req * tIdx_vCostIdxes.size() + min_req_per_v * vIdx_tIdxes.size());
    for (auto iter = tIdx_vCostIdxes.begin(); iter != tIdx_vCostIdxes.end(); ++iter) {
        //tvCombos.insert(iter->second.begin(), iter->second.end());
        for (int j = 0; j < (*iter).second.size(); j++) {
            tvCombos.emplace(std::pair<int, int>{(*iter).first, (*iter).second[j].second.second}, (*iter).second[j].first); //{trip, vehicle}, cost
        }
    }
    map<TIdxComparable, vector<pair<int, pair<int, int>> > >().swap(tIdx_vCostIdxes);
    for (int i = 0; i < vIdx_tIdxes.size(); i++) {
        for (int j = 0; j < vIdx_tIdxes[i].size(); j++) {
            tvCombos.emplace(std::pair<int, int>{vIdx_tIdxes[i][j].second.second, i}, vIdx_tIdxes[i][j].first); //{trip, vehicle}, cost
        }
    }
    vector<vector<pair<int, pair<int, int>> > >().swap(vIdx_tIdxes);

    int prevSize = tvCombos.size();

    //# riders, cost, r, v, tripIdx
    std::vector<std::tuple<int, int, int, int>> size_cost_t_v(prevSize, std::tuple<int, int, int, int>(-1, -1, -1, -1));
    int thisIdx = 0;
    for (auto it = tvCombos.begin(); it != tvCombos.end(); ++it)
    {
        const std::pair<std::pair<int, int>, int>& thisElement = *it; //{trip, vehicle}, cost
        std::tuple<int, int, int, int>& thisTup = size_cost_t_v[thisIdx++];
        std::get<0>(thisTup) = trips[thisElement.first.first].size(); //size
        std::get<1>(thisTup) = thisElement.second; //cost
        std::get<2>(thisTup) = thisElement.first.first; //t
        std::get<3>(thisTup) = thisElement.first.second; //v
    }
    std::sort(size_cost_t_v.begin(), size_cost_t_v.end(),
        [](const std::tuple<int, int, int, int>& a, const std::tuple<int, int, int, int>& b) {
            return std::get<1>(a) * 1.0 / std::get<0>(a) < std::get<1>(b) * 1.0 / std::get<0>(b);
        }); //orders ascending by req then veh

    std::vector<int> rAssignments(numRequests, -1);
    std::vector<int> vAssignments(numVehicles, -1);
    std::vector<int> rAssignmentCounts(numRequests, 0);
    std::vector<int> vAssignmentCounts(numVehicles, 0);
    std::vector<int> tAssignmentCounts(trips.size(), 0);
    int pre_size = size_cost_t_v.size();
    int assignableCnt = 0;
    uos keptTrips;
    print_line(outDir, logFile, "Starting slow part of pruning.");
    for (int i = 0; i < size_cost_t_v.size(); i++) {
        int vIdx = std::get<3>(size_cost_t_v[i]);
        int tIdx = std::get<2>(size_cost_t_v[i]);
        const uos& thisTrip = trips[tIdx];
        int minAssignmentCount = 0;
        bool vHasEnough = vAssignmentCounts[vIdx] >= min_req_per_v;
        bool vAssigned = (-1 != vAssignments[vIdx]);
        bool bAddThisOne = false;
        for (auto it = thisTrip.begin(); it != thisTrip.end(); ++it) {
            if (vHasEnough && rAssignmentCounts[*it] >= max_v_per_req && (vAssigned || -1 != rAssignments[*it])) continue;
            bAddThisOne = true;
            break;
        }
        if (!bAddThisOne || std::get<0>(size_cost_t_v[i]) == -1) {
            std::get<0>(size_cost_t_v[i]) = -1;
            continue;
        }
        vAssignmentCounts[vIdx]++;
        tAssignmentCounts[tIdx]++;
        keptTrips.insert(tIdx);
        for (auto it = thisTrip.begin(); it != thisTrip.end(); ++it) {
            rAssignmentCounts[*it]++;
        }
        if (-1 == vAssignments[vIdx]) {
            bool bAvailable = true;
            for (auto it = thisTrip.begin(); it != thisTrip.end(); ++it) {
                if (-1 != rAssignments[*it]) {
                    bAvailable = false;
                    break;
                }
            }
            vAssignments[vIdx] = tIdx;
            for (auto it = thisTrip.begin(); it != thisTrip.end(); ++it) {
                rAssignments[*it] = tIdx;
            }
        }
    }
    print_line(outDir, logFile, "Done with slow part of pruning.");
    std::vector<int>().swap(rAssignments);
    std::vector<int>().swap(vAssignments);
    size_cost_t_v.erase(std::remove_if(size_cost_t_v.begin(), size_cost_t_v.end(),
        [](const std::tuple<int, int, int, int>& o) { return std::get<0>(o) == -1; }),
        size_cost_t_v.end());
    size_cost_t_v.shrink_to_fit();

    vIdx_tIdxes.resize(numVehicles);
#pragma omp parallel for private(i) num_threads(omp_get_max_threads())
    for (i = 0; i < numVehicles; i++) {
        vIdx_tIdxes[i].reserve(vAssignmentCounts[i]);
    }
    for (auto it = keptTrips.begin(); it != keptTrips.end(); it++) {
        tIdx_vCostIdxes[*it].reserve(tAssignmentCounts[*it]);
    }

    for (i = 0; i < size_cost_t_v.size(); i++) {
        const std::tuple<int, int, int, int>& thisTup = size_cost_t_v[i];
        int cost = std::get<1>(thisTup);
        int tIdx = std::get<2>(thisTup);
        int vidx = std::get<3>(thisTup);
        if (tIdx == -1) {
            int x = 5;
        }
        tIdx_vCostIdxes[tIdx].push_back(pair<int, pair<int, int>>{cost, pair<int, int>{0, vidx}});
        vIdx_tIdxes[vidx].push_back(pair<int, pair<int, int>>{cost, pair<int, int>{0, tIdx}});
    }
    map<int, set<int> >().swap(rId_tIdxes);
    for (i = 0; i < size_cost_t_v.size(); i++) {
        int tIdx = std::get<2>(size_cost_t_v[i]);
        const uos& thisTrip = trips[tIdx];
        for (auto it = thisTrip.begin(); it != thisTrip.end(); ++it) {
            rId_tIdxes[*it].insert(tIdx);
        }
    }

    numTrips = tIdx_vCostIdxes.size();

    int prunedSize = 0, prunedSize2 = 0;
    for (int i = 0; i < vIdx_tIdxes.size(); i++)
    {
        prunedSize = prunedSize + vIdx_tIdxes[i].size();
    }
    for (auto it = tIdx_vCostIdxes.begin(); it != tIdx_vCostIdxes.end(); ++it) {
        prunedSize2 += it->second.size();
    }
    if (prunedSize2 != size_cost_t_v.size()) {
        int x = 5;
    }
    print_line(outDir, logFile, string_format("RTV size trimmed from %d & %d -> %d -> %d & %d.",
        origSize, origSize2, prevSize, prunedSize, prunedSize2));
}

void RTVGraph::solve(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& requests, vector<Request>& unservedCollector) {

    prune();
    // printf("Defining variables...\n");
    GRBModel model = GRBModel(*env);
    GRBVar** epsilon = new GRBVar * [numTrips];
    int numToDelete = numTrips;
    std::vector<int> validTripIdxes(numTrips, -1);
    std::unordered_map<int, int> validTripReverseLookup;
    int counter = 0;
    for (auto it = tIdx_vCostIdxes.begin(); it != tIdx_vCostIdxes.end(); it++) {
        validTripIdxes[counter] = it->first;
        validTripReverseLookup[it->first] = counter;
        counter++;
    }
    for (int i = 0; i < validTripIdxes.size(); i++) {
        epsilon[i] = model.addVars(tIdx_vCostIdxes[validTripIdxes[i]].size(), GRB_BINARY);
    }

    try {
        // default initial values
        // printf("Initializing...\n");
        std::map<int, std::map<int, int>> tempLookupRtoT;
        for (int i = 0; i < validTripIdxes.size(); i++) {
            int tripIdx = validTripIdxes[i];
            for (int j = 0; j < tIdx_vCostIdxes[tripIdx].size(); j++) {
                epsilon[i][j].set(GRB_DoubleAttr_Start, 0.0);
                tempLookupRtoT[(tIdx_vCostIdxes[tripIdx])[j].second.second][i] = j; //key = vIdx, value = map of (key = tIdx, value = idx into tIdx_vCostIdxes[tIdx] taht corresponds to vIdx)
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
        print_line(outDir, logFile, "1649.");
        /* Constraint #2: Each request is served on exactly one trip*/
        // printf("Adding constraint #2 ...\n");
        for (auto iterRV = rId_tIdxes.begin(); iterRV != rId_tIdxes.end(); ++iterRV) {
            GRBLinExpr constr = 0;
            int rId = iterRV->first;
            auto iterTIdx = iterRV->second.begin();
            while (iterTIdx != iterRV->second.end()) {
                int tIdx = *iterTIdx;
                auto it3 = tIdx_vCostIdxes.find(tIdx);
                if (it3 != tIdx_vCostIdxes.end()) {
                    vector<pair<int, pair<int, int>> >& vCostIdxes = it3->second;
                    for (int j = 0; j < vCostIdxes.size(); j++) {
                        constr += epsilon[validTripReverseLookup[tIdx]][j];
                    }
                }
                iterTIdx++;
            }
            model.addConstr(constr <= 1.0 + minimal);
        }

        print_line(outDir, logFile, "1670.");
        // greedy assignment
        // printf("Greedy assignment...\n");
        set<int> assignedRIds, assignedVIdxes;
        vector<int> tIdxes;
        int tripSize = max_trip_size;
        int maxCostTrip = 0;
        vector<vector<pair<int, pair<int, int>> >::iterator> edgeIters, edgeEnds;
        for (auto iterTV = tIdx_vCostIdxes.begin(); iterTV != tIdx_vCostIdxes.end(); ++iterTV) {
            if (iterTV->second.size() == 0) continue;
            if (tripSize > trips[iterTV->first].size()) {
                greedy_assign_same_trip_size(
                    edgeIters, edgeEnds, tIdxes, assignedRIds, assignedVIdxes,
                    epsilon, tempLookupRtoT, validTripReverseLookup
                );
                tripSize = trips[iterTV->first].size();
                edgeIters.clear();
                edgeEnds.clear();
                tIdxes.clear();
            }
            edgeIters.push_back(iterTV->second.begin());
            edgeEnds.push_back(iterTV->second.end());
            tIdxes.push_back(iterTV->first);
            maxCostTrip = max(maxCostTrip, iterTV->second[iterTV->second.size() - 1].first);
        }
        greedy_assign_same_trip_size(
            edgeIters, edgeEnds, tIdxes, assignedRIds, assignedVIdxes,
            epsilon, tempLookupRtoT, validTripReverseLookup
        );
        int thisPenalty = maxCostTrip * 10; //penalty also defined in globals.h/.cpp

        print_line(outDir, logFile, "1700.");
        // build objective expression
        GRBLinExpr objective = 0;
        // printf("Generating objective expression...\n");
        for (int i = 0; i < validTripIdxes.size(); i++) {
            int tIdx = validTripIdxes[i];
            vector<pair<int, pair<int, int>> >& vCostIdxes = tIdx_vCostIdxes[tIdx];
            int reqsInTrip = trips[tIdx].size();
            for (int vehIdx = 0; vehIdx < vCostIdxes.size(); vehIdx++) {
                objective += epsilon[i][vehIdx] * (vCostIdxes[vehIdx].first - reqsInTrip * thisPenalty);
            }
        }
        model.set("TimeLimit", "1500.0");
        model.set("MIPGap", "0.01");
        model.set("Method", "1");
        model.set("NodeFileStart", "0.5");
        model.set("Presolve", "1");
        //model.set("VarBranch","3");
        //model.set("MIPFocus","3");
        std::string nt = std::to_string(now_time);
        std::string grbLogName = outDir + "GurobiLogs/" + "rtv_solve_" + nt;
        model.set("LogFile", grbLogName + ".txt");
        model.setObjective(objective, GRB_MINIMIZE);

        // printf("Solving..
        // solve.\n");
        model.optimize();
        // printf("]\n");
    //printf("Number of unserved requests: %d\n", cnt);
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
                    int unadjustedTidx = validTripIdxes[tIdx];
                    for (auto iter2 = trips[unadjustedTidx].begin(); iter2 != trips[unadjustedTidx].end(); ++iter2) {
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

    }
    catch (GRBException e) {
        print_line(outDir, logFile, string_format("Gurobi exception code: %d.", e.getErrorCode()));
        print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
    }
    for (int i = 0; i < numToDelete; i++) {
        delete[] epsilon[i];
    }
    delete[] epsilon;
}

