#pragma once

#include <vector>
#include <map>
#include <set>
#include "util.h"
#include <unordered_set>
#include <random>
#include "RV.h"
#include "util.h"

using namespace std;

class RTVGraph {

    int numRequests, numTrips, numVehicles;
    class tripCandidate {
    public:
        uos requests;
        vector<int> dependentTrips;
        bool ruledOut;

        tripCandidate(const uos& reqs) {
            requests = reqs;
            ruledOut = false;
        };
        tripCandidate(const tripCandidate& toCopy) {
            requests = toCopy.requests;
            dependentTrips = toCopy.dependentTrips;
            ruledOut = toCopy.ruledOut;
        };
    };

    vector<vector<tripCandidate>> allPotentialTrips;


    /* WHAT WE NEED:
    *  trips arranged by size, so we can instantly get the 1-smaller trips 
    *       and add current size
    * exclusion list: for each trip, a list of all trips it includes.
    */

    //---- these are nodes of RTV graph ----
    vector<uos> trips; // trips[tIdx] = trip
    vector<int> vIds; // vIds[vIdx] = vehicleId
    //--------------------------------------
	
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen1; ; //Standard mersenne_twister_engine seeded with rd()
	std::mt19937 gen2; //Standard mersenne_twister_engine seeded with rd()
	std::uniform_int_distribution<> distribOfCars;
	std::uniform_int_distribution<> distribOfTrips;

    map<uos, int> trip_tIdx; // trip -> tIdx, reverse of vector "trips"

    class TIdxComparable {
    public:
        static RTVGraph* rtvGraph;
        int tIdx;

        TIdxComparable(int tIdx) : tIdx(tIdx) {}
        bool operator <(const TIdxComparable& other) const;
    };

    //---- these are edges of RTV graph ----
    map<int, set<int> > rId_tIdxes;
	// trip -> edges to vehicles (<cost, (random,vIdx)> pairs)
    map<TIdxComparable, vector<pair<int, pair<int,int>> > > tIdx_vCostIdxes;
    vector<vector<pair<int, pair<int,int>> > > vIdx_tIdxes; //cost, (random, tIdx)
    //--------------------------------------

    int addVehicleId(int vehicleId);

    int getTIdx(const uos& trip);

    void add_edge_trip_vehicle(uos& reqsInTrip, int vIdx, int cost);
    void build_potential_trips(RVGraph* rvGraph, vector<Request>& requests, map_of_pairs& dist);

    void build_single_vehicle(int vehicleId, int vIdx, vector<Vehicle>& vehicles, 
        RVGraph* rvGraph, vector<Request>& requests,
        map_of_pairs& dist, vector<vector<tripCandidate>>& potentialTrips);

    void sort_edges();

    void greedy_assign_same_trip_size(
	    vector<vector<pair<int, pair<int,int>> >::iterator>& edgeIters,
        vector<vector<pair<int, pair<int,int>> >::iterator>& edgeEnds,
        vector<int>& tIdxes,
        set<int>& assignedRIds, set<int>& assignedVIdxes,
        GRBVar** epsilon,
        std::map<int, map<int, int>>& lookupRtoT
    );

public:
    RTVGraph(RVGraph* rvGraph, vector<Vehicle>& vehicles,
        vector<Request>& requests, map_of_pairs& dist);

    void rebalance(GRBEnv* env,
        vector<Vehicle>& vehicles, vector<Request>& unserved,
        map_of_pairs& dist);

	void prune();
    void solve(GRBEnv* env,
        vector<Vehicle>& vehicles, vector<Request>& requests,
        vector<Request>& unservedCollector,
        map_of_pairs& dist);

    friend bool equal_to_sub(vector<int>& compared, vector<int>& origin, int excludeIdx);

    inline size_t key(int i, int j) { return (size_t)i << 32 | (unsigned int)j; }
};

