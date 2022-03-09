#pragma once

#include <vector>
#include <map>
#include <set>
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
        tripCandidate(const uos& reqs) {
            requests = reqs;
        };

        tripCandidate(const tripCandidate& toCopy) {
            requests = toCopy.requests;
            dependentTrips = toCopy.dependentTrips;
        };
    };

	typedef std::vector<tripCandidate> tripsVec;

    inline static std::vector<tripsVec> allPotentialTrips;


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

    typedef int TIdxComparable;

    //---- these are edges of RTV graph ----
    map<int, set<int> > rId_tIdxes;
	// trip -> edges to vehicles (<cost, (random,vIdx)> pairs)
    map<TIdxComparable, vector<pair<int, pair<int,int>> > > tIdx_vCostIdxes;
    vector<vector<pair<int, pair<int,int>> > > vIdx_tIdxes; //cost, (random, tIdx)
    //--------------------------------------

    int addVehicleId(int vehicleId);

    int getTIdx(const uos& trip);

    void add_edge_trip_vehicle(const uos& reqsInTrip, int vIdx, int cost);
    void add_edge_trip_vehicle(int tIdx, int vIdx, int cost);
    void build_potential_trips(RVGraph* rvGraph, vector<Request>& requests, vector<Vehicle>& vehicles);
    void build_single_vehicles(vector<Request>& requests, vector<Vehicle>& vehicles, 
        const map<int, int>& vehIDToVehIdx, int tripSize,
        const unordered_map<int, int>& adjustedTripIdxes, std::vector<int>& addedTrips,
        const std::vector<std::pair<int, uos>>& prevInclusions, std::vector<std::pair<int, uos>>& theseInclusions);

    void serialize_current_combos();
    void deserialize_current_combos();
    void deserialize_valid_trips();

    void greedy_assign_same_trip_size(
	    vector<vector<pair<int, pair<int,int>> >::iterator>& edgeIters,
        vector<vector<pair<int, pair<int,int>> >::iterator>& edgeEnds,
        vector<int>& tIdxes,
        set<int>& assignedRIds, set<int>& assignedVIdxes,
        GRBVar** epsilon,
        std::map<int, map<int, int>>& lookupRtoT,
		unordered_map<int,int>& validTripReverseLookup
    );

public:
    RTVGraph(RVGraph* rvGraph, vector<Vehicle>& vehicles, vector<Request>& requests);

    void rebalance(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved);
    void static update_unserved_from_rebalancing(vector<Request>& unserved, uos& newlyServed);
    void static rebalance_for_pruning_fix(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved);
    void static rebalance_for_finishing_cars(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved);
    void static rebalance_online_offline(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved, int now_time);
    void static rebalance_for_demand_forecasts(GRBEnv* env, vector<Vehicle>& vehicles, vector<Request>& unserved, int nowTime, bool bEquityVersion = false);
	void prune();
    void solve(GRBEnv* env,
        vector<Vehicle>& vehicles, vector<Request>& requests,
        vector<Request>& unservedCollector);

    friend bool equal_to_sub(vector<int>& compared, vector<int>& origin, int excludeIdx);

    inline size_t key(int i, int j) { return (size_t)i << 32 | (unsigned int)j; }
};

