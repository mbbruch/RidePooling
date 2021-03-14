#pragma once
#include <queue>
#include <vector>
#include <map>
#include <unordered_map>
#include "util.h"
#include "globals.h"
#include <set>
#include "Request.h"

using namespace std; 

class Vehicle {
    int location, timeToNextNode;
    bool available;
    int availableSince;
    queue<pair<int, int> > scheduledPath;

public:
    vector<Request> passengers;
    Vehicle();
    Vehicle(int location);
    bool isAvailable();
    int getAvailableSince();
    int get_location();
    void set_location(int location);
    int get_time_to_next_node();
    int get_num_passengers();
    void print_passengers();
    void insert_targets(set<int>& target);
    void check_passengers(int nowTime, int stop, bool& exceeded, int& sumCost,
        vector<int>& getOffPsngr, vector<Request>& schedule, map<int, int>& occupancyChanges, bool decided);
    void updateOccupancyTracker(map<int, int>& occupancyChanges, int time, int change);
    void setup_occupancy_changes(map<int, int>& changes);
    void reverse_passengers(vector<int>& getOffPsngr,
        vector<Request>& schedule, bool decided);
    void set_passengers(vector<Request>& psngrs);
    void head_for(int node, map_of_pairs& dist);
    void update(int nowTime, vector<Request>& newRequests, map_of_pairs& dist);
    void set_path(vector<pair<int, int> >& path);
    void finish_route(map_of_pairs& dist);
};
