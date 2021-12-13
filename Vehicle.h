#pragma once
#include <vector>
#include <map>
#include <unordered_map>
#include "util.h"
#include "globals.h"
#include <set>
#include "Request.h"

using namespace std; 

class Vehicle {
    int location;
    bool available;
    int availableSince;

private:
    int timeToNextNode;
    bool wasIdle;

public:
    vector<Request> passengers;
    iterable_queue<pair<int, int> > scheduledPath;
    Vehicle();
    Vehicle(int location);
    bool getWasIdle();
    bool isAvailable();
    int getAvailableSince();
    void setAvailableSince(int time);
    int get_location();
    void set_location(int location);
    int get_num_passengers();
    void fixPassengerStatus(int nowTime);
    void insert_targets(targetSet& target, map<locReq, set<locReq> >& src_dst, int currentTime);
    void check_passengers(int nowTime, locReq stop, bool& exceeded, int& sumDelays, int& newOffset, int& newPickups,
        vector<pair<int, int>>& getOffPsngr, vector<pair<int, int>>& getOnsPsngr,
        vector<Request>& schedule, map<int, int>& occupancyChanges, bool decided);
    void updateOccupancyTracker(map<int, int>& occupancyChanges, int time, int offset, int change);
    void setup_occupancy_changes(map<int, int>& changes, int currentTime);
    int checkMaxOccupancy(const vector<Request>& psgrs);
    void reverse_passengers(vector<pair<int, int>>& getOffPsngr, vector<pair<int, int>>& getOnsPsngr, map<int, int>& occupancyChanges,
        vector<Request>& schedule, int newOffset, bool decided);
    void set_passengers(vector<Request>& psngrs);
    void head_for(int node, int departureTimeFromNode);
    void update(int nowTime, vector<Request>& newRequests, int idx);
    void set_path(const vector<pair<int, int> >& path);
    void finish_route(int idx, int nowTime);
};
