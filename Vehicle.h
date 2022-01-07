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
    bool offline;
    vector<Request> passengers;
    iterable_queue<pair<int, int> > scheduledPath;
    Vehicle();
    Vehicle(int location);
    bool getWasIdle();
    bool isAvailable();
    int getAvailableSince() const;
    void setAvailableSince(int time);
    int get_location() const;
    void set_location(int location);
    int get_num_passengers() const;
    void fixPassengerStatus(int nowTime);
    void insert_targets(targetSet& target, map<locReq, set<locReq> >& src_dst, int currentTime);
    void check_passengers(int nowTime, locReq stop, bool& exceeded, int& sumDelays, int& newOffset, int& newPickups,
        vector<int>& getOffPsngr, vector<int>& getOnsPsngr,
        vector<Request>& schedule, int& occupancy, bool decided);
    int getOccupancyAt(int currentTime);
    int checkMaxOccupancy(const vector<Request>& psgrs);
    void reverse_passengers(const vector<int>& getOffPsngr, const vector<int>& getOnsPsngr, int& occupancy,
        vector<Request>& schedule, int newOffset, bool decided);
    void set_passengers(vector<Request>& psngrs);
    void clear_path();
    void refresh_status(int time);
    void head_for(int node, int departureTimeFromNode);
    void update(int nowTime, vector<Request>& newRequests, int idx);
    void set_path(const vector<pair<int, int> >& path);
    void finish_route(int idx, int nowTime);
};
