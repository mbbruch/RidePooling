#pragma once
#include <queue>
#include <deque>
#include <vector>
#include <map>
#include <unordered_map>
#include "util.h"
#include "globals.h"
#include <set>
#include "Request.h"

using namespace std; 

template<typename T, typename Container = std::deque<T> >
class iterable_queue : public std::queue<T, Container>
{
public:
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    iterator begin() { return this->c.begin(); }
    iterator end() { return this->c.end(); }
    const_iterator begin() const { return this->c.begin(); }
    const_iterator end() const { return this->c.end(); }
};

class Vehicle {
    int location, timeToNextNode;
    bool available;
    int availableSince;

public:
    vector<Request> passengers;
    iterable_queue<pair<int, int> > scheduledPath;
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
    void check_passengers(int nowTime, int stop, bool& exceeded, int& sumDelays, int& newPickups,
        vector<int>& getOffPsngr, vector<Request>& schedule, map<int, int>& occupancyChanges, bool decided);
    void updateOccupancyTracker(map<int, int>& occupancyChanges, int time, int change);
    void setup_occupancy_changes(map<int, int>& changes);
    void reverse_passengers(vector<int>& getOffPsngr,
        vector<Request>& schedule, bool decided);
    void set_passengers(vector<Request>& psngrs);
    void head_for(int node, int departureTimeFromNode);
    void update(int nowTime, vector<Request>& newRequests, int idx);
    void set_path(vector<pair<int, int> >& path);
    void finish_route(int idx);
};
