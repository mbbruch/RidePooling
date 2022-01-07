#include <cmath>
#include <vector>
#include <queue>
#include <set>
#include <fstream>
#include <unordered_map>
#include <omp.h>
#include "util.h"
#include "Request.h"
#include "Vehicle.h"
#include "globals.h"
#include "GPtree.h"
using namespace std;

Vehicle::Vehicle() {
    timeToNextNode = 0;
    available = false;
    availableSince = -9999;
    wasIdle = true;
    offline = true;
}

Vehicle::Vehicle(int location) {
    this->location = location;
    timeToNextNode = 0;
    availableSince = -9999;
    available = false;
    wasIdle = true;
    offline = true;
}

bool Vehicle::getWasIdle() {
    return this->wasIdle;
}
bool Vehicle::isAvailable() {
    return this->available;
}


int Vehicle::getAvailableSince() const {
    return this->availableSince;
}

void Vehicle::setAvailableSince(int time) {
    this->availableSince = time;
}

int Vehicle::get_location() const {
    return this->location;
}

void Vehicle::set_location(int location) {
    this->location = location;
}

int Vehicle::get_num_passengers() const {
    return passengers.size();
}

/// <summary>
/// Update the input set with a list of dropoff points for all passengers already assigned the car (or only those already in the car?? TODO check)
/// </summary>
/// <param name="target">List of targets to update</param>
void Vehicle::insert_targets(targetSet& target, map<locReq, set<locReq> >& src_dst, int currentTime) {
    for (int i = 0; i < passengers.size(); i++) {
        if (passengers[i].scheduledOnTime <= currentTime) {
            target.insert(make_pair(passengers[i].end, passengers[i].unique));
        }
        else {
            target.insert(make_pair(passengers[i].start, passengers[i].unique));
            src_dst[make_pair(passengers[i].start,passengers[i].unique)].insert(make_pair(passengers[i].end, passengers[i].unique));
        }
    }
}

int Vehicle::getOccupancyAt(int currentTime) {
    int toReturn = 0;
    for (int i = 0; i < passengers.size(); i++) {
        if (passengers[i].scheduledOnTime < currentTime) {
            toReturn++;
        }
        if (passengers[i].scheduledOffTime < currentTime) {
            toReturn--;
        }
    }
    return toReturn;
}

void Vehicle::fixPassengerStatus(int nowTime) {
    for (int i = 0; i < passengers.size(); i++) {
        Request& thisReq = passengers[i];
        if (thisReq.scheduledOnTime >= nowTime) {
            thisReq.status = Request::requestStatus::waiting;
        }
        else {
            if (thisReq.scheduledOffTime < nowTime) {
                thisReq.status = Request::requestStatus::droppedOff;
            }
            else {
                thisReq.status = Request::requestStatus::onBoard;
            }
        }
    }
}

/// <summary>
/// 
/// </summary>
/// <param name="nowTime">current time</param>
/// <param name="stop">node number of node we're currently checking in DFS</param>
/// <param name="exceeded">Set this to TRUE if dropoff times of any onboard passengers would be too late</param>
/// <param name="sumCost">Update with sum of delay time of each passenger who is getting off (only used if exceeded is FALSE)</param>
/// <param name="getOffPsngr"></param>
/// <param name="schedule"></param>
/// <param name="decided"></param>
void Vehicle::check_passengers(int nowTime, locReq stop, bool& exceeded, int& sumDelays, int& newOffset, int& newPickups, 
    vector<int>& getOffPsngr, vector<int>& getOnsPsngr,
    vector<Request>& schedule, int& occupancy, bool decided) {

    for (int i = 0; i < passengers.size(); i++) {
        Request& req = passengers[i];
        if (req.status == Request::requestStatus::onBoard) {
            if (nowTime - req.expectedOffTime > req.allowedDelay) {
                exceeded = true;
                return;
            }
            if (req.end == stop.first && req.unique == stop.second) {
                occupancy--;
                req.status = Request::requestStatus::droppedOff;
                if (decided) {
                    req.scheduledOffTime = nowTime;
                    schedule.push_back(req);
                }
                int addlDelay = nowTime - req.expectedOffTime;
                if (addlDelay > 0) {
                    sumDelays += addlDelay;
                }
                getOffPsngr.push_back(i);
            }
        }
        else if (req.status == Request::requestStatus::waiting) {
            if (nowTime - req.reqTime > req.allowedWait) {
                exceeded = true;
                return;
            }
            if (req.start == stop.first && req.unique == stop.second) {
                if (++occupancy > max_capacity) {
                    exceeded = true;
                    return;
                }
                req.status = Request::requestStatus::onBoard;
                int earliestArrivalAllowed = max(nowTime, req.reqTime);
                int timeDiff = earliestArrivalAllowed - nowTime;
                if (timeDiff > 0) {
                    if (nowTime < 0) {
                        newOffset = timeDiff;
                    }
                    nowTime = earliestArrivalAllowed;
                }
                if (decided) {
                    req.scheduledOnTime = nowTime;
                }
                getOnsPsngr.push_back(i);
            }
        }
    }
}

void Vehicle::reverse_passengers(const vector<int>& getOffPsngr, const vector<int>& getOnPsngr, int& occupancy, vector<Request>& schedule, int newOffset, bool decided) {

    size_t offCnt = getOffPsngr.size();
    for (auto it = getOffPsngr.begin(); it != getOffPsngr.end(); ++it) {
        passengers[*it].status = Request::requestStatus::onBoard;
        occupancy++;
    }
    if (decided) {
        while (offCnt > 0) {
            schedule.pop_back();
            offCnt--;
        }
    }
    size_t onCnt = getOnPsngr.size();
    for (auto it = getOnPsngr.begin(); it != getOnPsngr.end(); ++it) {
        passengers[*it].status = Request::requestStatus::waiting;
        occupancy--;
    }
}

void Vehicle::set_passengers(vector<Request>& psngrs) {
    this->passengers = psngrs;
}

void Vehicle::clear_path() {
    while (!this->scheduledPath.empty()) {
        this->scheduledPath.pop();
    }
}
void Vehicle::head_for(int node, int departureTimeFromNode) {
    clear_path();
    if (this->location == node) {
        this->scheduledPath.push(make_pair(departureTimeFromNode, node));
        return;
    }

    bool empty = true;
    vector<int> order;
    #pragma omp critical (findpath)
    treeCost.find_path(this->location - 1, node - 1, order);
    order[0] += 1;
    int distTravelled = 0;
    int tmpTime = 0;
    int baseTime = now_time + this->timeToNextNode;
    for (int i = 1; i < order.size(); i++) { // head is location itself
        order[i] += 1;
        distTravelled += treeCost.get_dist(order[i-1], order[i]).second;
        tmpTime = baseTime + ceil(static_cast<double>(distTravelled) / velocity * 1.0);
        this->scheduledPath.push(make_pair(tmpTime, order[i]));
        empty = false;
    }
    if (tmpTime < departureTimeFromNode) {
        this->scheduledPath.push(make_pair(departureTimeFromNode, node));
    }
}

void Vehicle::update(int nowTime, vector<Request>& newRequests, int idx) {
    if (availableSince == -9999 && this->passengers.size() == 0 && this->scheduledPath.size() == 0) {
        refresh_status(nowTime);
        return;
    }
    int counter = 0;
    for (int i = 0; i < this->passengers.size(); i++) {
        if (this->passengers[i].unique == 5 && now_time == 3600) {
            counter++;
        }
    }
    if (counter == 1) {
        int x = 5;
    }
    std::ofstream routes, pickups, requestDistances, thisveh;
    if (!this->scheduledPath.empty()) {
        routes.open(outDir + "Routes/" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
        pickups.open(outDir + "Pickups/" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
        requestDistances.open(outDir + "Misc/" + "requestDistances.csv", std::ofstream::out | std::ofstream::app);
        thisveh.open(outDir + "Routes/full_schedule" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
        for (auto iter = this->scheduledPath.begin(); iter != this->scheduledPath.end(); iter++) {
            thisveh << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "," + "\n";
        }
        thisveh.close();
    }

    //MOVE car to first stop on or after nowTime.
    //LOG only the stops on or before nowTime.
    const int startOfThisUpdate = this->availableSince;
    bool bSkipsThisTimestep = startOfThisUpdate > nowTime;
    bool bLogStartingPoint = ((nowTime - time_step) < startOfThisUpdate) && (startOfThisUpdate <= nowTime) && startOfThisUpdate!= this->scheduledPath.front().first;
    iterable_queue<pair<int, int> > copy = this->scheduledPath;
    int time1 = -INF, time2 = -INF, node1 = -INF, node2 = -INF, id1= -INF, id2= -INF;
    while (!this->scheduledPath.empty() || bLogStartingPoint) {
        int schedTime = bLogStartingPoint ? startOfThisUpdate : this->scheduledPath.front().first;
        int node = bLogStartingPoint ? this->location : this->scheduledPath.front().second;
        this->location = node;
        this->availableSince = schedTime;
        if (schedTime <= nowTime) {
            if (!bLogStartingPoint) this->scheduledPath.pop();
            #pragma omp critical (writeRoutes)
            routes << to_string(nowTime) + "," + to_string(schedTime) + "," + to_string(node) + "\n";
            if (!this->passengers.empty()) {
                for (int i = 0; i < this->passengers.size(); i++) {
                    if (this->passengers[i].scheduledOnTime == schedTime && this->passengers[i].start == node) {
                        id2 = id1;
                        time2 = time1;
                        node2 = node1;
                        id1 = this->passengers[i].unique;
                        time1 = schedTime;
                        node1 = node;
                        if (time1 == time2 && node1 == node2 && id1 == id2) {
                            int x = 5;
                        }
                        pickups << to_string(nowTime) + "," + to_string(this->passengers[i].unique) + ",on," + to_string(node) + "," + to_string(schedTime) + "," + to_string(this->passengers[i].reqTime) + "\n";
                        requestDistances << to_string(this->passengers[i].reqTime) + "," + to_string(this->passengers[i].unique) + "," + to_string(this->passengers[i].start) + "," + to_string(this->passengers[i].end) + "," + to_string(this->passengers[i].shortestDist) + "\n";
                    }
                    if (this->passengers[i].scheduledOffTime == schedTime && this->passengers[i].end == node) {
                        pickups << to_string(nowTime) + "," + to_string(this->passengers[i].unique) + ",off," + to_string(node) + "," + to_string(schedTime) + "," + to_string(this->passengers[i].expectedOffTime) + "\n";
                    }
                }
            }
        }
        bLogStartingPoint = false;
        if (bSkipsThisTimestep || schedTime >= nowTime) {
            break;
        }
    }
    routes.close();
    pickups.close();
    requestDistances.close();

    vector<Request> newPassengers;
    for (auto iterPsngr = this->passengers.begin(); iterPsngr != this->passengers.end(); iterPsngr++) {
        bool bNotYetOnboard = iterPsngr->scheduledOnTime > nowTime;
        bool bOffloaded = iterPsngr->scheduledOffTime <= nowTime;
        if (bNotYetOnboard) { // hasn't gotten on board
            iterPsngr->status = Request::requestStatus::waiting;
            for (int i = 0; i < newRequests.size(); i++) {
                if (newRequests[i].unique == iterPsngr->unique) {
                    int x = 5;
                }
            }
            #pragma omp critical(pushbackreq)
            newRequests.push_back(*iterPsngr);
        }
        else {
            if (bOffloaded) { // already got off
                iterPsngr->status = Request::requestStatus::droppedOff;
            }
            else { // now on board
                iterPsngr->status = Request::requestStatus::onBoard;
                newPassengers.push_back(*iterPsngr);
            }
        }
    }
    this->passengers = newPassengers;
    refresh_status(nowTime);
    //map: time, node, req index, on or off
    std::map<int, std::vector<locReq>> onOffs;
    for (int i = 0; i < this->passengers.size(); i++) {
        const Request& thisReq = this->passengers[i];
        onOffs[thisReq.scheduledOffTime].push_back(make_pair(thisReq.end, i));
    }
}

void Vehicle::refresh_status(int time) {
    this->wasIdle = this->availableSince < time;
    this->availableSince = (this->offline || (this->availableSince < 0 && this->passengers.size() ==0)) ? this->availableSince : max(this->availableSince, time);
    this->timeToNextNode = this->availableSince - time;
    this->available = this->offline ? false : this->availableSince < (time + time_step);
}

void Vehicle::set_path(const vector<pair<int, int>>& path) {
    while (!this->scheduledPath.empty()) this->scheduledPath.pop();
    auto it = path.begin();
    this->scheduledPath.push(*it);
    it++;
    while (it != path.end()) {
        if (this->scheduledPath.back() != *it && !(this->scheduledPath.back().first > it->first)) this->scheduledPath.push(*it);
        it++;
    }
}

void Vehicle::finish_route(int idx, int nowTime) {
    if (!this->passengers.empty()) {
        std::ofstream routes, pickups, requestDistances, thisveh;
        routes.open(outDir + "Routes/" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
        pickups.open(outDir + "Pickups/" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
        requestDistances.open(outDir + "Misc/" + "requestDistances.csv", std::ofstream::out | std::ofstream::app);
        thisveh.open(outDir + "Routes/full_schedule" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
        for (auto iter = this->scheduledPath.begin(); iter != this->scheduledPath.end(); iter++) {
            thisveh << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "," + "\n";
        }
        thisveh.close();
        for (auto iterPsngr = this->passengers.begin(); iterPsngr != this->passengers.end(); iterPsngr++) {
            these_served_reqs++;
            served_reqs++;
            // printf("%d on, ", iterPsngr->unique);
            total_wait_time += iterPsngr->scheduledOnTime - iterPsngr->reqTime;
        }

        while (!this->scheduledPath.empty()) {
            int schedTime = this->scheduledPath.front().first;
            int node = this->scheduledPath.front().second;
            routes << to_string(nowTime) + "," + to_string(schedTime) + "," + to_string(node) + "\n";
            if (!this->passengers.empty()) {
                for (int i = 0; i < this->passengers.size(); i++) {
                    if (this->passengers[i].scheduledOnTime == schedTime && this->passengers[i].start == node) {
                        pickups << to_string(nowTime) + "," + to_string(this->passengers[i].unique) + ",on," + to_string(node) + "," + to_string(schedTime) + "," + to_string(this->passengers[i].reqTime) + "\n";
                        requestDistances << to_string(this->passengers[i].reqTime) + "," + to_string(this->passengers[i].unique) + "," + to_string(this->passengers[i].start) + "," + to_string(this->passengers[i].end) + "," + to_string(this->passengers[i].shortestDist) + "\n";
                    }
                    if (this->passengers[i].scheduledOffTime == schedTime && this->passengers[i].end == node) {
                        pickups << to_string(nowTime) + "," + to_string(this->passengers[i].unique) + ",off," + to_string(node) + "," + to_string(schedTime) + "," + to_string(this->passengers[i].expectedOffTime) + "\n";
                    }
                }
            }
            this->location = node;
            this->scheduledPath.pop();
        }
        routes.close();
        pickups.close();
        requestDistances.close();
    }
}

