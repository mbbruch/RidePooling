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
    available = true;
    availableSince = -9999;
}

Vehicle::Vehicle(int location) {
    this->location = location;
    timeToNextNode = 0;
    availableSince = -9999;
    available = true;
}

bool Vehicle::isAvailable() {
    return this->available;
}


int Vehicle::getAvailableSince() {
    return this->availableSince;
}

int Vehicle::get_location() {
    return this->location;
}

void Vehicle::set_location(int location) {
    this->location = location;
}

int Vehicle::get_time_to_next_node() {
    return this->timeToNextNode;
}

int Vehicle::get_num_passengers() {
    return passengers.size();
}

void Vehicle::print_passengers() {
    for (int i = 0; i < passengers.size(); i++) {
        printf("%d: ", passengers[i].unique);
        if (passengers[i].status == Request::onBoard) {
            printf("onboard, ");
        }
        else if (passengers[i].status == Request::waiting) {
            printf("waiting, ");
        }
        else if (passengers[i].status == Request::droppedOff) {
            printf("dropped off, ");
        }
    }
    printf("\n");
}

/// <summary>
/// Update the input set with a list of dropoff points for all passengers already assigned the car (or only those already in the car?? TODO check)
/// </summary>
/// <param name="target">List of targets to update</param>
void Vehicle::insert_targets(set<int>& target) {
    for (int i = 0; i < passengers.size(); i++) {
        target.insert(passengers[i].end);
    }
}

void Vehicle::setup_occupancy_changes(map<int, int>& changes) {
    for (int i = 0; i < passengers.size(); i++) {
        updateOccupancyTracker(changes, passengers[i].scheduledOnTime, 1);
        updateOccupancyTracker(changes, passengers[i].scheduledOffTime, -1);
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
void Vehicle::check_passengers(int nowTime, int stop, bool& exceeded, int& sumDelays, int& newPickups, vector<int>& getOffPsngr, vector<Request>& schedule,
    map<int, int>& occupancyChanges, bool decided) {
    newPickups = 0;
    auto it = occupancyChanges.begin();
    int occupancy = 0;
    auto itEnd = occupancyChanges.end();
    while (it != itEnd) {
        occupancy += it->second;
        if (it->second > 0 && occupancy>1) {
            newPickups += occupancy;
        }
        if (occupancy > max_capacity) {
            exceeded = true;
            return;
        }
        it++;
    }

    for (int i = 0; i < passengers.size(); i++) {
        Request& req = passengers[i];
        if (req.status == Request::onBoard) {
            if (nowTime - req.expectedOffTime > max_delay_sec) {
                exceeded = true;
                return;
            }
            if (req.end == stop) {
                req.status = Request::droppedOff;
                if (decided) {
                    req.scheduledOffTime = nowTime;
                    schedule.push_back(req);
                }
                sumDelays += nowTime - req.expectedOffTime;
                getOffPsngr.push_back(i);
            }
        }
    }
}

void Vehicle::updateOccupancyTracker(map<int, int>& occupancyChanges, int time, int change)
{
    int val = occupancyChanges[time];
  
    if (val == -change) {
        occupancyChanges.erase(time);
    }
    else {
        occupancyChanges[time] = val + change;
    }
	/*
	auto it = occupancyChanges.find(time);
	if(((*it).second+= change) == 0) {
		occupancyChanges.erase(it);
	}
    if ((occupancyChanges[time] += change) == 0) {
        occupancyChanges.erase(time);
    }
	*/
}

void Vehicle::reverse_passengers(vector<int>& getOffPsngr, vector<Request>& schedule, bool decided) {

    size_t offCnt = getOffPsngr.size();
    for (auto it = getOffPsngr.begin(); it != getOffPsngr.end(); it++) {
        passengers[*it].status = Request::onBoard;
    }
    if (decided) {
        while (offCnt > 0) {
            schedule.pop_back();
            offCnt--;
        }
    }
}

void Vehicle::set_passengers(vector<Request>& psngrs) {
    this->passengers = psngrs;
}

void Vehicle::head_for(int node, int departureTimeFromNode) {
    vector<int> order;
    #pragma omp critical (findpath)
    treeCost.find_path(this->location - 1, node - 1, order);
    while (!this->scheduledPath.empty()) {
        this->scheduledPath.pop();
    }
    int distTravelled = 0;
    int tmpTime = 0;
    order[0] += 1;
    int baseTime = (this->timeToNextNode < time_step) ?
        now_time + this->timeToNextNode : now_time - this->timeToNextNode;
    if (this->location == node) {
        this->scheduledPath.push(make_pair(departureTimeFromNode, node));
    }
    for (int i = 1; i < order.size(); i++) { // head is location itself
        order[i] += 1;
        distTravelled += treeCost.get_dist(order[i-1], order[i]).second;
        tmpTime = baseTime + ceil((double(distTravelled)) / velocity);
        this->scheduledPath.push(make_pair(tmpTime, order[i]));
    }
    if (tmpTime < departureTimeFromNode) {
        this->scheduledPath.push(make_pair(departureTimeFromNode, node));
    }
}

void Vehicle::update(int nowTime, vector<Request>& newRequests, int idx) {
    if (this->scheduledPath.empty()) {
        return;
    }
    std::ofstream routes;
    routes.open(outDir + "Routes/" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
    std::ofstream pickups;
    pickups.open(outDir + "Pickups/" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
    std::ofstream requestDistances;
    requestDistances.open(outDir + "Misc/" + "requestDistances.csv", std::ofstream::out | std::ofstream::app);

    Vehicle copy = Vehicle(*this);
    bool b29IsPassenger = false;
    bool b29DroppedOff = false;

    std::ofstream thisveh;
    thisveh.open(outDir + "Routes/full_schedule" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
    for (auto iter = this->scheduledPath.begin(); iter != this->scheduledPath.end(); iter++) {
        thisveh << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "," + "\n";
    }
    thisveh.close();
    for (int i = 0; i < this->passengers.size(); i++) {
        int origin = this->passengers[i].start;
        int originTime = this->passengers[i].scheduledOnTime;
        int schedStartTime = this->scheduledPath.front().first;
        int destination = this->passengers[i].end;
        bool bDIncluded = false;
        bool bOIncluded = false;
        for (auto iter = this->scheduledPath.begin(); iter != this->scheduledPath.end(); iter++) {
            if (iter->second == destination) {
                bDIncluded =true;
            }
            if (iter->second == origin) {
                bOIncluded = true;
            }
        }
        if (!bDIncluded || (originTime >= schedStartTime && !bOIncluded)) {
            int x = 0;
            if (this->location == origin) {
                x = 1;
            }
        }
    }
    


    if (this->timeToNextNode < time_step) {
        while (!this->scheduledPath.empty()) {
            // TODO all these (mbruchon: no idea what this todo means)
            int schedTime = this->scheduledPath.front().first;
            int node = this->scheduledPath.front().second;

            if (schedTime < nowTime || schedTime - nowTime < time_step) {
                if (!this->passengers.empty()) {
                    int onboardCnt = 0;
                    for (int i = 0; i < this->passengers.size(); i++) {
                        if (this->passengers[i].scheduledOnTime == schedTime) {
                            onboardCnt++;
                            pickups << to_string(now_time) + "," + to_string(this->passengers[i].unique) + ",on," + to_string(node) + "," +
                                to_string(schedTime) + "," + to_string(this->passengers[i].reqTime) + "\n";
                            requestDistances << to_string(this->passengers[i].reqTime) + "," + to_string(this->passengers[i].unique) + "," +
                                to_string(this->passengers[i].start) + "," + to_string(this->passengers[i].end) + "," + to_string(this->passengers[i].shortestDist) + "\n";
                        }
                        if (this->passengers[i].scheduledOffTime == schedTime) {
                            onboardCnt--;
                            if (this->passengers[i].unique == 29) {
                                b29DroppedOff = true;
                            }
                            pickups << to_string(now_time) + "," + to_string(this->passengers[i].unique) + ",off," + to_string(node) + "," +
                                to_string(schedTime) + "," + to_string(this->passengers[i].expectedOffTime) + "\n";
                        }
                    }
                    if (onboardCnt < max_capacity) {
                        this->available = true;
                        this->availableSince = schedTime;
                    }
                    else {
                        this->available = false;
                        this->availableSince = this->timeToNextNode + nowTime;
                    }
                }
                this->location = node;
                this->scheduledPath.pop();
                routes <<  to_string(now_time) + "," + to_string(schedTime) + "," + to_string(node) +  "\n";
            }
            if (schedTime >= nowTime) {
                this->timeToNextNode = schedTime - nowTime;
                this->available = (this->timeToNextNode < time_step);
                this->availableSince = schedTime;
                break;
            }
        }
    }
    else {
        this->timeToNextNode -= time_step;
        this->available = (this->timeToNextNode < time_step);
        this->availableSince = this->timeToNextNode + nowTime;
        if (this->available) {
            int schedTime = this->scheduledPath.front().first;
            int node = this->scheduledPath.front().second;
            if (!this->passengers.empty()) {
                int onboardCnt = 0;
                for (int i = 0; i < this->passengers.size(); i++) {
                    if (this->passengers[i].scheduledOnTime <= schedTime) {
                        if (this->passengers[i].unique == 1069) {
                            b29DroppedOff = true;
                        }
                        onboardCnt++;
                        pickups << to_string(now_time) + "," + to_string(this->passengers[i].unique) + ",on," + to_string(node) + "," +
                            to_string(schedTime) + "," + to_string(this->passengers[i].reqTime) + "\n";
                        requestDistances << to_string(this->passengers[i].reqTime) + "," + to_string(this->passengers[i].unique) + "," +
                            to_string(this->passengers[i].start) + "," + to_string(this->passengers[i].end) + "," + to_string(this->passengers[i].shortestDist) + "\n";
                    }
                    if (this->passengers[i].scheduledOffTime <= schedTime) {
                        if (this->passengers[i].unique == 1069) {
                            //b29DroppedOff = true;
                        }
                        onboardCnt--;
                        pickups <<  to_string(now_time) + "," + to_string(this->passengers[i].unique) + ",off," + to_string(node) + "," +
                            to_string(schedTime) + "," + to_string(this->passengers[i].expectedOffTime) + "\n";
                    }
                }
            }
            this->location = node;
            this->scheduledPath.pop();
            routes << to_string(now_time) + "," + to_string(schedTime) + "," + to_string(node) + "\n";
        }
    }

    routes.close();
    pickups.close();
    requestDistances.close();

    if (b29IsPassenger ==true) {
        int x = 5;
    }
    //From here below looks good; logging needs upgrades though
    int baseTime = this->available ? nowTime + this->timeToNextNode : nowTime; //TODO review this
    vector<Request> newPassengers;
    for (auto iterPsngr = this->passengers.begin(); iterPsngr != this->passengers.end(); iterPsngr++) {
        // hasn't got on board
        if (iterPsngr->scheduledOnTime > baseTime) {
            iterPsngr->status = Request::waiting; 
            #pragma omp critical(pushbackreq)
            newRequests.push_back(*iterPsngr);
        }
        else if (iterPsngr->scheduledOffTime <= baseTime) {
            // already got off
            iterPsngr->status = Request::droppedOff; //TODO is this addition needed?
            served_reqs++;
            // printf("%d off, ", iterPsngr->unique); //TODO log the trip as completed in a CSV somewhere
            total_wait_time += iterPsngr->scheduledOnTime - iterPsngr->reqTime; //TODO also tally delay time using scheduledOffTime
        }
        else { // now on board
            iterPsngr->status = Request::onBoard;
            // printf("%d on, ", iterPsngr->unique);
            newPassengers.push_back(*iterPsngr);
        }
    }
    // printf("\n");
    this->passengers = newPassengers;

}

void Vehicle::set_path(vector<pair<int, int>>& path) {
    while (!this->scheduledPath.empty()) {
        this->scheduledPath.pop();
    }

    for (auto it = path.begin(); it != path.end(); it++) {
        this->scheduledPath.push(*it);
    }
    for (int i = 0; i < this->passengers.size(); i++) {
        int origin = this->passengers[i].start;
        int originTime = this->passengers[i].scheduledOnTime;
        int schedStartTime = this->scheduledPath.front().first;
        int destination = this->passengers[i].end;
        bool bDIncluded = false;
        bool bOIncluded = false;
        if (this->get_location() == origin) {
            bOIncluded = true;
        }
        for (auto iter = this->scheduledPath.begin(); iter != this->scheduledPath.end(); iter++) {
            if (iter->second == destination) {
                bDIncluded = true;
            }
            if (iter->second == origin) {
                bOIncluded = true;
            }
        }
        if (!bDIncluded || (originTime >= schedStartTime && !bOIncluded)) {
            int x = 0;
        }
    }
}

void Vehicle::finish_route(int idx) {
    std::ofstream routes;
    routes.open(outDir + "Routes/" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);
    std::ofstream pickups;
    pickups.open(outDir + "Pickups/" + to_string(idx) + ".csv", std::ofstream::out | std::ofstream::app);

    if (!this->passengers.empty()) {
        for (auto iterPsngr = this->passengers.begin(); iterPsngr != this->passengers.end(); iterPsngr++) {
            these_served_reqs++;
            served_reqs++;
            // printf("%d on, ", iterPsngr->unique);
            total_wait_time += iterPsngr->scheduledOnTime - iterPsngr->reqTime;
        }
        // printf("\n");

        while (!this->scheduledPath.empty()) {
            int schedTime = this->scheduledPath.front().first;
            int node = this->scheduledPath.front().second;
            if (!this->passengers.empty()) {
                int onboardCnt = 0;
                for (int i = 0; i < this->passengers.size(); i++) {
                    if (this->passengers[i].scheduledOnTime <= schedTime) {
                        onboardCnt++;
                        pickups << to_string(now_time) + "," + to_string(this->passengers[i].unique) + ",on," + to_string(node) + "," +
                            to_string(schedTime) + "," + to_string(this->passengers[i].reqTime) + "\n";
                    }
                    if (this->passengers[i].scheduledOffTime <= schedTime) {
                        onboardCnt--;
                        pickups << to_string(now_time) + "," + to_string(this->passengers[i].unique) + ",off," + to_string(node) + "," +
                            to_string(schedTime) + "," + to_string(this->passengers[i].expectedOffTime) + "\n";
                    }
                }

            }
            this->location = node;
            this->scheduledPath.pop();
        }
    }
    routes.close();
    pickups.close();
}

