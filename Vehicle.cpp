#include <cmath>
#include <vector>
#include <queue>
#include <set>
#include <unordered_map>
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
void Vehicle::check_passengers(int nowTime, int stop, bool& exceeded, int& sumDelays, vector<int>& getOffPsngr, vector<Request>& schedule,
    map<int, int>& occupancyChanges, bool decided) {

    auto it = occupancyChanges.begin();
    int occupancy = 0;
    auto itEnd = occupancyChanges.end();
    while (it != itEnd) {
        occupancy += it->second;
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

void Vehicle::head_for(int node, map_of_pairs& dist) {
    vector<int> order;
    find_path(this->location - 1, node - 1, order);
    while (!this->scheduledPath.empty()) {
        this->scheduledPath.pop();
    }
    int distTravelled = 0;
    int tmpNode = this->location;
    int baseTime = (this->timeToNextNode < time_step) ?
        now_time + this->timeToNextNode : now_time - this->timeToNextNode;
    for (int i = 1; i < order.size(); i++) { // head is location itself
        order[i] += 1;
        distTravelled += get_dist(tmpNode, order[i], dist);
        int tmpTime = baseTime + ceil((double(distTravelled)) / velocity);
        this->scheduledPath.push(make_pair(tmpTime, order[i]));
    }
}

void Vehicle::update(int nowTime, vector<Request>& newRequests, map_of_pairs& dist) {

    if (this->scheduledPath.empty()) {
        return;
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
                        }
                        if (this->passengers[i].scheduledOffTime == schedTime) {
                            onboardCnt--;
                        }
                    }
                    if (onboardCnt > 0) {
                        total_dist += get_dist(this->location, node, dist);
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
            }
            if (schedTime >= nowTime) {
                this->timeToNextNode = schedTime - nowTime;
                this->available = (this->timeToNextNode < time_step);
                this->availableSince = this->timeToNextNode + nowTime;
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
                    if (this->passengers[i].scheduledOnTime < schedTime) {
                        onboardCnt++;
                    }
                    if (this->passengers[i].scheduledOffTime < schedTime) {
                        onboardCnt--;
                    }
                }
                if (onboardCnt > 0) {
                    total_dist += get_dist(this->location, node, dist);
                }
            }
            this->location = node;
            this->scheduledPath.pop();
        }
    }

    int baseTime = this->available ? nowTime + this->timeToNextNode : nowTime; //TODO review this
    vector<Request> newPassengers;
    for (auto iterPsngr = this->passengers.begin(); iterPsngr != this->passengers.end(); iterPsngr++) {
        // hasn't got on board
        if (iterPsngr->scheduledOnTime > baseTime) {
            iterPsngr->status = Request::waiting; //TODO is this addition needed?
            newRequests.push_back(*iterPsngr);
        }
        else if (iterPsngr->scheduledOffTime <= baseTime) {
            // already got off
            iterPsngr->status = Request::droppedOff; //TODO is this addition needed?
            served_reqs++;
            // printf("%d off, ", iterPsngr->unique);
            total_wait_time += iterPsngr->scheduledOnTime - iterPsngr->reqTime;
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
}

void Vehicle::finish_route(map_of_pairs& dist) {
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
                    if (this->passengers[i].scheduledOnTime < schedTime) {
                        onboardCnt++;
                    }
                    if (this->passengers[i].scheduledOffTime < schedTime) {
                        onboardCnt--;
                    }
                }
                if (onboardCnt > 0) {
                    total_dist += get_dist(this->location, node, dist);
                }
            }
            this->location = node;
            this->scheduledPath.pop();
        }
    }
}

