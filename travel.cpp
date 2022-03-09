#include <cstdio>
#include <cmath>
#include <cassert>
#include <ctime>
#include <map>
#include <set>
#include <algorithm>
#include <omp.h>
#include "util.h"
#include "travel.h"
#include "GPtree.h"
#include "globals.h"
using namespace std;

/// <summary>
/// 
/// </summary>
/// <param name="vehicle">Vehicle to test (uses its starting location and current trip dropoff point)</param>
/// <param name="reqs">requests to test (not including current request? TODO verify!)</param>
/// <param name="numReqs">This seems to be the size of reqs</param>
/// <param name="target">Set with origin of all new reqs, plus each in progress passengers' destination (TODO should be all the pickups and dropoffs??)</param>
/// <param name="src_dst">For each distinct origin in reqs, a vector of attached destinations</param>
/// <param name="path">Current path (empty when called from travel)</param>
/// <param name="schedule">Current ordering of requests (empty when called from travel)</param>
/// <param name="dist">Cache of node-to-node travel distances</param>
/// <param name="travelled">(0 when called from travel)</param>
/// <param name="nowDelays">Current cost (0 when called from travel)</param>
/// <param name="beginTime">Current time (nowTime + vehicle's timeToNextNode when called from travel)</param>
/// <param name="decided">false=just determine cost; true=assign the path (passed through from travel)</param>
void TravelHelper::dfs(Vehicle& vehicle, Request *reqs[], const int numReqs,
    targetSet& target, map<locReq, set<locReq> >& src_dst,
    vector<pair<int, locReq> >& path, vector<Request>& schedule,
    const int beginOccupancy, const int beginTotalCost, const int beginTime, const int beginOffset, 
    const bool decided, const bool observeReqTimeLimits, const bool bFeasibilityCheck) {

    /* If there's nowhere the car still needs to go, finished!*/
    if (target.size() == 0) {
        if (beginTotalCost < ansCost) {
            ansCost = beginTotalCost;
            ansOffset = beginOffset;
            if (decided) {
                ansPath = path;
                ansSchedule = schedule;
            }
        }
        return;
    }

    const int prevLoc = vehicle.get_location();
    /* Initialize tmpTarget to be all elements in (integer-sorted) set */
    vector<locReq> tmpTarget(target.begin(), target.end());
    vector<int> getOnsReq, getOffsReq, getOnsPsngr, getOffsPsngr; //request index, time
    getOnsReq.reserve(numReqs);
    getOffsReq.reserve(numReqs);
    getOnsPsngr.reserve(vehicle.get_num_passengers());
    getOffsPsngr.reserve(vehicle.get_num_passengers());
    vector<locReq> inserted;
    // try to arrive at a target
    //TODO: change this to only look for dropoffs when the car is already full?? unclear how much this might optimize things.
    for (int idx = 0; idx < tmpTarget.size(); idx++) {
        const locReq& node = tmpTarget[idx];
        inserted.clear();
        getOnsReq.clear();
        getOnsPsngr.clear();
        getOffsReq.clear();
        getOffsPsngr.clear();
        int occupancy = beginOccupancy;
        int newTotalCost = beginTotalCost;
        int newOffset = beginOffset;
        int newPickups = 0;
        int newDelays = 0;
        int currentWaits = 0;
        /* Get the time from current location to node*/
        int interDist = 0;
        std::pair<int,int> result = treeCost.get_dist(prevLoc, node.first);
        newTotalCost += result.first;
        interDist += result.second;
        int newTime = beginTime + ceil(static_cast<double>(interDist) / velocity * 1.0);
        /* If it's impossible to get to any of the request pickup spots before their max wait time, 
           this node shouldn't be visited at this point, so move on to the next node*/
        bool exceeded = false;
        if (observeReqTimeLimits) {
            for (int i = 0; i < numReqs; i++) {
                // exceed max waiting time
                int thisWait = newTime - reqs[i]->reqTime;
                if (reqs[i]->getStatus() == Request::requestStatus::waiting && thisWait > 0) {
                    if (thisWait > reqs[i]->allowedWait) {
                        exceeded = true;
                        break;
                    }
                    currentWaits += (newTime - reqs[i]->reqTime);
                }
            }
        }
        if (newTotalCost + delayPenalty * (newDelays + currentWaits) + pickupPenalty * newPickups > ansCost) {
            exceeded = true;
        }
        /* Set a flag for whether this node in tmpTarget has already been visited*/
        bool visited = false;
        if (!exceeded) {
            for (int m = 0; m < path.size(); m++) {
                if (path[m].second == node) {
                    visited = true;
                    break;
                }
            }
        }

        /* 1. Set "exceeded" flag if expectedOffTime isn't OK for any onBoard requests
           2. For onBoard requests before "exceeded" is set, if this node in tmpTarget is the dropoff point:
                a. Clear onBoard flag for the request
                b. Increment cost with this trip's travel delay
                c. Add request's index+time to getOffsReq
        */
        if (!visited && !exceeded) {
            for (int i = 0; i < numReqs; i++) {
                if (reqs[i]->getStatus() == Request::requestStatus::onBoard) {
                    // total delay time exceeded
                    int thisWait = newTime - reqs[i]->expectedOffTime;
                    if (observeReqTimeLimits && (thisWait > reqs[i]->allowedDelay)) {
                        exceeded = true;
                        break;
                    }
                    if (reqs[i]->end == node.first && reqs[i]->unique == node.second) {
                        occupancy--;
                        reqs[i]->setStatus(Request::requestStatus::droppedOff);
                        if (decided) {
                            reqs[i]->scheduledOffTime = newTime;
                            schedule.push_back(*reqs[i]);
                        }
                        int addlDelay = newTime - reqs[i]->expectedOffTime;
                        if (addlDelay > 0) {
                            newDelays += addlDelay;
                        }
                        // record who got off
                        getOffsReq.push_back(i);
                    }
                    else if(thisWait > 0){
                        currentWaits += thisWait;
                    }
                }
            }
        }
        if (newTotalCost + delayPenalty * (newDelays + currentWaits) + pickupPenalty * newPickups > ansCost) {
            exceeded = true;
        }

        /* If this node in tmpTarget is not already visited AND is a get-on node: 
            1. Insert all drop-off nodes of requests starting at the node into target
            2. Record that insertion in the "inserted" vector 
            3. Set all of those requests as onBoard
            4. Add all of those requests to "getOns" vector
        */
        if (!visited && !exceeded) {
            auto itNode = src_dst.find(node);
			if(itNode != src_dst.end()){
                auto iterDst = (*itNode).second.begin();
                while (iterDst != (*itNode).second.end()) {
                    target.insert(*iterDst);
                    inserted.push_back(*iterDst);
                    ++iterDst;
                }
                for (int i = 0; i < numReqs; i++) {
                    if (reqs[i]->start == node.first && reqs[i]->unique ==node.second && reqs[i]->getStatus()==Request::requestStatus::waiting) {
                        newPickups += occupancy * 2;
                        if (++occupancy > max_capacity) {
                            exceeded = true;
                            break;
                        }
                        reqs[i]->setStatus(Request::requestStatus::onBoard);
                        int earliestArrivalAllowed = max(now_time, reqs[i]->reqTime);
                        int timeDiff = earliestArrivalAllowed - newTime;
                        if (timeDiff > 0) {
                            if (newTime < 0) {
                                newOffset = timeDiff;
                            }
                            newTime = earliestArrivalAllowed;
                        }
                        if (decided) {
                            reqs[i]->scheduledOnTime = newTime;
                        }
                            // record who got on
                        getOnsReq.push_back(i);
                        break;
                    }
                }
            }
        }
        if (newTotalCost + delayPenalty * (newDelays + currentWaits) + pickupPenalty * newPickups > ansCost) {
            exceeded = true;
        }

        /* For each preexisting passenger in the vehicle, IF the request is onBoard:
            1. If newTime is already too late to drop it off on time, set "exceeded" flag 
            2. Otherwise, IF this node in tmpTarget is the dropoff point:
                a. set onBoard = FALSE for the request
                b. push back the request onto schedule
                c. push back the request onto getOffsPassenger
                d. update scheduledOffTime to be newTime
                e. add the request's delay time to newDelays
         */
        if (!visited && !exceeded) {
            vehicle.check_passengers(newTime, node, exceeded, currentWaits, newDelays, newOffset, newPickups,
                getOffsPsngr, getOnsPsngr, schedule, occupancy, decided);
            newTotalCost = newTotalCost + newPickups * pickupPenalty + newDelays * delayPenalty;
            if (newTotalCost + delayPenalty * currentWaits >= ansCost) {
                exceeded = true;
            }
        }

        if (!visited && !exceeded) {
            path.push_back(make_pair(newTime, node));
            target.erase(node);
            vehicle.set_location(node.first);
           
            dfs(vehicle, reqs, numReqs, target, src_dst, path, schedule, occupancy, newTotalCost, newTime, newOffset, decided, observeReqTimeLimits, bFeasibilityCheck);

            vehicle.set_location(prevLoc);
            if (bFeasibilityCheck && ansCost != INF) {
                return;
            }
            target.insert(node);
            path.pop_back();
        }

        // NOTE: mbruchon reversed this order: undo getoffs, then undo getons -- this should handle cars that pickup and dropoff one person in the time window
        vehicle.reverse_passengers(getOffsPsngr, getOnsPsngr, occupancy, schedule, newOffset, decided);
        for (int m = 0; m < getOffsReq.size(); m++) {
            // printf("%d ", *iterRec);
            reqs[getOffsReq[m]]->setStatus(Request::requestStatus::onBoard);
            occupancy++;
        }

        // restore attribute "onBoard" of recorded reqs
        for (int m = 0; m < getOnsReq.size(); m++) {
            // printf("%d ", *iterRec);
            reqs[getOnsReq[m]]->setStatus(Request::requestStatus::waiting);
            occupancy--;
        }

        if (decided) {
            size_t offCnt = getOffsReq.size();
            while (offCnt > 0) {
                    schedule.pop_back();
                offCnt--;
            }
        }
        for (auto itRec = inserted.begin(); itRec != inserted.end(); itRec++) {
            target.erase(*itRec);
        }
    }
}

/// <summary>
/// Gets the cost (and possibly assigns the path) to serve the specified paths
/// </summary>
/// <param name="vehicle">Vehicle to test (uses its starting location and current trip dropoff point)</param>
/// <param name="reqs">requests to test (not including current request? TODO verify!)</param>
/// <param name="numReqs">This seems to be the size of reqs</param>
/// <param name="dist">Cache of node-to-node travel distances</param>
/// <param name="decided">false=just determine cost; true=assign the path</param>
/// <returns>Cost (travel time)</returns>
int TravelHelper::travel(Vehicle& vehicle, Request *reqs[], int numReqs, bool decided, bool observeReqTimeLimits, bool bFeasibilityCheck) {

    clock_t beginClock = clock();

    targetSet target; // Origin of all new reqs, plus destination of in-progress passengers
    map<locReq, set<locReq> > src_dst; // For all origins in reqs, a vector of attached destinations
    // insert new requests: s->t into src_dst
    for (int i = 0; i < numReqs; i++) {
        Request *req = reqs[i];
        src_dst[make_pair(req->start,req->unique)].insert(make_pair(req->end,req->unique));
        target.insert(make_pair(req->start,req->unique));
    }

    int beginTime = vehicle.getAvailableSince();
    vehicle.fixPassengerStatus(beginTime);
    for (int i = 0; i < numReqs; i++) {
        reqs[i]->fixStatus(beginTime);
    }
    /*
    vector<Request::requestStatus> pre_statuses_veh(vehicle.get_num_passengers(),Request::requestStatus::waiting);
    vector<Request::requestStatus> pre_statuses_req(numReqs, Request::requestStatus::waiting);
    for (int i = 0; i < vehicle.get_num_passengers(); i++) {
        pre_statuses_veh[i] = vehicle.passengers[i].getStatus();
    }
    for (int i = 0; i < numReqs; i++) {
        pre_statuses_req[i] = reqs[i]->getStatus();
    }
*/
    // Insert vehicle's pre-existing passengers' destinations into temporary target set
    // Important note: set is auto-sorted using integer comparison, ie, order is pretty arbitrary
    vehicle.insert_targets(target, src_dst, beginTime);

    ansCost = INF;
    ansSchedule.clear();

    vector<pair<int, locReq> > path;
    path.reserve(numReqs * 2 + 1);
    // path.push_back(vehicle.location); //TODO figure out why this isn't needed; probably because beginTime assumes next node???
    vector<Request> schedule;
    /* Set of: 
        time, 
        change in the car's # of passengers (-1 for dropoff, +1 for pickup), 
        request # (index into vector of requests)
    */
    Request reqCopy = *(reqs[0]);
    int occupancyStart = vehicle.getOccupancyAt(beginTime);
    assert(!vehicle.offline || bFeasibilityCheck);
    dfs(vehicle, reqs, numReqs, target, src_dst, path, schedule, 
        occupancyStart, 0, beginTime, 0, 
        decided, observeReqTimeLimits, bFeasibilityCheck);
/*
    if (!decided) {
        for (int i = 0; i < vehicle.get_num_passengers(); i++) {
            vehicle.passengers[i].setStatus(pre_statuses_veh[i]);
        }
        for (int i = 0; i < numReqs; i++) {
            reqs[i]->setStatus(pre_statuses_req[i]);
        }
    }
*/
    if (ansCost != INF) {
        if (decided) {
            if (vehicle.get_num_passengers() == 1 && vehicle.passengers[0].unique == 0) {
                int x = 5;
            }
            else {
                if (now_time > 900) {
                    int x = 5;
                }
            }
            beginTime += ansOffset;
            int passedDist = 0;
            vector<int> order;
            vector<pair<int, int> > finalPath;
            int prevNode = vehicle.get_location();
            if (ansOffset != 0) {
                finalPath.push_back(make_pair(beginTime, prevNode));
            }
            for (int m = 0; m < ansPath.size(); m++) {
                if (m == 0 && ansPath[m].second.first == vehicle.get_location() && ansPath[m].first == vehicle.getAvailableSince() && !vehicle.getWasIdle()) {
                    continue;
                }
                /*
                if (finalPath.size() > 0) {
                    beginTime = finalPath[finalPath.size() - 1].first;
                }*/
                int node = ansPath[m].second.first;
                pair<int, int> fpResult;
                order.clear();
                #pragma omp critical (findpath)
                fpResult = treeCost.find_path(prevNode - 1, node - 1, order);
                order[0] += 1;
                for (int i = 1; i < order.size(); i++) {
                    order[i] += 1;
                    passedDist += treeCost.get_dist(order[i - 1], order[i]).second;
                    if (finalPath.size() > 0) {
                        if (finalPath.back().first > beginTime + ceil(static_cast<double>(passedDist) / velocity * 1.0)) {
                            int x = 5;
                        }
                    }
                    finalPath.push_back(make_pair(beginTime + ceil(static_cast<double>(passedDist) / velocity * 1.0), order[i]));
                }
                if (finalPath.size() > 0) {
                    int diff = finalPath[finalPath.size() - 1].first - ansPath[m].first;
                    if (diff < 0) {
                        /* TODO fix this case:
                        @ time 32700
                            scheduledPath:
                            32414, 2017
                            32504, 2017
                            32414, 2017
                            32504, 2017
                            32639, 2017
                            passengers :
                            2569 - 2017 reqTime 30214, expOff 31704, scheduledOn 30465, scheduledOff 32414
                            2017 - 3315 reqTime 32504, expOff 33709, scheduledOn 32504, scheduledOff 33844
                            2017 - 3172 reqTime 32639, expOff 34222, scheduledOn 32639, scheduledOff 34391
                        */
                        int min = finalPath[finalPath.size() - 1].first;
                        int max = ansPath[m].first;
                        set<int> timesToAdd;
                        timesToAdd.insert(max);
                        for (auto it = ansSchedule.begin(); it != ansSchedule.end(); it++) {
                            if (it->scheduledOnTime <= min && it->end == ansPath[m].second.first) {
                                timesToAdd.insert(it->scheduledOffTime);
                            }
                            if (it->scheduledOnTime >= min && it->start == ansPath[m].second.first && it->scheduledOnTime < max) {
                                timesToAdd.insert(it->scheduledOnTime);
                            }
                        }
                        if (m == 0 && node == vehicle.get_location() && timesToAdd.size()==1) {
                            finalPath[finalPath.size() - 1].first = *(timesToAdd.begin());
                        }
                        else {
                            for (auto it = timesToAdd.begin(); it != timesToAdd.end(); it++) {
                                //car arrived early but must wait for pickups and/or dropoffs
                                finalPath.push_back(make_pair(*it, ansPath[m].second.first));
                            }
                        }
                    }
                    else if(diff > 0) {
                        //TODO investigate why this is needed sometimes: occurs when summed distance of 
                        //finalPath's segments is longer than distance of O to D
                        ansPath[m].first = finalPath[finalPath.size() - 1].first;
                        for (int m = 0; m < ansSchedule.size(); m++) {
                            Request& thisReq = ansSchedule[m];
                            if (thisReq.unique == ansPath[m].second.second) {
                                if (thisReq.start == finalPath[finalPath.size() - 1].second) {
                                    thisReq.scheduledOnTime = finalPath[finalPath.size() - 1].first;
                                }
                                else if (thisReq.end == finalPath[finalPath.size() - 1].second) {
                                    if (thisReq.scheduledOnTime == -1) {
                                        int x = 5;
                                    }
                                    thisReq.scheduledOffTime = finalPath[finalPath.size() - 1].first;
                                }
                            }
                        }
                    }
                }
                prevNode = node;
            }
            vehicle.set_passengers(ansSchedule);
            vehicle.set_path(finalPath);
        }
        clock_t endClock = clock();
        double this_time = (double(endClock - beginClock)) / CLOCKS_PER_SEC;
        travel_max = max(travel_max, this_time);
        travel_time += this_time;
        // for (int i = 0; i < ansPath.size(); i++) {
            // printf("%d, ", ansPath[i]);
        // }
        // printf("\n");
        return ansCost;
    } else {
        clock_t endClock = clock();
        double this_time = (double(endClock - beginClock)) / CLOCKS_PER_SEC;
        travel_max = max(travel_max, this_time);
        travel_time += this_time;
        return -1;
    }
}
