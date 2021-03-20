#include <cstdio>
#include <cmath>
#include <cassert>
#include <ctime>
#include <map>
#include "util.h"
#include <set>
#include <algorithm>
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
/// <param name="nowTime">Current time (nowTime + vehicle's timeToNextNode when called from travel)</param>
/// <param name="decided">false=just determine cost; true=assign the path (passed through from travel)</param>
void TravelHelper::dfs(Vehicle& vehicle, Request *reqs[], int numReqs,
    set<int>& target, map<int, set<int> >& src_dst,
    vector<pair<int, int> >& path, vector<Request>& schedule,
    map<int, int>& occupancyChanges,
    map_of_pairs& dist,
    int travelled, int nowDelays, int& nowTime, bool decided, bool feasibilityCheck, bool simplestCheck) {

    /* If there's nowhere the car still needs to go, finished!*/
    if (target.size() == 0) {
        if (nowDelays < ansDelays) {
            ansDelays = nowDelays;
            //ansCost = nowCost;
            ansTravelled = travelled;
            if (decided) {
                ansPath = path;
                ansSchedule = schedule;
            }
        }
        return;
    }

    int prevLoc = vehicle.get_location(); //TODO is this "NextNode" or current location?
    /* Initialize tmpTarget to be all elements in (integer-sorted) set */
    vector<int> tmpTarget(target.begin(), target.end());
    vector<pair<int, int>> getOns, getOffsReq; //request index, time
    vector<int> getOffsPsngr, inserted;
    // try to arrive at a target
    //TODO: change this to only look for dropoffs when the car is already full?? unclear how much this might optimize things.
    for (int idx = 0; idx < tmpTarget.size(); idx++) {
        int node = tmpTarget[idx];
        inserted.clear();
        getOns.clear();
        getOffsReq.clear();
        getOffsPsngr.clear();

        /* Get the time from current location to node*/
        int interDist = get_dist(prevLoc, node, dist, simplestCheck);
        int newTime = nowTime + ceil((double(interDist)) / velocity);  //TODO PROBLEM: negative costs if the person gets picked up EARLY. they shouldn't be picked up early.
        /* If it's impossible to get to any of the request pickup spots before their max wait time, 
           this node shouldn't be visited at this point, so move on to the next node*/
        bool exceeded = false;
        for (int i = 0; i < numReqs; i++) {
            // exceed max waiting time
            if (reqs[i]->status== Request::waiting && newTime > reqs[i]->reqTime + max_wait_sec) {
                exceeded = true;
                break;
            }
        }

        /* TODO: maybe some optimizations to be made if exceeded = TRUE here */



        /* Set a flag for whether this node in tmpTarget has already been visited*/
        bool visited = false;
        for (int m = 0; m < path.size(); m++) {
            if (path[m].second == node) {
                visited = true;
                break;
            }
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
                if (target.insert(*iterDst).second) {
                    // record nodes newly inserted into target
                    inserted.push_back(*iterDst);
                }
                iterDst++;
            }
            for (int i = 0; i < numReqs; i++) {
                if (reqs[i]->start == node && reqs[i]->status==Request::waiting) {
                    reqs[i]->status = Request::onBoard;
                    int timeDiff = reqs[i]->reqTime - newTime;
                    if (timeDiff > 0) {
                        // TODO maybe add this back? 
                        nowTime += timeDiff; //TODO should there be a +1 here?
                        newTime = reqs[i]->reqTime;
                    }
                    if (decided) {
                        reqs[i]->scheduledOnTime = newTime;
                    }
                    vehicle.updateOccupancyTracker(occupancyChanges, newTime, 1);

                    // record who got on

                    if (reqs[i]->start == 566 && reqs[i]->end == 2956) {
                        int x = 5;
                    }
                    getOns.push_back(make_pair(i,newTime));
                }
            }
			}
        }

        int newDelays = nowDelays;

        /* 1. Set "exceeded" flag if expectedOffTime isn't OK for any onBoard requests 
           2. For onBoard requests before "exceeded" is set, if this node in tmpTarget is the dropoff point:
                a. Clear onBoard flag for the request
                b. Increment cost with this trip's travel delay
                c. Add request's index+time to getOffsReq
        */
        if (!exceeded) {
            for (int i = 0; i < numReqs; i++) {
                if (reqs[i]->status==Request::onBoard) {
                    // total delay time exceeded
                    if (newTime - reqs[i]->expectedOffTime > max_delay_sec) {
                        exceeded = true;
                        break; //TODO why isn't this continue?? some requests aren't checked
                    }
                    if (reqs[i]->end == node) {
                        reqs[i]->status = Request::droppedOff;
                        if (decided) {
                            reqs[i]->scheduledOffTime = newTime;
                            schedule.push_back(*reqs[i]);
                        }
                        vehicle.updateOccupancyTracker(occupancyChanges, newTime, -1);
                        int addlDelay = newTime - reqs[i]->expectedOffTime;
                        if (addlDelay > 0) {
                            newDelays += addlDelay;
                        }
                        // record who got off
                        getOffsReq.push_back(make_pair(i, newTime));
                    }
                }
            }
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
        //TODO why is this never called with decided==TRUE?
        vehicle.check_passengers(newTime, node, exceeded, newDelays,
            getOffsPsngr, schedule, occupancyChanges, decided);
        if (newDelays >= ansDelays) {
            exceeded = true;
        }

        if (!exceeded) {
            path.push_back(make_pair(newTime, node));
            target.erase(node);
            vehicle.set_location(node);
           
            dfs(vehicle, reqs, numReqs, target, src_dst, path, schedule, occupancyChanges, dist, travelled + interDist, newDelays, newTime, decided, feasibilityCheck, simplestCheck);

            vehicle.set_location(prevLoc);
            target.insert(node);
            path.pop_back();
        }

        // NOTE: mbruchon reversed this order: undo getoffs, then undo getons -- this should handle cars that pickup and dropoff one person in the time window
        vehicle.reverse_passengers(getOffsPsngr, schedule, decided);
        for (int m = 0; m < getOffsReq.size(); m++) {
            // printf("%d ", *iterRec);
            reqs[getOffsReq[m].first]->status = Request::onBoard;
            vehicle.updateOccupancyTracker(occupancyChanges, getOffsReq[m].second, 1);
        }

        // restore attribute "onBoard" of recorded reqs
        for (int m = 0; m < getOns.size(); m++) {
            // printf("%d ", *iterRec);
            reqs[getOns[m].first]->status = Request::waiting;
            vehicle.updateOccupancyTracker(occupancyChanges, getOns[m].second, -1);
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
        // printf("\n## b\n");
//        if (feasibilityCheck && ansTravelled > 0) {
//           return;
//        }
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
int TravelHelper::travel(Vehicle& vehicle, Request *reqs[], int numReqs,
map_of_pairs& dist, bool decided, bool feasibilityCheck, bool simplestCheck) {

    clock_t beginClock = clock();
    travel_cnt++;

    set<int> target; // Origin of all new reqs, plus destination of in-progress passengers
    map<int, set<int> > src_dst; // For all origins in reqs, a vector of attached destinations
    // insert new requests: s->t into src_dst
    for (int i = 0; i < numReqs; i++) {
        Request *req = reqs[i];
        src_dst[req->start].insert(req->end);
        target.insert(req->start);
    }
    // Insert vehicle's pre-existing passengers' destinations into temporary target set
    // Important note: set is auto-sorted using integer comparison, ie, order is pretty arbitrary
    vehicle.insert_targets(target);

    ansDelays = max_delay_sec * numReqs;
    ansTravelled = -1;
    ansSchedule.clear();

    vector<pair<int, int> > path;
    path.reserve(numReqs * 2 + 1);
    // path.push_back(vehicle.location); //TODO figure out why this isn't needed; probably because beginTime assumes next node???
    vector<Request> schedule;

    int beginTime = ((vehicle.isAvailable()) ? vehicle.getAvailableSince() : now_time) + vehicle.get_time_to_next_node();

    /* Set of: 
        time, 
        change in the car's # of passengers (-1 for dropoff, +1 for pickup), 
        request # (index into vector of requests)
    */
    map<int,int> occupancyChanges;
    vehicle.setup_occupancy_changes(occupancyChanges);
    dfs(vehicle, reqs, numReqs, target, src_dst, path, schedule, occupancyChanges, dist, 0, 0,
        beginTime, decided, feasibilityCheck, simplestCheck);
    
    if (ansTravelled >= 0) {
        if (decided) {
            int tmp = numReqs + vehicle.get_num_passengers();
            int schcnt = ansSchedule.size();

            vector<int> order;
            vector<pair<int, int> > finalPath;
            int prevNode = vehicle.get_location();
            int passedDist = 0;
            for (int m = 0; m < ansPath.size(); m++) {
                int node = ansPath[m].second;
                order.clear();
                find_path(prevNode - 1, node - 1, order);
                order[0] += 1;
                for (int i = 1; i < order.size(); i++) {
                    order[i] += 1;
                    passedDist += get_dist(order[i - 1], order[i], dist, simplestCheck);
                    finalPath.push_back(make_pair(beginTime + ceil((double(passedDist)) / velocity), order[i]));
                }
                prevNode = node;
            }
            vehicle.set_path(finalPath);
            vehicle.set_passengers(ansSchedule);
        }
        clock_t endClock = clock();
        double this_time = (double(endClock - beginClock)) / CLOCKS_PER_SEC;
        travel_max = max(travel_max, this_time);
        travel_time += this_time;
        // for (int i = 0; i < ansPath.size(); i++) {
            // printf("%d, ", ansPath[i]);
        // }
        // printf("\n");
        // printf("%d\n", ansTravelled);
        return ansDelays;
    } else {
        clock_t endClock = clock();
        double this_time = (double(endClock - beginClock)) / CLOCKS_PER_SEC;
        travel_max = max(travel_max, this_time);
        travel_time += this_time;
        return -1;
    }
}
