#pragma once

#include<map>
#include<vector>
#include "util.h"
#include "Vehicle.h"
#include "Request.h"

using namespace std;
class TravelHelper {
private:
	int ansOffset;
	vector<pair<int, locReq> > ansPath;
	vector<Request> ansSchedule;
public:

	int ansCost;

	void dfs(Vehicle& vehicle, Request* reqs[], const int numReqs,
		targetSet& target, map<locReq, set<locReq> >& src_dst,
		vector<pair<int, locReq> >& path, vector<Request>& schedule,
		const int beginOccupancy, const int beginTotalCost, const int beginTime, const int beginOffset,
		const bool decided, const bool observeReqTimeLimits, const bool bFeasibilityCheck);
	int travel(Vehicle& vehicle, Request* reqs[], int numReqs, bool decided, bool observeReqTimeLimits = true, bool bFeasibilityCheck = false);
	int getTravelCost() { return ansCost; }
};
