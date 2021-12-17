#pragma once

#include<map>
#include<vector>
#include "util.h"
#include "Vehicle.h"
#include "Request.h"

using namespace std;
class TravelHelper {
private:
	int ansDelays;
	int ansTravelled;
	int ansCost;
	int ansOffset;
	vector<pair<int, locReq> > ansPath;
	vector<Request> ansSchedule;
public:


	void dfs(Vehicle& vehicle, Request* reqs[], int numReqs,
		targetSet& target, map<locReq, set<locReq> >& src_dst,
		vector<pair<int, locReq> >& path, vector<Request>& schedule,
		int occupancy,
		int travelled, int nowDelays, int nowCost, int& beginTime, int beginOffset, const int nowTime,
		bool decided, bool observeReqTimeLimits, bool bFeasibilityCheck);
	int travel(Vehicle& vehicle, Request* reqs[], int numReqs, bool decided, bool observeReqTimeLimits = true, bool bFeasibilityCheck = false);
	int getTravelCost() { return ansCost; }
};
