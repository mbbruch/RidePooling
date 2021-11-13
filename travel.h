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
	vector<pair<int, int> > ansPath;
	vector<Request> ansSchedule;
public:


	void dfs(Vehicle& vehicle, Request* reqs[], int numReqs,
		set<int>& target, map<int, set<int> >& src_dst,
		vector<pair<int, int> >& path, vector<Request>& schedule,
		map<int,int>& occupancyChanges,
		int travelled, int nowDelays, int nowCost, int& nowTime, int nowOffset, 
		bool decided, bool feasibilityCheck, bool simplestCheck);
	int travel(Vehicle& vehicle, Request* reqs[], int numReqs, bool decided, bool feasibilityCheck = true, bool simplestCheck = false);
	int getTravelCost() { return ansTravelled * 0.0001; }
};
