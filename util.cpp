#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <random>
#include <map>
#include <cmath>
#include <chrono>
#include <unordered_set>
#include <omp.h>
#include "util.h"
#include "Vehicle.h"
#include "util.h"
#include "globals.h"
#include "GPtree.h"
using namespace std;

void save_vector(const vector<int>& v)
{
    printf("%d ", (int)v.size());
    for (int i = 0; i < (int)v.size(); i++)printf("%d ", v[i]);
    printf("\n");
}
void load_vector(vector<int>& v)
{
    v.clear();
    int n, i, j;
    int result = scanf("%d", &n);
    if (result <= 0) {
        exit(0);
    }
    if (n < 1000000000 && n >= 0) {
        v.reserve(n);
    }
    else {
        printf("%d\n", result);
        v.reserve(n / n);
        exit(0);
    }
    for (i = 0; i < n; i++)
    {
        scanf("%d", &j);
        v.push_back(j);
    }
}
void save_vector_vector(const vector<vector<int> >& v)
{
    printf("%d\n", (int)v.size());
    for (int i = 0; i < (int)v.size(); i++)save_vector(v[i]);
    printf("\n");
}
void load_vector_vector(vector<vector<int> >& v)
{
    v.clear();
    int n, i, j;
    scanf("%d", &n);
    v.reserve(n);
    vector<int>ls;
    for (i = 0; i < n; i++)
    {
        load_vector(ls);
        v.push_back(ls);
    }
}
void save_vector_pair(const vector<pair<int, int> >& v)
{
    printf("%d ", (int)v.size());
    for (int i = 0; i < (int)v.size(); i++)printf("%d %d ", v[i].first, v[i].second);
    printf("\n");
}
void load_vector_pair(vector<pair<int, int> >& v)
{
    v.clear();
    int n, i, j, k;
    scanf("%d", &n);
    v.reserve(n);
    for (i = 0; i < n; i++)
    {
        scanf("%d%d", &j, &k);
        v.push_back(make_pair(j, k));
    }
}
void save_map_int_pair(map<int, pair<int, int> >& h)
{
    printf("%zd\n", h.size());
    for (auto iter = h.begin(); iter != h.end(); iter++)
        printf("%d %d %d\n", iter->first, iter->second.first, iter->second.second);
}
void load_map_int_pair(map<int, pair<int, int> >& h)
{
    int n, i, j, k, l;
    scanf("%d", &n);
    vector<pair<int, pair<int, int>>> temp;
    temp.reserve(n);
    for (i = 0; i < n; i++)
    {
        scanf("%d%d%d", &j, &k, &l);
        temp.push_back(make_pair(j, pair<int, int>{k, l}));
        //h[j] = make_pair(k, l);
    }
    map<int, pair<int, int>> tempMap((temp.begin()), temp.end());
    h = tempMap;
}
void save_map_int_int(map<int, int>& h)
{
    printf("%zd\n", h.size());
    for (auto iter = h.begin(); iter != h.end(); iter++)
        printf("%d %d\n", iter->first, iter->second);
}
void load_map_int_int(map<int, int>& h)
{
    int n, i, j, k;
    scanf("%d", &n);
    vector<pair<int, int>> temp;
    temp.reserve(n);
    for (i = 0; i < n; i++)
    {
        scanf("%d%d", &j, &k);
        temp.push_back(make_pair(j, k));
        h[j] = k;
    }
    map<int, int> tempMap((temp.begin()), temp.end());
    h = tempMap;
}
void save_map_intpair_int(map_of_pairs& h, FILE* out)
{
    fprintf(out, "%zd\n", h.size());
    for (auto iter = h.begin(); iter != h.end(); iter++)
        fprintf(out, "%d %d %d\n", iter->first.first, iter->first.second, iter->second);
}
void load_map_intpair_int(map_of_pairs& h)
{
    int n, i, j, k;
    scanf("%d", &n);
    vector<pair<pair<int, int>, int>> temp;
    temp.reserve(n);
    for (i = 0; i < n; i++)
    {
        scanf("%d%d%d", &i, &j, &k);
        temp.push_back(make_pair(pair<int, int>{i, j}, k));
        //h[make_pair(i, j)] = k;
    }
    map_of_pairs tempMap((temp.begin()), temp.end());
    h = tempMap;
}

void save_map_intpair_intpair(pairs_to_pairs& h, FILE* out)
{
    fprintf(out, "%zd\n", h.size());
    for (auto iter = h.begin(); iter != h.end(); iter++)
        fprintf(out, "%d %d %d %d\n", iter->first.first, iter->first.second, iter->second.first, iter->second.second);
}
void load_map_intpair_intpair(pairs_to_pairs& h)
{
    int n, i, j, k, l;
    scanf("%d", &n);
    vector<pair<pair<int, int>, pair<int,int>>> temp;
    temp.reserve(n);
    for (i = 0; i < n; i++)
    {
        scanf("%d%d%d", &i, &j, &k, &l);
        temp.push_back(make_pair(pair<int, int>{i, j}, pair<int, int>{k, l}));
        //h[make_pair(i, j)] = k;
    }
    pairs_to_pairs tempMap((temp.begin()), temp.end());
    h = tempMap;
}

FILE* get_requests_file(const char* file) {
    FILE *in = NULL;
    in = fopen(file, "r");
    return in;
}

void read_vehicles(const char* file, vector<Vehicle>& vehicles) {
    vehicles.clear();
    printf("vehicle file = %s\n", file);
    FILE *in = NULL;
    in = fopen(file, "r");
    int loc;
    int num = 0;
    max_vehicle = 0;
    while (num != EOF && max_vehicle < fleet_size){
        num = fscanf(in, "%d\n", &loc);
        if (num != EOF) {
            vehicles.push_back(Vehicle(loc));
            max_vehicle++;
        }
    }
    fclose(in);
    vehicle_depot = vehicles[0].get_location();
}

void setup_vehicles(int nCars, vector<Vehicle>& vehicles) {
    vehicles.clear();
    int loc = 1;
    int num = 0;
    int idx = 0;
    for (idx = 0; idx < nCars; idx++) {
        vehicles.push_back(Vehicle(loc));
    }
}

bool read_requests(FILE*& in, vector<Request>& requests, int toTime) {
    
    int num = 0;
    int start, end, reqTime, dist;
    these_reqs = 0;
    these_served_reqs = 0;
    int counter = 0;
    while (num != EOF ) {
        num = fscanf(in, "%d,%d,%d,%d\n", &reqTime, &start, &end, &dist);
        if (num != EOF) {
            if (start != end) {
                Request r(start, end, reqTime);
                r.shortestDist = dist;
                r.expectedOffTime = reqTime + ceil((double(r.shortestDist)) / velocity);
                requests.push_back(r);
				//print_line(outDir, logFile, string_format("%d, %d, %d, %d.", reqTime, start, end, dist)); 
				
				/*
				std::vector<int> order;
				std::vector<int> suborder;
				std::vector<int> dists1;
				std::vector<int> dists2;
				std::vector<int> dists3;
                pair<int,int> result1 = treeCost.get_dist(start, end);
				pair<int,int> result3 = treeCost.find_path(start-1, end-1, order);
				int pathDist1 = 0;
				int pathDist2 = 0;
				int pathDist3 = 0;
				for(int i = 1; i < order.size(); i++){
					int thisDist1 = treeCost.get_dist(order[i - 1]+1, order[i]+1).second;
					int thisDist3 = treeCost.find_path(order[i - 1], order[i],suborder).second;
                    pathDist1 += thisDist1;
                    pathDist3 += thisDist3;
					dists1.push_back(thisDist1);
					dists3.push_back(thisDist3);
				}
				*/
/*				if(counter==0 | (result1.second != dist | result3.second != dist | result1.second != result3.second)){
					print_line(outDir,logFile, string_format("Counter = %d, result1.dist = %d,result3.dist = %d, trip.dist = %d, path.dist = %d,%d.",counter,result1.second,result3.second,dist,pathDist1,pathDist3));	
					for(int i = 1; i < order.size(); i++){
						print_line(outDir,logFile, string_format("Segment %d from %d to %d comparison:%d vs %d.",i,order[i-1],order[i],dists1[i-1],dists3[i-1]));
					}	
				}*/
				/*
				std::vector<int> suborder;
                pair<int,int> result = treeCost.find_path(start-1, end-1, order);
                order[0] += 1;
				print_line(outDir, logFile, string_format("%d, %d, %d, %d, %d, %d.", result1.first, result2.first, result.first, result1.second, result2.second, result.second));
				
                print_line(outDir, logFile, string_format("Request distance from %d to %d: %f dm (%f minutes).", r.start, r.end, result.second, (double)(r.expectedOffTime - r.reqTime)/60.0));

*/
                //print_line(outDir, logFile, string_format("Request distance: %d.", r.expectedOffTime - r.reqTime));
                raw_dist += r.shortestDist;
                total_reqs++;
                these_reqs++;
                if (reqTime > toTime) {
                    return true;
                }
                if (++counter >= req_per_window) {
                    toTime = reqTime;
					time_step = toTime - now_time;
                }
            }
        }
    }
    return false;
}

void update_requests_with_dist(FILE*& in) {
    int start, end, reqTime, dist, dist_new;
    these_reqs = 0;
    these_served_reqs = 0;
    int num = 0;
    std::ofstream ofs;
    ofs.open(baseInDir + "requests_with_dist_" + costs + ".csv", std::ofstream::out | std::ofstream::app);
    while (num != EOF) {
        num = fscanf(in, "%d,%d,%d, %d\n", &reqTime, &start, &end, &dist);
        if (num != EOF) {
            if (start != end) {
                dist_new = treeCost.get_dist(start, end).second;
                ofs << string_format("%d,%d,%d,%d\n", reqTime, start, end, dist_new);
            }
        }
    }
    ofs.close(); 
}

void handle_unserved(vector<Request>& unserved, vector<Request>& requests,
    int nowTime) {
	int overTimeLimit = 0;
    std::ofstream ofs;
    ofs.open(outDir + "Misc/unservedOverLimit.csv", std::ofstream::out | std::ofstream::app);
    for (auto iter = unserved.begin(); iter != unserved.end(); iter++) {
        //iter->allowedDelay = iter->allowedDelay + time_step;
        //iter->allowedWait = iter->allowedWait + time_step;
        if (nowTime - iter->reqTime <= iter->allowedWait) {
            iter->setStatus(Request::requestStatus::waiting);
            iter->scheduledOnTime = -INF;
            iter->scheduledOffTime = -INF;
            requests.push_back(*iter);
        }
        else {
            ofs << to_string(now_time) + "," + to_string(iter->unique) + "," + to_string(iter->reqTime) + "," + 
                to_string(iter->expectedOffTime) + to_string(iter->start) + to_string(iter->end) + "\n";
            overTimeLimit++;
        }
    }
    ofs.close();
	
	print_line(outDir, logFile, string_format("Unserved: %d (%d over time limit)", (int)unserved.size(), overTimeLimit));
}

void update_vehicles(vector<Vehicle>& vehicles, vector<Request>& requests, int nowTime) {
    int i = 0;
    #pragma omp parallel for num_threads(omp_get_max_threads()) default(none) private(i) shared(vehicles, nowTime, requests)
    for (i = 0; i < vehicles.size(); i++) {
        vehicles[i].update(nowTime, requests, i);
    }
}

void finish_all(vector<Vehicle>& vehicles, vector<Request>& unserved) {
    for (int i = 0; i < vehicles.size(); i++) {
        vehicles[i].finish_route(i, now_time);
    }
    for (auto iter = unserved.begin(); iter != unserved.end(); iter++) {
        unserved_dist += iter->shortestDist;
    }
}

void setupOutfiles(const std::string& outDir, const std::string& outFilename){
    std::filesystem::create_directories(outDir);
    std::filesystem::create_directories(outDir + "GurobiLogs/");
    std::filesystem::create_directories(outDir + "Routes/");
    std::filesystem::create_directories(outDir + "VehicleStatuses/");
    std::filesystem::create_directories(outDir + "Pickups/");
    std::filesystem::create_directories(outDir + "Misc/");
    std::filesystem::create_directories(outDir + "Misc/Trips/");
    std::filesystem::create_directories(outDir + "Code/");
	//std::filesystem::copy("/ocean/projects/eng200002p/mbruchon/Pooling", outDir + "Code/");
	system(("cp -p /ocean/projects/eng200002p/mbruchon/RidePooling/*.cpp " + outDir + "Code/").c_str());
	system(("cp -p /ocean/projects/eng200002p/mbruchon/RidePooling/*.h " + outDir + "Code/").c_str());
	system(("cp -p /ocean/projects/eng200002p/mbruchon/RidePooling/*.o " + outDir + "Code/").c_str());
	system(("cp -p /ocean/projects/eng200002p/mbruchon/RidePooling/Makefile " + outDir + "Code/").c_str());
    std::ofstream ofs;
    ofs.open(outDir + outFilename, std::ofstream::out | std::ofstream::app);
    ofs << "NowTime,CPUTime,TotalReqs,ServedReqs,TotalDist,UnservedDist,RawDist,WaitTime,Pooled1,Pooled2,Pooled3,Pooled4,DisconnectedCars\n";
    ofs.close();
	
	ofs.open(outDir + "VehicleStatuses/psgrCounter.csv", std::ofstream::out | std::ofstream::app);
	ofs << "time,val,count\n";
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/waitingCounter.csv", std::ofstream::out | std::ofstream::app);
	ofs << "time,val,count\n";
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/onBoardCounter.csv", std::ofstream::out | std::ofstream::app);
	ofs << "time,val,count\n";
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/areaCounter.csv", std::ofstream::out | std::ofstream::app);
	ofs << "time,val,count\n";
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/startOccupancy.csv", std::ofstream::out | std::ofstream::app);
	ofs << "time,val,count\n";
	ofs.close();
	ofs.open(outDir + "VehicleStatuses/endOccupancy.csv", std::ofstream::out | std::ofstream::app);
	ofs << "time,val,count\n";
	ofs.close();

}

void print_stats(const std::string& outDir, const std::string& outFilename) {
 /*   std::ofstream ofs;
    ofs.open(outDir + outFilename, std::ofstream::out | std::ofstream::app);
    ofs << to_string(now_time) + "," + to_string(clock()) + "," +
        to_string(these_reqs) + "," + to_string(these_served_reqs) + "," + 
        to_string(total_dist) + "," + to_string(unserved_dist) + "," +
        to_string(raw_dist) + "," + to_string(total_wait_time) + "," + 
        to_string(utilization[1]) + "," + to_string(utilization[2]) + "," + to_string(utilization[3]) + "," + to_string(utilization[4]) + "," +
        to_string(disconnectedCars) +
        "\n";
    ofs.close();
*/
}
void print_line(const std::string& outDir, const std::string& outFilename, const std::string& message) {
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-startTime;
	std::chrono::duration<double> elapsed_seconds_2 = end-lastTimeCheck;
    std::ofstream ofs;
    ofs.open(outDir + outFilename, std::ofstream::out | std::ofstream::app);
    ofs << std::to_string(elapsed_seconds.count()) + ": " + std::to_string(elapsed_seconds_2.count()) + ": "+ message + "\n";
	lastTimeCheck = end;
    ofs.close();
}

void print_ram(const std::string& outDir, const std::string& descriptor) {
	//system(("top -u mbruchon -n 1 -o %MEM > " + outFilename + ".txt").c_str());
	std::ofstream ofs;
    ofs.open(outDir + "memory_log.txt", std::ofstream::out | std::ofstream::app);
	ofs << std::left << std::setfill(' ') << std::setw(48) << descriptor + ": ";
    ofs.close();
	//system(("top -u mbruchon -n 1 -o %MEM > " + outFilename + ".txt").c_str());
	system(("top -u mbruchon -bn 1 | grep mbruchon | grep main >> " + outDir + "memory_log.txt").c_str());
	//system((std::string(R"(top -bn 1 | grep "^ " | awk '{ printf("%s%s%s%s\n","{CPU:"$9",","MEM:"$10",","CMD:"$12",","User:"$2"}"); }' | head -n 10 | tail -n +2 | grep mbruchon > )") +  outDir + "memory_log.txt").c_str());
}

void write_vehicle_routes(const std::string& outDir, const std::vector<Vehicle>& vehicles, int now_time) {
    std::ofstream ofs;
    for (int i = 0; i < vehicles.size(); i++) {
        ofs.open(outDir + "Routes/" + to_string(i) + ".csv", std::ofstream::out | std::ofstream::app);
        for (auto iter = vehicles[i].scheduledPath.begin(); iter != vehicles[i].scheduledPath.end(); iter++) {
            ofs << to_string(now_time) + "," + to_string(iter->first) + "," + to_string(iter->second) + "\n";
        }
        ofs.close();
        for (auto iter = vehicles[i].passengers.begin(); iter != vehicles[i].passengers.end(); iter++) {
            ofs.open(outDir + "Riders/" + to_string(iter->unique) + ".csv", std::ofstream::out | std::ofstream::app);
            ofs << to_string(now_time) + "," + to_string(i) + "," + to_string(iter->start) + "," + to_string(iter->end) + "," +
                to_string(iter->reqTime) + "," + to_string(iter->scheduledOnTime) + "," + to_string(iter->scheduledOffTime) + "," +
                to_string(static_cast<int>(iter->getStatus())) + "\n";
        }
        ofs.close();
    }
}

void log_stats() {
 // TBD
}

std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::random_device rd; 
    std::uniform_int_distribution<int> dist(0, 9);
    s = s + std::to_string(dist(rd)) + std::to_string(dist(rd));
    std::replace(s.begin(), s.end(), '-', '_');
    std::replace(s.begin(), s.end(), ':', '_');
    s.erase(7, 1);
    s.erase(12, 1);
    return s;
}

void getDisjunction(const uos& a, const uos& b, pair<int, int>& toReturn) {
    auto i = a.begin(), j = b.begin(), endA = a.end(), endB = b.end();
    toReturn.first = -1;
    int aNotB = 0;
    int bNotA = 0;
    while (i != endA && j != endB) {
        if (*j == *i) {
            ++j;
            ++i;
        }
        else if (*j < *i) {
            if (++bNotA < 2) {
                toReturn.first = 0;
                toReturn.second = *j;
                ++j;
            }
            else {
                toReturn.first = -1;
                return;
            }
        }
        else {
            if (++aNotB < 2) {
                toReturn.first = 1;
                toReturn.second = *i;
                ++i;
            }
            else {
                toReturn.first = -1;
                return;
            }
        }
    }//first = -1 for fail, 0 for add to a, 1 for add to b; second: the thing to add
}

