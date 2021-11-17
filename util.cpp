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
#include <chrono>
#include "util.h"
#include <unordered_set>
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
    while (num != EOF) {
        num = fscanf(in, "%d\n", &loc);
        if (num != EOF) {
            vehicles.push_back(Vehicle(loc));
            max_vehicle++;
        }
    }
    fclose(in);
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
    int start, end, reqTime;
    these_reqs = 0;
    these_served_reqs = 0;
    while (num != EOF) {
        num = fscanf(in, "%d,%d,%d\n", &reqTime, &start, &end);
        if (num != EOF) {
            if (start != end) {
                Request r(start, end, reqTime);
                r.shortestDist = -1; //TODO add this as something read in 
                r.expectedOffTime = reqTime + ceil((double(r.shortestDist)) / velocity);
                requests.push_back(r);
                raw_dist += r.shortestDist;
                total_reqs++;
                these_reqs++;
                if (reqTime > toTime) {
                    return true;
                }
            }
        }
    }
    return false;
}

void handle_unserved(vector<Request>& unserved, vector<Request>& requests,
    int nowTime) {
    
    for (auto iter = unserved.begin(); iter != unserved.end(); iter++) {
        if (nowTime - iter->reqTime <= max_wait_sec) {
            iter->status = Request::waiting;
            iter->scheduledOnTime = -1;
            requests.push_back(*iter);
        } else {
            unserved_dist += iter->shortestDist;
        }
    }
}

void update_vehicles(vector<Vehicle>& vehicles, vector<Request>& requests,
    int nowTime) {
    int i = 0;
    for (auto it = vehicles.begin(); it != vehicles.end(); it++) {
        it->update(nowTime, requests, i);
        i++;
    }
}

void finish_all(vector<Vehicle>& vehicles, vector<Request>& unserved) {
    for (int i = 0; i < vehicles.size(); i++) {
        vehicles[i].finish_route(i);
    }
    for (auto iter = unserved.begin(); iter != unserved.end(); iter++) {
        unserved_dist += iter->shortestDist;
    }
}

void setupOutfiles(const std::string& outDir, const std::string& outFilename){
    std::filesystem::create_directories(outDir);
    std::filesystem::create_directories(outDir + "GurobiLogs/");
    std::filesystem::create_directories(outDir + "Routes/");
    std::filesystem::create_directories(outDir + "Pickups/");
    std::filesystem::create_directories(outDir + "Misc/");
    std::filesystem::create_directories(outDir + "Code/");
	//std::filesystem::copy("/ocean/projects/eng200002p/mbruchon/Pooling", outDir + "Code/");
	//system(("cp -p /ocean/projects/eng200002p/mbruchon/RidePooling/*.* " + outDir + "Code/").c_str());
    std::ofstream ofs;
    ofs.open(outDir + outFilename, std::ofstream::out | std::ofstream::app);
    ofs << "NowTime,CPUTime,TotalReqs,ServedReqs,TotalDist,UnservedDist,RawDist,WaitTime,Pooled1,Pooled2,Pooled3,Pooled4,DisconnectedCars\n";
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
    std::ofstream ofs;
    ofs.open(outDir + outFilename, std::ofstream::out | std::ofstream::app);
    ofs << std::to_string(elapsed_seconds.count()) + ": " + message + "\n";
    ofs.close();
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
                to_string(iter->status) + "\n";
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
