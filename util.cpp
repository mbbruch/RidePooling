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

bool read_requests(FILE*& in, vector<Request>& requests, int toTime,
    map_of_pairs& dist) {
    
    int num = 0;
    int start, end, reqTime;
    these_reqs = 0;
    these_served_reqs = 0;
    while (num != EOF) {
        num = fscanf(in, "%d,%d,%d\n", &reqTime, &start, &end);
        if (num != EOF) {
            if (start != end) {
                Request r(start, end, reqTime);
                r.shortestDist = get_dist(start, end, dist);
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
    int nowTime, map_of_pairs& dist) {
    for (auto it = vehicles.begin(); it != vehicles.end(); it++) {
        it->update(nowTime, requests, dist);
    }
}

void finish_all(vector<Vehicle>& vehicles, vector<Request>& unserved, map_of_pairs& dist) {
        
    int idx = 0;
    for (auto it = vehicles.begin(); it != vehicles.end(); it++) {
        // printf("V #%d: ", idx++);
        it->finish_route(dist);
    }
    for (auto iter = unserved.begin(); iter != unserved.end(); iter++) {
        unserved_dist += iter->shortestDist;
    }
}

void setupOutfiles(const std::string& outDir, const std::string& outFilename){
    std::filesystem::create_directories(outDir);
    std::filesystem::create_directories(outDir + "GurobiLogs/");
    std::filesystem::create_directories(outDir + "Code/");
	//std::filesystem::copy("/ocean/projects/eng200002p/mbruchon/Pooling", outDir + "Code/");
	system(("cp -p /ocean/projects/eng200002p/mbruchon/RidePooling/*.* " + outDir + "Code/").c_str());
    std::ofstream ofs;
    ofs.open(outDir + outFilename, std::ofstream::out | std::ofstream::app);
    ofs << "NowTime,CPUTime,TotalReqs,ServedReqs,TotalDist,UnservedDist,RawDist,WaitTime,Pooled1,Pooled2,Pooled3,Pooled4,DisconnectedCars\n";
    ofs.close();
}

void print_stats(const std::string& outDir, const std::string& outFilename) {
 /*   FILE *out = fopen(outFile, "w");
    fprintf(out, "Service rate: %d / %d = %f\n",
        served_reqs, total_reqs, (double(served_reqs)) / total_reqs);
    fprintf(out, "Dratio = %f\n", double(total_dist + unserved_dist) / raw_dist);
    fprintf(out, "Eratio = %f\n", double(total_dist) / (raw_dist - unserved_dist));
    fprintf(out, "Total waiting time = %d\n", total_wait_time);
    fclose(out);
*/
    std::ofstream ofs;
    ofs.open(outDir + outFilename, std::ofstream::out | std::ofstream::app);
    ofs << to_string(now_time) + "," + to_string(clock()) + "," +
        to_string(these_reqs) + "," + to_string(these_served_reqs) + "," + 
        to_string(total_dist) + "," + to_string(unserved_dist) + "," +
        to_string(raw_dist) + "," + to_string(total_wait_time) + "," + 
        to_string(utilization[1]) + "," + to_string(utilization[2]) + "," + to_string(utilization[3]) + "," + to_string(utilization[4]) + "," +
        to_string(disconnectedCars) +
        "\n";
    ofs.close();
}
void print_line(const std::string& outDir, const std::string& outFilename, const std::string& message) {
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-startTime;
    std::ofstream ofs;
    ofs.open(outDir + outFilename, std::ofstream::out | std::ofstream::app);
    ofs << std::to_string(elapsed_seconds.count()) + ": " + message + "\n";
    ofs.close();
}

void log_stats() {
    printf("Service rate: %d / %d = %f\n",
        these_served_reqs, these_reqs, (double(these_served_reqs)) / these_reqs);
 //   printf("Dratio = %f\n", double(total_dist + unserved_dist) / raw_dist);
 //   printf("Eratio = %f\n", double(total_dist) / (raw_dist - unserved_dist));
 //   printf("Total waiting time = %d\n", total_wait_time);
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

costComponents::costComponents()
{
    distance = 0;
    time = 0;
    emissions_nox = 0;
    emissions_sox = 0;
    emissions_nh3 = 0;
    emissions_pm = 0;
    emissions_ghg = 0;
}

bool costComponents::operator<(const costComponents& other) const
{
    int cost1 = getTotal();
    int cost2 = other.getTotal();
    return cost1 < cost2 ? true : false;
}

double costComponents::getTotal() const {
    return distance;
}