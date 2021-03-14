#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <map>
#include "util.h"
#include <set>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <omp.h>
#include <cstring>
#include "gurobi_c++.h"

#include "globals.h"
#include "RTV.h"
#include "util.h"
#include "GPtree.h"

using namespace std;


int main(int argc, char* argv[]) {

    char* reqFile = "C:/Code_Projects/RidePooling/requests.csv";//argv[1];
    char* vehFile = "C:/Code_Projects/RidePooling/vehicles.csv";//argv[2];
    std::string filenameTime = GetCurrentTimeForFileName();
    outDir = "C:/Code_Projects/RidePooling/" + filenameTime + "/";
    std::string outFilename = "main_" + filenameTime + ".csv";//argv[3];
   logFile = "logfile_" + filenameTime + ".txt";//argv[3];
    setupOutfiles(outDir, outFilename);
    //max_capacity = atoi(argv[4]);
    map_of_pairs dist;

	print_line(outDir,logFile,"Initializing GRBEnv");
    GRBEnv *env = new GRBEnv();
    env->set(GRB_IntParam_OutputFlag, 0);
    env->set(GRB_IntParam_Threads, 4);
    now_time = -time_step;
    total_reqs = served_reqs = these_reqs = these_served_reqs = 0;
    total_dist = unserved_dist = raw_dist = 0;
    total_wait_time = 0;
    dist_map_size = 0;
    disconnectedCars = 0;
	
	print_line(outDir,logFile,"Start initializing");
    initialize(false, dist);
    print_line(outDir,logFile,"load_end");

    vector<Vehicle> vehicles;
    vehicles.reserve(max_vehicle);
    read_vehicles(vehFile, vehicles);

    vector<Request> requests;
    requests.reserve(100);

    bool hasMore = false;
    Request tail;

    vector<Request> unserved;

    FILE *in = get_requests_file(reqFile);
    fclose(in);
    in = get_requests_file(reqFile);

    while (true) {
		auto beforeTime = std::chrono::system_clock::now();
        utilization.clear();
        travel_time = 0;
        travel_max = 0;
        travel_cnt = 0;
        these_reqs = 0;
        now_time += time_step;
        print_line(outDir,logFile,string_format("Timestep: %d", now_time));

        requests.clear();
        if (hasMore) {
            requests.push_back(tail);
        }

        handle_unserved(unserved, requests, now_time);

        if (read_requests(in, requests, now_time+time_step, dist)) { 
            //Note: the read_requests code reads one request beyond now_time+time_step (unless it returns false)
            tail = requests.back();
            requests.pop_back();
            hasMore = true;
        } else {
            hasMore = false;
        }

        update_vehicles(vehicles, requests, now_time, dist);

		std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Preprocessing time = %f", elapsed_seconds.count()));

        //one-to-one: vehicle locations + ongoing request locations
        //all-to-all 1: vehicle locations + new request locations (start points only)
        //all-to-all 2: ongoing request locations + new requestion locations
        std::set<std::pair<int, int>> ongoingLocs;
        std::set<int> allToAll1;
        std::set<int> allToAll2;
        map_of_pairs dist;
        for (int i = 0; i < vehicles.size(); i++) {
            const int vehLoc = vehicles[i].get_location();
            allToAll1.emplace(vehLoc);
            for (int j = 0; j < vehicles[i].get_num_passengers(); j++) {
                ongoingLocs.emplace(std::pair{ vehLoc, vehicles[i].passengers[j].start });
                ongoingLocs.emplace(std::pair{ vehLoc, vehicles[i].passengers[j].end });
                ongoingLocs.emplace(std::pair{ vehicles[i].passengers[j].start, vehicles[i].passengers[j].end });
                allToAll2.emplace(vehicles[i].passengers[j].start);
                allToAll2.emplace(vehicles[i].passengers[j].end);
            }
        }
        for (int i = 0; i < requests.size(); i++) {
            allToAll1.emplace(requests[i].start);
            allToAll2.emplace(requests[i].start);
            allToAll2.emplace(requests[i].end);
        }
        dist = reinitialize_dist_map(ongoingLocs, allToAll1, allToAll2);

        print_line(outDir, logFile, string_format("Dist map size = %f", dist.size()));
		beforeTime = std::chrono::system_clock::now();
        RVGraph *RV = new RVGraph(vehicles, requests, dist);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("RV build time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
        RTVGraph *RTV = new RTVGraph(RV, vehicles, requests, dist);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("RTV build time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
        RTV->solve(env, vehicles, requests, unserved, dist);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Solving time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
        RTV->rebalance(env, vehicles, unserved, dist);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Rebalancing time = %f", elapsed_seconds.count()));
		
        delete RV;
        delete RTV;

        clock_t tock = clock();
//        printf("travel / total = %f / %f\n", travel_time, (double(tock - tick)) / CLOCKS_PER_SEC);

        log_stats();
        print_stats(outDir, outFilename);
        for (auto it = utilization.begin(); it != utilization.end(); it++) {
            if (it->first > 0) {
                print_line(outDir,logFile,string_format("Vehicles serving %d passengers: %d.", it->first, it->second));
            }
        }
        if (!hasMore) {
            break;
        }
    }
    finish_all(vehicles, unserved, dist);

    print_stats(outDir, outFilename);

    fclose(in);

    delete env;

    return 0;
}