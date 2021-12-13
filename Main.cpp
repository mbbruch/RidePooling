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
#include <fstream>
#include <iomanip>
#include "gurobi_c++.h"
#include "hps_src/hps.h"
#include "globals.h"
#include "RTV.h"
#include "util.h"
#include "GPtree.h"

using namespace std;


int main(int argc, char* argv[]) {
    std::string filenameTime = GetCurrentTimeForFileName();
    outDir = baseOutDir + filenameTime + "/";
    std::string outFilename = "main_" + filenameTime + ".csv";//argv[3];
    logFile = "logfile_" + filenameTime + ".txt";//argv[3];
    setupOutfiles(outDir, outFilename);
    //max_capacity = atoi(argv[4]);

    print_line(outDir, logFile, "Initializing GRBEnv");
    GRBEnv* env;
    try {
        env = new GRBEnv();
    }
    catch (GRBException& e) {
        print_line(outDir, logFile, string_format("Gurobi exception code: %d.", e.getErrorCode()));
        print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
    }

    env->set(GRB_IntParam_OutputFlag, 1);
    env->set(GRB_IntParam_LogToConsole, 0);
    //env->set(GRB_IntParam_Threads, 4);
    now_time = -time_step;
    total_reqs = served_reqs = these_reqs = these_served_reqs = 0;
    total_dist = unserved_dist = raw_dist = 0;
    total_wait_time = 0;
    disconnectedCars = 0;

	print_ram(outDir,string_format("%d_start", now_time));

    //  while ((getchar()) != '\n');
    print_line(outDir, logFile, "Start initializing");
    treeCost.EdgeWeightsFile = costFile;
    treeCost.initialize(false);
    print_line(outDir, logFile, "Initialized");
	cout << " tree initialized" << endl;
	print_ram(outDir,string_format("%d_initialized", now_time));

    print_line(outDir, logFile, "load_end");
    vector<Vehicle> vehicles;
    vehicles.reserve(max_vehicle);
    read_vehicles(vehFile.c_str(), vehicles);
    print_line(outDir, logFile, "Vehicles read in");

    bool hasMore = false;
    Request tail;
    vector<Request> requests;
    vector<Request> unserved;
    FILE* in = get_requests_file(reqFile.c_str());
    while (true) {
        auto beforeTime = std::chrono::system_clock::now();
        utilization.clear();
        travel_time = 0;
        travel_max = 0;
        travel_cnt = 0;
        these_reqs = 0;
        now_time += time_step;
        print_line(outDir, logFile, string_format("Timestep: %d", now_time));

        requests.clear();
        if (hasMore) {
            requests.push_back(tail);
        }
        map<int, int> reqCounter;
        // insert new requests: s->t into src_dst
        for (int i = 0; i < unserved.size(); i++) {
            reqCounter[unserved[i].unique]++;
        }
        for (auto it = reqCounter.begin(); it != reqCounter.end(); it++) {
            if (it->second > 1) {
                int x = 5;
            }
        }
        print_line(outDir, logFile, string_format("Unserved: %d", unserved.size()));
        handle_unserved(unserved, requests, now_time);
        reqCounter.clear();
        for (int i = 0; i < requests.size(); i++) {
            reqCounter[requests[i].unique]++;
        }
        for (auto it = reqCounter.begin(); it != reqCounter.end(); it++) {
            if (it->second > 1) {
                int x = 5;
            }
        }

        print_line(outDir, logFile, "Reading requests");
        if (read_requests(in, requests, now_time + time_step)) {
            //Note: the read_requests code reads one request beyond now_time+time_step (unless it returns false)
            tail = requests.back();
            requests.pop_back();
            hasMore = true;
        }
        else {
            hasMore = false;
        }
        reqCounter.clear();
        for (int i = 0; i < requests.size(); i++) {
            reqCounter[requests[i].unique]++;
        }
        for (auto it = reqCounter.begin(); it != reqCounter.end(); it++) {
            if (it->second > 1) {
                int x = 5;
            }
        }

        print_line(outDir, logFile, "Updating vehicles");
        vector<Request> reqCopy = requests;
        map<int, vector<int>> indices;
        for (int i = 0; i < vehicles.size(); i++) {
            for (int j = 0; j < vehicles[i].get_num_passengers(); j++) {
                indices[vehicles[i].passengers[j].unique].push_back(i);
            }
        }
        for (int i = 0; i < requests.size(); i++) {
            indices[requests[i].unique].push_back(1000000 + i);
        }
        for (auto it = indices.begin(); it != indices.end(); it++) {
            if (it->second.size() > 1) {
                bool bDupeFound = true;
            }
        }

        
        update_vehicles(vehicles, requests, now_time);
        reqCounter.clear();
        for (int i = 0; i < requests.size(); i++) {
            reqCounter[requests[i].unique]++;
        }
        for (auto it = reqCounter.begin(); it != reqCounter.end(); it++) {
            if (it->second > 1) {
                int x = 5;
            }
        }
        std::chrono::duration<double> asdf = std::chrono::system_clock::now() - std::chrono::system_clock::now();
        std::string temp = string_format("Rebalancing time = %f", asdf.count());

		std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Preprocessing time = %f", elapsed_seconds.count()));
		beforeTime = std::chrono::system_clock::now();
        RVGraph *RV = new RVGraph(vehicles, requests);

        reqCounter.clear();
        for (int i = 0; i < requests.size(); i++) {
            reqCounter[requests[i].unique]++;
        }
        for (auto it = reqCounter.begin(); it != reqCounter.end(); it++) {
            if (it->second > 1) {
                int x = 5;
            }
        }
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("RV build time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
		print_ram(outDir,string_format("%d_before_rtv_build", now_time));
        RTVGraph *RTV = new RTVGraph(RV, vehicles, requests);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("RTV build time = %f", elapsed_seconds.count()));

        delete RV;
		beforeTime = std::chrono::system_clock::now();

        RTV->solve(env, vehicles, requests, unserved);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Solving time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
        RTV->rebalance(env, vehicles, unserved);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Rebalancing time = %f", elapsed_seconds.count()));
		
        delete RTV;

        clock_t tock = clock();
//        printf("travel / total = %f / %f\n", travel_time, (double(tock - tick)) / CLOCKS_PER_SEC);

        log_stats();
        //write_vehicle_routes(outDir, vehicles, now_time);
        print_stats(outDir, outFilename);
        vector<int> carUtilization(max_trip_size + max_capacity, 0);
        for (auto it = vehicles.begin(); it != vehicles.end(); it++) {
            int psgrs = it->get_num_passengers();
            if (carUtilization.size() > psgrs && psgrs > 0) {
                carUtilization[psgrs - 1]++;
            }
        }
        for (int i = 0; i < carUtilization.size(); i++) {
            if (carUtilization[i] == 0) continue;
            print_line(outDir, logFile, string_format("Vehicles serving %d passengers: %d.", i+1, carUtilization[i]));
        }
        if (!hasMore) {
            break;
        }
    }
    finish_all(vehicles, unserved);

    print_stats(outDir, outFilename);

    fclose(in);

    delete env;

    return 0;
}
