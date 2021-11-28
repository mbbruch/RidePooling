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

    env->set(GRB_IntParam_OutputFlag, 0);
    //env->set(GRB_IntParam_Threads, 4);
    now_time = -time_step;
    total_reqs = served_reqs = these_reqs = these_served_reqs = 0;
    total_dist = unserved_dist = raw_dist = 0;
    total_wait_time = 0;
    disconnectedCars = 0;

    //  while ((getchar()) != '\n');
    print_line(outDir, logFile, "Start initializing");
    treeCost.EdgeWeightsFile = costFile;
    treeCost.initialize(false);
	cout << " tree initialized" << endl;

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

        handle_unserved(unserved, requests, now_time);

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

        print_line(outDir, logFile, "Updating vehicles");
        update_vehicles(vehicles, requests, now_time);

        std::chrono::duration<double> asdf = std::chrono::system_clock::now() - std::chrono::system_clock::now();
        std::string temp = string_format("Rebalancing time = %f", asdf.count());

		std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Preprocessing time = %f", elapsed_seconds.count()));
		beforeTime = std::chrono::system_clock::now();
        RVGraph *RV = new RVGraph(vehicles, requests);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("RV build time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
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
        for (auto it = utilization.begin(); it != utilization.end(); it++) {
            if (it->first > 0) {
                print_line(outDir,logFile,string_format("Vehicles serving %d passengers: %d.", it->first, it->second));
            }
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
