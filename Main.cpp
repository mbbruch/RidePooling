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
    vector<int> order;
    std::pair<int,int> result1 = treeCost.get_dist(2617, 2684); //should be 290070 distance
    pair<int, int> result2 = treeCost.search_cache(2617 - 1, 2684 - 1);
    pair<int, int> fpResult1 = treeCost.find_path(2617 - 1, 2684 - 1, order);
    pair<int,int> result3 = treeCost.get_dist(8, 7184); //should be 43.16695064 12560
    pair<int, int> result4 = treeCost.search_cache(8 - 1, 7184 - 1);
    pair<int, int> fpResult2 = treeCost.find_path(8 - 1, 7184 - 1, order);
    result3 = treeCost.get_dist(1350, 1290); //should be 43.16695064 12560; 12600948
    result4 = treeCost.search_cache(1350 - 1, 1290 - 1);
    fpResult2 = treeCost.find_path(1350 - 1, 1290 - 1, order);
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

        if (read_requests(in, requests, now_time + time_step)) {
            //Note: the read_requests code reads one request beyond now_time+time_step (unless it returns false)
            tail = requests.back();
            requests.pop_back();
            hasMore = true;
        }
        else {
            hasMore = false;
        }

        update_vehicles(vehicles, requests, now_time);

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

		beforeTime = std::chrono::system_clock::now();

        RTV->solve(env, vehicles, requests, unserved);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Solving time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
        RTV->rebalance(env, vehicles, unserved);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Rebalancing time = %f", elapsed_seconds.count()));
		
        delete RV;
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
