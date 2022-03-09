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
    print_line(outDir, logFile, "Initializing GRBEnv");
    GRBEnv* env;
    try {
        env = new GRBEnv();
		env->set(GRB_IntParam_OutputFlag, 1);
		env->set(GRB_IntParam_LogToConsole, 0);
    }
    catch (GRBException& e) {
        print_line(outDir, logFile, string_format("Gurobi exception code: %d.", e.getErrorCode()));
        print_line(outDir, logFile, "Gurobi exception message: " + e.getMessage());
    }
	//int value1 = omp_get_max_threads();
	//print_line(outDir, logFile, string_format("Default max threads: %d.", value1));
	//omp_set_num_threads(1);
	//print_line(outDir, logFile, string_format("New max threads: %d.", omp_get_max_threads()));
	//omp_set_num_threads(value1);
	//print_line(outDir, logFile, string_format("Fixed max threads: %d.", omp_get_max_threads()));
    //env->set(GRB_IntParam_Threads, 4);
	time_step = 300;
    now_time = -time_step; //250200 -time_step;
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
	
	std::vector<Request> temp;
    RTVGraph::rebalance_for_demand_forecasts(env, vehicles, temp, 0);

    bool hasMore = false;
    Request tail; 
    vector<Request> requests;
    vector<Request> unserved;

    std::vector<Vehicle> beforesolveVeh;
    std::vector<Request> beforesolveReq;
    std::vector<Request> beforesolveUns;
    std::vector<Vehicle> beforerebalance;
	std::string inFile = baseInDir + "requests.csv";
//	FILE* in2 = get_requests_file(inFile.c_str());    
//	update_requests_with_dist(in2);
//    fclose(in2);
    FILE* in = get_requests_file(reqFile.c_str());    
	vector<int> order;
	int thisCost1 = treeCost.get_dist(1, 2).first;
	int thisCost2 = treeCost.search_cache(0,1).first;
	int thisCost3 = treeCost.find_path(0,1,order).first;
	int thisDist1 = treeCost.get_dist(1,2).second;
	int thisDist2 = treeCost.search_cache(0,1).second;
	int thisDist3 = treeCost.find_path(0,1,order).second;
	print_line(outDir, logFile, string_format("%d to %d: costs %d,%d,%d; dists %d,%d,%d.", 1,2,thisCost1,thisCost2,thisCost3,thisDist1,thisDist2,thisDist3));


	thisCost1 = treeCost.get_dist(500, 505).first;
	thisCost2 = treeCost.search_cache(499,504).first;
	thisCost3 = treeCost.find_path(499,504,order).first;
	thisDist1 = treeCost.get_dist(500,505).second;
	thisDist2 = treeCost.search_cache(499,504).second;
	thisDist3 = treeCost.find_path(499,504,order).second;
	print_line(outDir, logFile, string_format("%d to %d: costs %d,%d,%d; dists %d,%d,%d.", 500,505,thisCost1,thisCost2,thisCost3,thisDist1,thisDist2,thisDist3));
	
	thisCost1 = treeCost.get_dist(68, 419).first;
	thisCost2 = treeCost.search_cache(67,418).first;
	thisCost3 = treeCost.find_path(67,418,order).first;
	thisDist1 = treeCost.get_dist(68,419).second;
	thisDist2 = treeCost.search_cache(67,418).second;
	thisDist3 = treeCost.find_path(67,418,order).second;
	print_line(outDir, logFile, string_format("%d to %d: costs %d,%d,%d; dists %d,%d,%d.", 68,419,thisCost1,thisCost2,thisCost3,thisDist1,thisDist2,thisDist3));
    while (true) {
		time_step = 300;
        auto beforeTime = std::chrono::system_clock::now();
        travel_time = 0;
        travel_max = 0;
        travel_cnt = 0;
        these_reqs = 0;
        now_time += time_step;
        print_line(outDir, logFile, string_format("now_time: %d, time_step: %d, default_time_step: %d.", now_time, time_step, default_time_step));

        requests.clear();
        if (hasMore) {
            requests.push_back(tail);
        }

        handle_unserved(unserved, requests, now_time);

        print_line(outDir, logFile, string_format("Reading requests from %d to %d, on top of %d unserved and %d requests.", now_time, now_time+default_time_step, unserved.size(), requests.size()));
        if (read_requests(in, requests, now_time + default_time_step)) {
            //Note: the read_requests code reads one request beyond now_time+time_step (unless it returns false)
            tail = requests.back();
            requests.pop_back();
            hasMore = true;
        }
        else {
            hasMore = false;
        }
		//if (requests.size() > 0) time_step = requests[requests.size()-1].reqTime - now_time;

        print_line(outDir, logFile, string_format("For %d total requests (%d unserved), updating %d vehicles.", requests.size(), unserved.size(), vehicles.size()));
        update_vehicles(vehicles, requests, now_time);
        print_line(outDir, logFile, string_format("After vehicle update, %d requests (%d unserved).", requests.size(), unserved.size()));

		std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("TOtal preprocessing time = %f", elapsed_seconds.count()));
		beforeTime = std::chrono::system_clock::now();
        RVGraph *RV = new RVGraph(vehicles, requests);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("RV build time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
		print_ram(outDir,string_format("%d_before_rtv_build", now_time));
        RTVGraph *RTV = new RTVGraph(RV, vehicles, requests);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("RTV build time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();

        RTV->solve(env, vehicles, requests, unserved);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
	print_ram(outDir,string_format("%d_before_solve", now_time));
        print_line(outDir,logFile,string_format("Solving time = %f", elapsed_seconds.count()));

		beforeTime = std::chrono::system_clock::now();
        RTV->rebalance(env, vehicles, unserved);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
	print_ram(outDir,string_format("%d_before_rebalance", now_time));

        print_line(outDir,logFile,string_format("Rebalancing time = %f", elapsed_seconds.count()));
	print_ram(outDir,string_format("%d_after_rebalance", now_time));

		
        delete RV;
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
            if (carUtilization[i] > 0) print_line(outDir, logFile, string_format("Vehicles serving %d passengers: %d.", i+1, carUtilization[i]));
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
