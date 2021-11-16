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
    std::string reqFile = "C:/Code_Projects/RidePooling/In/requests.csv";//argv[1];
    std::string vehFile = "C:/Code_Projects/RidePooling/In/vehicles.csv";//argv[2];
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
    dist_map_size = 0;
    disconnectedCars = 0;

    //  while ((getchar()) != '\n');
    print_line(outDir, logFile, "Start initializing");
    treeCost.EdgeWeightsFile = Cost_File;
    treeCost.initialize(false);
    vector<int> order1, order2;
    std::pair<int, int> result1, result2;

    result1 = treeCost.find_path(1 - 1, 5 - 1, order1);
    print_line(outDir, logFile, "load_end");
    vector<Vehicle> vehicles;
    vehicles.reserve(max_vehicle);
    read_vehicles(vehFile.c_str(), vehicles);
    print_line(outDir, logFile, "Vehicles read in");

    vector<int> results1;
    results1.reserve(500 * 500);
    int i = 0;
    int j = 0;
    std::vector<int> out;
    treeCost.G.dijkstra(101 - 1, out);
 //   #pragma omp parallel for default(none) private(i) shared(treeCost)
 //   for (i = 1; i < 500; i++) {
   //     out = treeCost.G.find_path(76 - 1, 143 - 1);
    //}
    int yes = 5;
//#pragma omp parallel for default(none) private(i) shared(treeCost)
//    for (i = 1; i < 500; i++) {
  //      result1 = treeCost.find_path(76 - 1, 143 - 1, order1);
    //}
    int correct = 0;
    int incorrect = 0;
    out = treeCost.G.find_path(101 - 1, 455 - 1);
    result1 = treeCost.get_dist(101 - 1, 455 - 1);

 //   #pragma omp parallel for default(none) private(i,j) shared(treeCost)
    for (i = 1; i < 500; i++) {
        for (j = 1; j < 500; j++) {
            result1 = treeCost.find_path(i - 1, j - 1, order1);
            result2 = treeCost.get_dist(i, j);
            if (result1 != result2) {
                incorrect++;
                if (result1.first < result2.first) {
                    bool suboptimal = false;
                }
            }
            else {
                correct++;
            }
        }
    }

    int x = correct; int y = incorrect;
    vector<int> results2;
    results2.reserve(500 * 500);
//#pragma omp parallel for default(none) private(i,j) shared(treeCost)
    for (i = 1; i < 500; i++) {
        for (j = 1; j < 500; j++) {
        }
    }

    for (int i = 0; i < results1.size(); i++) {
        if (results1[i] != results2[i]) {
            int x = 5;
        }
}
    vector<Request> requests;
    requests.reserve(100);

    bool hasMore = false;
    Request tail;

    vector<Request> unserved;

    FILE* in = get_requests_file(reqFile.c_str());
    fclose(in);
    in = get_requests_file(reqFile.c_str());

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

        //one-to-one: vehicle locations + ongoing request locations
        //all-to-all 1: vehicle locations + new request locations (start points only)
        //all-to-all 2: ongoing request locations + new requestion locations
		
		beforeTime = std::chrono::system_clock::now();
        std::set<std::pair<int, int>> ongoingLocs;
        std::set<int> allToAll1;
        std::set<int> allToAll2;
        for (int i = 0; i < vehicles.size(); i++) {
            const int vehLoc = vehicles[i].get_location();
            allToAll1.emplace(vehLoc);
            for (int j = 0; j < vehicles[i].get_num_passengers(); j++) {
                ongoingLocs.insert({
                    std::pair{vehLoc, vehicles[i].passengers[j].start },
                    std::pair{ vehLoc, vehicles[i].passengers[j].end },
                    std::pair{ vehicles[i].passengers[j].start, vehicles[i].passengers[j].end } });
                allToAll2.insert({ vehicles[i].passengers[j].start, vehicles[i].passengers[j].end});
                }
                }
        for (int i = 0; i < requests.size(); i++) {
            allToAll1.emplace(requests[i].start);
            allToAll2.insert({ requests[i].start, requests[i].end });
        }
        pairs_to_pairs dist{ 2*(ongoingLocs.size() +
            (allToAll1.size() * (allToAll1.size() - 1) / 2) +
            (allToAll2.size() * (allToAll2.size() - 1) / 2)) };

		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Dist map combo set up time = %f", elapsed_seconds.count()));
		beforeTime = std::chrono::system_clock::now();
        vector<int> vec1{ allToAll1.begin(), allToAll1.end() };
        set<int>().swap(allToAll1);
        vector<int> vec2{ allToAll2.begin(), allToAll2.end() };
        set<int>().swap(allToAll2);
        treeCost.reinitialize_dist_map(ongoingLocs, vec1, vec2);
        vector<int>().swap(vec1);
        vector<int>().swap(vec2);
        set<pair<int, int>>().swap(ongoingLocs);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Dist map set up time = %f", elapsed_seconds.count()));

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
