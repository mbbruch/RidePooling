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

    std::string reqFile = "/ocean/projects/eng200002p/mbruchon/RidePooling/In/requests_chicago.csv";//argv[1];
    std::string vehFile = "/ocean/projects/eng200002p/mbruchon/RidePooling/In/vehicles_chicago.csv";//argv[2];
    std::string filenameTime = GetCurrentTimeForFileName();
    outDir = "/ocean/projects/eng200002p/mbruchon/RidePooling/Out/" + filenameTime + "/";
    std::string outFilename = "main_" + filenameTime + ".csv";//argv[3];
   logFile = "logfile_" + filenameTime + ".txt";//argv[3];
    setupOutfiles(outDir, outFilename);
    //max_capacity = atoi(argv[4]);
    map_of_pairs dist;
	
	print_line(outDir,logFile,"Initializing GRBEnv");
	GRBEnv *env;
	try{
		env = new GRBEnv();
	}
		catch (GRBException& e) {
		print_line(outDir, logFile,string_format("Gurobi exception code: %d.",e.getErrorCode()));
		print_line(outDir, logFile,"Gurobi exception message: "+e.getMessage());
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
	print_line(outDir,logFile,"Start initializing");
    initialize(false, dist);
    print_line(outDir,logFile,"load_end");

    vector<Vehicle> vehicles;
    vehicles.reserve(max_vehicle);
    read_vehicles(vehFile.c_str(), vehicles);
    print_line(outDir,logFile,"Vehicles read in");

    vector<Request> requests;
    requests.reserve(100);

    bool hasMore = false;
    Request tail;

    vector<Request> unserved;

    FILE *in = get_requests_file(reqFile.c_str());
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
        print_line(outDir,logFile,string_format("Timestep: %d", now_time));

        requests.clear();
        if (hasMore) {
            requests.push_back(tail);
        }

        handle_unserved(unserved, requests, now_time);

        if (read_requests(in, requests, now_time + time_step, dist)) {
            //Note: the read_requests code reads one request beyond now_time+time_step (unless it returns false)
            tail = requests.back();
            requests.pop_back();
            hasMore = true;
        }
        else {
            hasMore = false;
        }

        update_vehicles(vehicles, requests, now_time, dist);

		std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Preprocessing time = %f", elapsed_seconds.count()));

        //one-to-one: vehicle locations + ongoing request locations
        //all-to-all 1: vehicle locations + new request locations (start points only)
        //all-to-all 2: ongoing request locations + new requestion locations
		
		beforeTime = std::chrono::system_clock::now();
        std::set<std::pair<int, int>> ongoingLocs;
        for (int i = 0; i < vehicles.size(); i++) {
            int vehLoc = vehicles[i].get_location();
            for (int j = 0; j < vehicles[i].get_num_passengers(); j++) {
                int dropoff = vehicles[i].passengers[j].end;
                //Vehicle to on-board passenger dropoff
                if (vehLoc < dropoff){
                    ongoingLocs.insert(make_pair(vehLoc, dropoff));
                }
                else if (vehLoc > dropoff) {
                    ongoingLocs.insert(make_pair(dropoff, vehLoc));
                }
                for (int k = j+1; k < vehicles[i].get_num_passengers(); k++) {
                    int newdropoff = vehicles[i].passengers[k].end;
                    //On-board to on-board passenger dropoff
                    if (newdropoff < dropoff) {
                        ongoingLocs.insert(make_pair(newdropoff, dropoff));
                    }
                    else if (newdropoff > dropoff) {
                        ongoingLocs.insert(make_pair(dropoff, newdropoff));
                    }
                }

                for (int k = 0; k < requests.size(); k++) {
                    int newpickup = requests[k].start;
                    int newdropoff = requests[k].end;
                    //On-board passenger dropoff to new request pickup
                    if (newpickup < dropoff) {
                        ongoingLocs.insert(make_pair(newpickup, dropoff));
					}
                    else if (newpickup > dropoff) {
                        ongoingLocs.insert(make_pair(dropoff, newpickup));
					}
                    //On-board passenger dropoff to new request dropoff
                    if (newdropoff < dropoff) {
                        ongoingLocs.insert(make_pair(newdropoff, dropoff));
                    }
                    else if (newdropoff > dropoff) {
                        ongoingLocs.insert(make_pair(dropoff, newdropoff));
                    }
                }
            }
            //Vehicle location to new request pickup
            for (int j = 0; j < requests.size(); j++) {
                int newPickup = requests[j].start;
                if (vehLoc < newPickup) {
                    ongoingLocs.insert(make_pair(vehLoc, newPickup));
                }
                else if (vehLoc > newPickup) {
                    ongoingLocs.insert(make_pair(newPickup, vehLoc));
                }
            }
        }
        std::set<int> allToAll;
        for (int i = 0; i < requests.size(); i++) {
            allToAll.insert({ requests[i].start, requests[i].end });
        }
        map_of_pairs dist{ ongoingLocs.size() +
            (allToAll.size() * (allToAll.size() - 1) / 2) };
			
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Dist map combo set up time = %f", elapsed_seconds.count()));
		beforeTime = std::chrono::system_clock::now();		
        reinitialize_dist_map(ongoingLocs, allToAll, dist);
		elapsed_seconds = std::chrono::system_clock::now()-beforeTime;
        print_line(outDir,logFile,string_format("Dist map set up time = %f", elapsed_seconds.count()));

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
