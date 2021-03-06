#include "globals.h"
#include <map>

int now_time;
int total_reqs, served_reqs, these_reqs, these_served_reqs;
long long total_dist, unserved_dist, raw_dist;
int total_wait_time;
map<int, int> utilization;
int disconnectedCars;

double travel_time;
int travel_cnt;
double travel_max;

int max_vehicle;
int vehicle_depot;

std::string baseOutDir = "/ocean/projects/eng200002p/mbruchon/RidePooling/Out/"; //"/ocean/projects/ddm180004p/mbruchon/Out/"; //"/ocean/projects/eng200002p/shared/Out/";// baseDir + "Out/";
std::string outDir;
std::string logFile;

extern const int max_wait_sec = 60*15;
extern const int max_delay_sec = 60 * 15; // max_wait_sec;
extern const int velocity = 67; // 84 dm/s = 19 mph

extern const int penalty = max_delay_sec*1000;
extern const int max_v_per_req = 15;
extern const int min_req_per_v = 15;
extern const int cost_scale_factor = 20; //NOTE: this must be set to ONE when building dist_map the first time; must be larger otherwise

extern const double pickupPenalty = 123.6025; //cents per pickup
extern const double delayPenalty = 33.70264/60; //cents per second

extern const double minimal = 1e-4;