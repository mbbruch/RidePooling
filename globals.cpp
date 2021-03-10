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
int dist_map_size;

std::string outDir;
std::string logFile;

extern const int time_step = 60*5;
extern const int max_node = 27087; //264346;
extern const int max_wait_sec = 60*15;
extern const int max_delay_sec = 60 * 15; // max_wait_sec;
extern const int velocity = 84;// 84; //84; // dm/s or 19 mph

extern const int penalty = max_delay_sec*1000;
extern const int max_v_per_req = 50;
extern const int min_req_per_v = 50;

extern const double minimal = 1e-4;
