#pragma once
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <cassert>
#include <cstdint>
#include <chrono>
using namespace std;

extern int now_time;
extern int total_reqs, served_reqs, these_reqs, these_served_reqs;
extern long long total_dist, unserved_dist, raw_dist;
extern int total_wait_time;
extern map<int, int> utilization;
extern int disconnectedCars;

extern double travel_time;
extern int travel_cnt;
extern double travel_max;

extern int max_vehicle;
extern int dist_map_size;

static const int max_capacity = 2;
static const int max_trip_size = 8;
static const std::chrono::time_point startTime = std::chrono::system_clock::now();

extern const int time_step;
extern const int max_node;
extern const int max_wait_sec;
extern const int max_delay_sec;
extern const int velocity; // dm/s

extern const int penalty;
extern const int max_v_per_req;
extern const int min_req_per_v;

extern const double minimal;
static const int INF = 0x3fffffff;

extern std::string outDir; 
extern std::string logFile;

typedef std::set<int> uos;

struct sum_of_hashes
{
public:
    std::size_t operator()(const size_t& Left, int Right) const
    {
        return (Left + std::hash<int>{}(Right));
    }
};

inline void hash_combine(std::size_t& seed, int const& v)
{
    seed ^= std::hash<int>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

class MyHash
{
public:
    std::size_t operator()(const uos& s) const
    {
        // return std::accumulate(s.begin(), s.end(), std::hash<int>{}(0), sum_of_hashes());
        // return std::hash<int>{}(std::accumulate(s.begin(), s.end(), 0));
        size_t seed = 0;
        for (auto it = s.begin(); it != s.end(); it++)
            //Combine the hash of the current vector with the hashes of the previous ones
            hash_combine(seed, *it);
        return seed;
    };

};

class PairHash
{
public:
    inline size_t operator()(const std::pair<int, int>& p)const {
        //assert(sizeof(std::size_t) >= 8);
        return hash<long long>()(((long long)p.first) ^ (((long long)p.second) << 32));
        //return (((uint64_t)p.first) << 32) | ((uint64_t)p.second);
        //return p.first * 31 + p.second;
    };
};

//typedef std::unordered_set<std::unordered_set<int>, MyHash> set_of_uos;
typedef std::unordered_map<uos, pair<int, uos>, MyHash  > map_of_uos;
typedef std::unordered_map<pair<int, int>, int, PairHash> map_of_pairs;