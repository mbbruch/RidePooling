#pragma once
#include <map>
#include <set>
#include <queue>
#include <deque>
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <boost/container/flat_set.hpp>
#include <utility>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <chrono>
#include <random>
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
extern int vehicle_depot;

static int time_step = 300;
static const int default_time_step = 300;
static const int max_capacity = 2;
static const int max_trip_size = 8;
static const int fleet_size = 25000;
static const double cars_needed_per_trip_per_30 = 2;
static const int req_per_window = 1000;
static const std::chrono::time_point startTime = std::chrono::system_clock::now();
static std::chrono::time_point lastTimeCheck = std::chrono::system_clock::now();
static const std::string baseDir = "C:/Code_Projects/RidePooling/";
static const std::string city = "austin";
static const std::string costs = "private";
static const std::string baseInDir = baseDir + "In/";
static const std::string distFile = baseInDir + city + ".dist"; // In the first line, two integers n, m represent the number of points and edges, 
                                    //and in the next m lines, three integers U, V, C represent U->V has an edge of length C
static const std::string costFile = baseInDir + city + "_" + costs + ".cost";// In the first line, two integers n, m represent the number of points and edges, 
                                    //and in the next m lines, three integers U, V, C represent U->V has an edge of length C
static const std::string nodeFile = baseInDir + city + ".co"; //A total of N lines, an integer and two real numbers for each line id, x, y represents the longitude and latitude 
                                    //of the id node (but the input does not consider the id, only the order is read from 0 to n-1, the integer N is in the Edge file)
static const std::string GPTreeFile = baseInDir + "GP_Tree.data";
static const std::string DistMapFile = baseInDir + city + "_" + costs + "_dist_map.hps";
static const std::string reqFile = baseInDir + "requests_with_dist_" + costs + ".csv";//argv[1];
static const std::string vehFile = baseInDir + "vehicles.csv";//argv[2];
static const std::string medoidFile = baseInDir + city + "_area_medoids.txt";
static const std::string forecastFile = baseInDir + city + "_forecasts.txt";
static const std::string cityForecastFile = baseInDir + city + "_forecast_total_30_max.txt";
static const int numAreas = 17;
static const bool RevE = true;//false represents a directed graph，true Represents an undirected graph read edge copy reverse an edge
static const int Naive_Split_Limit = 33;//The sub-graph size is smaller than this value
static const int INF = 0x3fffffff;
static const int max_node = 10016; //264346;

static const std::string stdOutput = "CONOUT$";

extern int time_step;
extern const int max_wait_sec;
extern const int max_delay_sec;
extern const int velocity; // dm/s

extern const int penalty;
extern const int max_v_per_req;
extern const int min_req_per_v;

extern const int cost_scale_factor;

extern const double pickupPenalty;
extern const double delayPenalty;

extern const double minimal;

extern std::string baseOutDir;
extern std::string outDir; 
extern std::string logFile;

typedef boost::container::flat_set<int> uos;
//typedef std::set<int> uos;

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
class MyHash_Array3
{
public:
    std::size_t operator()(const std::array<int,3>& s) const
    {
        // return std::accumulate(s.begin(), s.end(), std::hash<int>{}(0), sum_of_hashes());
        // return std::hash<int>{}(std::accumulate(s.begin(), s.end(), 0));
        size_t seed = 0;
        //Combine the hash of the current vector with the hashes of the previous ones
        hash_combine(seed, s[0]);
        hash_combine(seed, s[1]);
        hash_combine(seed, s[2]);
        return seed;
    };

};

template<typename T, typename Container = std::deque<T> >
class iterable_queue : public std::queue<T, Container>
{
public:
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    iterator begin() { return this->c.begin(); }
    iterator end() { return this->c.end(); }
    const_iterator begin() const { return this->c.begin(); }
    const_iterator end() const { return this->c.end(); }
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
typedef std::pair<int, int> locReq;
typedef boost::container::flat_set<locReq> targetSet;
typedef std::vector<std::pair<int, int>> stMap;

//typedef tbb::concurrent_unordered_set<int> uosTBB;
typedef uos uosTBB;
typedef std::unordered_map<uos, pair<int, uos>, MyHash  > map_of_uos;
typedef std::unordered_map<pair<int, int>, int, PairHash> map_of_pairs;
typedef std::unordered_map<pair<int, int>, pair<int,int>, PairHash> pairs_to_pairs;
typedef std::unordered_map<pair<int, int>, vector<int>, PairHash> pairs_to_vec;
typedef std::unordered_set<pair<int, int>, PairHash> set_of_pairs;
typedef std::unordered_set<uos, MyHash> uos_of_uos;
typedef std::unordered_set<std::pair<int,int>, PairHash> uos_of_uos2;
typedef std::unordered_set<std::array<int,3>, MyHash_Array3> uos_of_uos3;
//typedef boost::container::flat_set<uos> uos_of_uos;
typedef std::vector<int> vecTBB;

