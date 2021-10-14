#pragma once

#include <vector>
#include <string>
#include <numeric>
#include <memory>
#include <utility>
#include <unordered_set>
#include <unordered_map>
#include "Vehicle.h"
#include "Request.h"

using namespace std;

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

FILE* get_requests_file(const char* file);

void read_vehicles(const char* file, vector<Vehicle>& vehicles);

void setup_vehicles(int nCars, vector<Vehicle>& vehicles);

bool read_requests(FILE*& in, vector<Request>& requests, int toTime, map_of_pairs& dist);

void handle_unserved(vector<Request>& unserved, vector<Request>& requests, int nowTime);

void update_vehicles(vector<Vehicle>& vehicles, vector<Request>& requests, int nowTime, map_of_pairs& dist);

void finish_all(vector<Vehicle>& vehicles, vector<Request>& unserved, map_of_pairs& dist);

void setupOutfiles(const std::string& outDir, const std::string& outFilename);

std::string GetCurrentTimeForFileName();

void print_stats(const std::string& outDir, const std::string& outFilename);

void print_line(const std::string& outDir, const std::string& outFilename, const std::string& message);

void write_route(const Vehicle& veh);
void write_request(const Request& req);

void log_stats();

class costComponents {

public:
    double distance;
    double time;
    double emissions_nox;
    double emissions_sox;
    double emissions_nh3;
    double emissions_pm;
    double emissions_ghg;
    costComponents();
    double getTotal() const;
    bool operator <(const costComponents& other) const;
};
