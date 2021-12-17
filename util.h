#pragma once

#include <vector>
#include<iostream>
#include <string>
#include <numeric>
#include <memory>
#include <utility>
#include <unordered_set>
#include <unordered_map>
#include <stdio.h>
#include "Vehicle.h"
#include "Request.h"
#include "globals.h"

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

bool read_requests(FILE*& in, vector<Request>& requests, int toTime);
void update_requests_with_dist(FILE*& in);

void handle_unserved(vector<Request>& unserved, vector<Request>& requests, int nowTime);

void update_vehicles(vector<Vehicle>& vehicles, vector<Request>& requests, int nowTime);

void finish_all(vector<Vehicle>& vehicles, vector<Request>& unserved);

void setupOutfiles(const std::string& outDir, const std::string& outFilename);

std::string GetCurrentTimeForFileName();

void print_stats(const std::string& outDir, const std::string& outFilename);
void print_ram(const std::string& outDir, const std::string& descriptor);
void print_line(const std::string& outDir, const std::string& outFilename, const std::string& message);
void write_vehicle_routes(const std::string& outDir, const std::vector<Vehicle>& vehicles, int now_time);
void log_stats();


void save_vector(const vector<int>& v);
void load_vector(vector<int>& v);
void save_vector_vector(const vector<vector<int> >& v);
void load_vector_vector(vector<vector<int> >& v);
void save_vector_pair(const vector<pair<int, int> >& v);
void load_vector_pair(vector<pair<int, int> >& v);
void save_map_int_pair(map<int, pair<int, int> >& h);
void load_map_int_pair(map<int, pair<int, int> >& h);
void save_map_int_int(map<int, int>& h);
void load_map_int_int(map<int, int>& h);
void save_map_intpair_int(map_of_pairs& h, FILE* out);
void load_map_intpair_int(map_of_pairs& h);
void save_map_intpair_intpair(pairs_to_pairs& h, FILE* out);
void load_map_intpair_intpair(pairs_to_pairs& h);

pair<int, int> getDisjunction(const uos& a, const uos& b);