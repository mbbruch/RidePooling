#pragma once
#include<iostream>
#include <vector>
#include "util.h"
#include "globals.h"
#include "util.h"
#include "Graph.h"
#include "Node.h"
using namespace std;

struct coor { coor(double a = 0.0, double b = 0.0) :x(a), y(b) {}double x, y; };

struct GPTree
{
	std::string EdgeWeightsFile;
	stMap dist_map;
	Graph G;
	int root;
	vector<int>id_in_node;//The leaf node number where the real node is
	vector<vector<int> >car_in_node;//Used to hang border method KNN, record the number of each node boarding
	vector<coor>coordinate;
	vector<int> node_areas;
	vector<int> area_medoids;
	int vehStartingPoint;
	pairs_to_vec area_forecasts;
	std::unordered_map<int, int> city_forecasts_max;
	int node_tot, node_size;
	Node *node;
	double coor_dist(const coor& a, const coor& b);
	double Euclidean_Dist(int S, int T);
	void read();
	void initialize(bool load_cache);
	void init_dist_map();
	void init_rand();
	void save();
	void load();
	void write();
	void add_border(int x, int id, int id2);
	void make_border(int x, const vector<int>& color);
	int partition_root(int x = 1);
	void build(int x = 1, int f = 1);
	void build_dist_part1(int x = 1);
	void build_dist_part2(int x = 1);
	void build_border_in_father_son();

	void push_borders_up(int x, vector<int>& dist1, vector<int>& dist2, int type);
	void push_borders_up_cache(int x, int bound = INF);
	void push_borders_down_cache(int x, int y, int bound = INF);
	void push_borders_brother_cache(int x, int y, int bound = INF);
	void push_borders_up_path(int x, vector<int>& dist1, vector<int>& dist2);
	int find_LCA(int x, int y);
	std::pair<int,int> search(int S, int T);
	std::pair<int, int> get_dist(int S, int T);
	std::pair<int, int> search_cache(int S, int T, int bound = INF);
	std::pair<int, int> find_path(const int S, const int T, vector<int>& order);
	int real_border_number(int x);
	const void find_path_border(int x, int S, int T, vector<int>& v, int rev);
};

extern GPTree treeCost;