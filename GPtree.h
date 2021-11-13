#pragma once
#include<iostream>
#include <vector>
#include "util.h"
#include "globals.h"
#include "util.h"
#include "Graph.h"
#include "Matrix.h"
using namespace std;

struct coor { coor(double a = 0.0, double b = 0.0) :x(a), y(b) {}double x, y; };

struct GPTree
{
	std::string EdgeWeightsFile;
	map_of_pairs dist_map;
	Graph G;
	int root;
	vector<int>id_in_node;//The leaf node number where the real node is
	vector<vector<int> >car_in_node;//Used to hang border method KNN, record the number of each node boarding
	vector<int>car_offset;//Used to record the distance between the car id and the node where the car is located
	vector<coor>coordinate;
	struct Node
	{
		Graph G;
		Node() { clear(); }
		int part;//Number of nodes
		int n = 0, father =0, *son =0, deep =0;//n: Number of subgraph nodes,father Parent node number,
								//son[2] Left and right son #,deep Depth of tree where node is located
		vector<int>color;// The nodes are in that son
		Matrix dist, order;//border Distance,as well as border do floyd midpoint k scheme/program,
		//order=(-1:directly connected)|(-2:connect in parent node)|(-3:connect in child nodes)|(-INF:no plan)
		map<int, pair<int, int> >borders;//first:real border number;second:<border serial number,
		//corresponding to the number in the subgraph(0~n-1)>
		vector<int>border_in_father, border_in_son, border_id, border_id_innode;//borders father and son, borders number in the list,
		//border number in original graph,border number in the node
		vector<int>path_record;//path query helper array, meaningless
		int cache_id, cache_bound;//current cache saved starting point #，current cache saved updated cacheof bound(<bound of begin updated end)
		vector<int>cache_dist;//current cache_dist保存的是从图G中结点cache_id到每个border的距离，其中只有值小于等于cache_bound的部分值是正确的
		// current cache_dist saves the distance from the node cache_id in Figure G to each border, and only part of the value less than or equal to cache_bound is correct
		vector<int>border_son_id;//current border the number of the son node where
		int min_border_dist;//The minimum distance between the current node and the boundary point of the cache (for KNN pruning)
		vector<pair<int, int> >min_car_dist;//<car_dist,node_id> closest to each border in the vehicle collection
		void save()
		{
			printf("%d %d %d %d %d %d %d\n", n, father, part, deep, cache_id, cache_bound, min_border_dist);
			for (int i = 0; i<part; i++)printf("%d ", son[i]); cout << endl;
			save_vector(color);
			order.save();
			save_map_int_pair(borders);
			save_vector(border_in_father);
			save_vector(border_in_son);
			save_vector(border_id);
			save_vector(border_id_innode);
			save_vector(path_record);
			save_vector(cache_dist);
			save_vector_pair(min_car_dist);
		}
		void load()
		{
			scanf("%d%d%d%d%d%d%d", &n, &father, &part, &deep, &cache_id, &cache_bound, &min_border_dist);
			if (son != NULL)delete[] son;
			son = new int[part];
			for (int i = 0; i<part; i++)scanf("%d", &son[i]);
			load_vector(color);
			order.load();
			load_map_int_pair(borders);
			load_vector(border_in_father);
			load_vector(border_in_son);
			load_vector(border_id);
			load_vector(border_id_innode);
			load_vector(path_record);
			load_vector(cache_dist);
			load_vector_pair(min_car_dist);
		}
		void init(int n)
		{
			part = n;
			son = new int[n];
			for (int i = 0; i<n; i++)son[i] = 0;
		}
		void clear()
		{
			part = n = father = deep = 0;
			delete[] son;
			dist.clear();
			order.clear();
			G.clear();
			color.clear();
			borders.clear();
			border_in_father.clear();
			border_in_son.clear();
			border_id.clear();
			border_id_innode.clear();
			cache_dist.clear();
			cache_id = -1;
		}
		void make_border_edge()//Update the directly connected edges between borders to dist(build_dist1)
		{
			int i, j;
			for (auto iter = borders.begin(); iter != borders.end(); iter++)
			{
				i = iter->second.second;
				for (j = G.head[i]; j; j = G.next[j])
					if (color[i] != color[G.list[j]])
					{
						int id1, id2;
						id1 = iter->second.first;
						id2 = borders[G.id[G.list[j]]].first;
						if (dist.a[id1][id2]>G.cost[j])
						{
							dist.a[id1][id2] = G.cost[j];
							order.a[id1][id2] = -1;
						}
					}
			}
		}
		void write()
		{
			printf("n=%d deep=%d father=%d", n, deep, father);
			printf(" son:("); for (int i = 0; i<part; i++) { if (i>0)printf(" "); printf("%d", son[i]); }printf(")\n");
			//G.draw();
			printf("color:"); for (int i = 0; i<(int)color.size(); i++)printf("%d ", color[i]); cout << endl;
			printf("dist:\n");
			dist.write();
			printf("order:\n");
			order.write();
			printf("borders:");
			for (auto iter = borders.begin(); iter != borders.end(); iter++)printf("(%d,%d,%d)", iter->first, iter->second.first, iter->second.second); cout << endl;
			printf("border_id"); for (int i = 0; i<borders.size(); i++)printf("(%d,%d)", i, border_id[i]); printf("\n");
			printf("border_in_father"); for (int i = 0; i<borders.size(); i++)printf("(%d,%d)", i, border_in_father[i]); printf("\n");
			printf("border_in_son"); for (int i = 0; i<borders.size(); i++)printf("(%d,%d)", i, border_in_son[i]); printf("\n");
			printf("cache_dist "); for (int i = 0; i<cache_dist.size(); i++)printf("(%d,%d)", i, cache_dist[i]); printf("\n");
			printf("min_car_dist "); for (int i = 0; i<min_car_dist.size(); i++)printf("(i:%d,D:%d,id:%d)", i, min_car_dist[i].first, min_car_dist[i].second); printf("\n");
		}
	};
	int node_tot, node_size;
	Node *node;
	double coor_dist(const coor& a, const coor& b);
	double Euclidean_Dist(int S, int T);
	void read();
	void initialize(bool load_cache);
	void save_dist_map();
	void load_dist_map();
	void init_dist_map();
	void reinitialize_dist_map(set<pair<int, int>>& ongoingLocs, vector<int>& vec1, vector<int>& vec2);
	void init_rand();
	void save();
	void load();
	void write();
	void add_border(int x, int id, int id2);
	void make_border(int x, const vector<int>& color);
	int partition_root(int x = 1);
	void build(int x = 1, int f = 1);
	void build_dist1(int x = 1);
	void build_dist2(int x = 1);
	void build_border_in_father_son();

	void push_borders_up(int x, vector<int>& dist1, int type);
	void push_borders_up_cache(int x, int bound = INF);
	void push_borders_down_cache(int x, int y, int bound = INF);
	void push_borders_brother_cache(int x, int y, int bound = INF);
	void push_borders_up_path(int x, vector<int>& dist1);
	int find_LCA(int x, int y);
	int search(int S, int T);
	int get_dist(int S, int T, bool simplestCheck=false, bool bOnlySearchCache=false);
	int search_cache(int S, int T, int bound = INF);
	const int find_path(const int S, const int T, vector<int>& order);
	const int find_path_simple(const int S, const int T, vector<int>& order);
	int real_border_number(int x);
	const void find_path_border(int x, int S, int T, vector<int>& v, int rev);


	int begin[10000], end[10000];//已算出的序列编号,未算出的序列编号
};

extern GPTree treeCost, treeDist;