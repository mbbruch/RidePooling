#pragma once

#include <vector>
#include "util.h"
#include "globals.h"
using namespace std;


struct Graph// Undirected graph structure
{
	int n, m;//n points, m edges, points from 0 to n-1
	int tot;
	vector<int> id;//id[i] The real number of the i point in the sub-graph in the original graph
	vector<int>head, list, next, cost;//Adjacency list
	Graph();
	~Graph();
	void save();
	void load();
	void add_D(int a, int b, int c);
	void add(int a, int b, int c);
	void init(int N, int M, int t = 1);
	void clear();
	void draw();
	//图划分算法
	vector<int>color;//01染色数组
	vector<int>con;//连通性
	vector<int> Split(Graph* G[], int nparts);
	vector<int> Split_Naive(Graph& G1, Graph& G2);
	int Split_Borders(int nparts);
	struct state { 
		int id; int len; int index; 
		state(int a = 0, int b = 0, int c = 0);
	}; //用于dijkstra的二元组

	struct cmp { bool operator()(const state& a, const state& b);
	};//重载priority_queue的比较函数
	void dijkstra(int S, vector<int>& dist);
	vector<int> KNN(int S, int K, vector<int>T);
	vector<int> find_path(int S, int T);
	int real_node();

	//给定起点集合S，处理到每个结点的前K短路长度以及起点编号在list中的编号
	vector<vector<int> >K_Near_Dist, K_Near_Order;
	void KNN_init(const vector<int>& S, int K);
	vector<int>* KNN_Dijkstra(int S);

	friend void initialize(bool load_cache, map_of_pairs& dist);
	friend int find_path(int S, int T, vector<int>& order);
	friend int get_dist(int S, int T, const map_of_pairs& dist, bool simplestCheck);
	friend void init_dist_map(map_of_pairs& dist_map);
	friend void reinitialize_dist_map(std::set<std::pair<int, int>>& ongoingLocs,
		set<int>& allToAll1,
		set<int>& allToAll2,
		map_of_pairs& mop);
};

	extern void initialize(bool load_cache, map_of_pairs& dist);
	extern int find_path(int S, int T, vector<int>& order);
	extern int get_dist(int S, int T, const map_of_pairs& dist, bool simplestCheck=false);
	extern void init_dist_map(map_of_pairs& dist_map);
	extern void reinitialize_dist_map(set<pair<int, int>>& ongoingLocs,
		set<int>& allToAll,
		map_of_pairs& mop);

struct Heap//双指针大根堆
{
	Heap();
	int n;
	vector<int>id, iid, a;//id[i]表示编号为i的元素在a中的位置，iid[i]表示a[i]的id（id:[0~n-1],a/iid:[1~n]）
	void clear();
	void swap_(int x, int y);
	void up(int x);
	void down(int x);
	int top();
	int top_id();
	int size();
	void change(int x, int num);
	void add(int x, int num);
	void push(int num);
	void draw();
};