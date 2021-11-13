#pragma once
#include <vector>
#include<cmath>
#include "util.h"
#include "globals.h"

struct Graph// Undirected graph structure
{
	int n, m;//n points, m edges, points from 0 to n-1
	int tot;
	vector<int> id; //id[i] The real number of the i point in the sub-graph in the original graph
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
	int Split_Borders(int nparts);
	struct state {
		int id; int len; int index;
		state(int a = 0, int b = 0, int c = 0);
	}; //用于dijkstra的二元组

	struct cmp {
		bool operator()(const state& a, const state& b);
	};//重载priority_queue的比较函数
	void dijkstra(int S, vector<int>& dist);
	vector<int> KNN(int S, int K, vector<int>T);
	vector<int> find_path(int S, int T);
	int real_node();

	//给定起点集合S，处理到每个结点的前K短路长度以及起点编号在list中的编号
	vector<vector<int> >K_Near_Dist, K_Near_Order;
	void KNN_init(const vector<int>& S, int K);
	vector<int>* KNN_Dijkstra(int S);
};