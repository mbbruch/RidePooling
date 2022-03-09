#pragma once
#include<iostream>
#include <vector>
#include "Graph.h"
#include "Matrix.h"

struct Node
{
	Graph G;
	Node() { clear(); }
	int part;//Number of nodes
	int n = 0, father = 0, * son = 0, deep = 0;//n: Number of subgraph nodes,father Parent node number,
							//son[2] Left and right son #,deep Depth of tree where node is located
	vector<int>color;// The nodes are in that son
	Matrix dist, dist2, order;//border Distance,as well as border do floyd midpoint k scheme/program
	//order=(-1:directly connected)|(-2:connect in parent node)|(-3:connect in child nodes)|(-INF:no plan)
	map<int, pair<int, int> >borders;//first:real border number;second:<border serial number,
	//corresponding to the number in the subgraph(0~n-1)>
	vector<int>border_in_father, border_in_son, border_id, border_id_innode;//borders father and son, borders number in the list,
	//border number in original graph,border number in the node
	vector<int>path_record;//path query helper array, meaningless
	int cache_id, cache_bound;//current cache saved starting point #,current cache saved updated cacheof bound(<bound of begin updated end)
	vector<int>cache_dist, cache_dist2;//Current cache_dist saves the distance from the node cache_id in Figure G to each border, 
	//and only the part of the value less than or equal to cache_bound is correct
	vector<int>border_son_id;//current border the number of the son node where
	int min_border_dist;//The minimum distance between the current node and the boundary point of the cache (for KNN pruning)
	vector<pair<int, int> >min_car_dist;//<car_dist,node_id> closest to each border in the vehicle collection
	void save()
	{
		printf("%d %d %d %d %d %d %d\n", n, father, part, deep, cache_id, cache_bound, min_border_dist);
		for (int i = 0; i < part; i++)printf("%d ", son[i]); cout << endl;
		save_vector(color);
		order.save();
		save_map_int_pair(borders);
		save_vector(border_in_father);
		save_vector(border_in_son);
		save_vector(border_id);
		save_vector(border_id_innode);
		save_vector(path_record);
		save_vector(cache_dist);
		save_vector(cache_dist2);
		save_vector_pair(min_car_dist);
	}
	void load()
	{
		scanf("%d%d%d%d%d%d%d", &n, &father, &part, &deep, &cache_id, &cache_bound, &min_border_dist);
		if (son != NULL)delete[] son;
		son = new int[part];
		for (int i = 0; i < part; i++)scanf("%d", &son[i]);
		load_vector(color);
		order.load();
		load_map_int_pair(borders);
		load_vector(border_in_father);
		load_vector(border_in_son);
		load_vector(border_id);
		load_vector(border_id_innode);
		load_vector(path_record);
		load_vector(cache_dist);
		load_vector(cache_dist2);
		load_vector_pair(min_car_dist);
	}
	void init(int n)
	{
		part = n;
		son = new int[n];
		for (int i = 0; i < n; i++)son[i] = 0;
	}
	void clear()
	{
		part = n = father = deep = 0;
		delete[] son;
		dist.clear();
		dist2.clear();
		order.clear();
		G.clear();
		color.clear();
		borders.clear();
		border_in_father.clear();
		border_in_son.clear();
		border_id.clear();
		border_id_innode.clear();
		cache_dist.clear();
		cache_dist2.clear();
		cache_id = -1;
	}
	void make_border_edge()//Update the directly connected edges between borders to dist(build_dist_part1)
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
					if (dist.a[id1][id2] > G.cost[j])
					{
						dist.a[id1][id2] = G.cost[j];
						dist2.a[id1][id2] = G.cost2[j];
						order.a[id1][id2] = -1;
					}
				}
		}
	}
	void write()
	{
		printf("n=%d deep=%d father=%d", n, deep, father);
		printf(" son:("); for (int i = 0; i < part; i++) { if (i > 0)printf(" "); printf("%d", son[i]); }printf(")\n");
		//G.draw();
		printf("color:"); for (int i = 0; i < (int)color.size(); i++)printf("%d ", color[i]); cout << endl;
		printf("dist:\n");
		dist.write();
		dist2.write();
		printf("order:\n");
		order.write();
		printf("borders:");
		for (auto iter = borders.begin(); iter != borders.end(); iter++)printf("(%d,%d,%d)", iter->first, iter->second.first, iter->second.second); cout << endl;
		printf("border_id"); for (int i = 0; i < borders.size(); i++)printf("(%d,%d)", i, border_id[i]); printf("\n");
		printf("border_in_father"); for (int i = 0; i < borders.size(); i++)printf("(%d,%d)", i, border_in_father[i]); printf("\n");
		printf("border_in_son"); for (int i = 0; i < borders.size(); i++)printf("(%d,%d)", i, border_in_son[i]); printf("\n");
		printf("cache_dist "); for (int i = 0; i < cache_dist.size(); i++)printf("(%d,%d)", i, cache_dist[i]); printf("\n");
		printf("cache_dist2 "); for (int i = 0; i < cache_dist2.size(); i++)printf("(%d,%d)", i, cache_dist2[i]); printf("\n");
		printf("min_car_dist "); for (int i = 0; i < min_car_dist.size(); i++)printf("(i:%d,D:%d,id:%d)", i, min_car_dist[i].first, min_car_dist[i].second); printf("\n");
	}
};
