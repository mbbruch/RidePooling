#include <sstream>
#include <iostream>
#include<cstdio>
#include<cstdlib>
#include<cstring>
#include<vector>
#include<utility>
#include<ctime>
#include<fstream>
#include<algorithm>
#include<map>
#include<chrono>
#include "util.h"
#include<set>
#include<omp.h>
#include<cmath>
#include<queue>
#include<sys/timeb.h>
#include "hps_src/hps.h"
#include "GPtree.h"
#include "Graph.h"
#include "Matrix.h"
#include "globals.h"
#include "util.h"

using namespace std;

GPTree treeCost;

const bool DEBUG_ = false;
const bool Optimization_G_tree_Search = true;//Whether to enable full connection acceleration algorithm
const bool Optimization_KNN_Cut = true;//Whether to enable the KNN pruning query algorithm
const bool Optimization_Euclidean_Cut = false;//Whether to enable pruning algorithm based on Euclidean distance in cache query
const int Partition_Part = 4;//K Fork Tree
long long Additional_Memory = 0;//Additional space for building auxiliary matrices (int)
const bool Distance_Offset = false;//KNN Whether to consider the correction distance of the vehicle from the node
const double R_earth = 6371000.0;//Earth radius, used to input longitude and latitude into x,y coordinates
const double Unit = 0.1;//Unit length of road network file/m
const double PI = acos(-1.0);

double coor_dist(const coor& a, const coor& b)
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double Distance_(double a1, double b1, double a2, double b2)
{

	//	printf("a1: %f, b1: %f, a2: %f, b2: %f\n", a1, b1, a2, b2);
	double x, y, z, AB, A_B;
	if (b1 < 0)
	{
		b1 = abs(b1) + 180;
	}
	if (b2 < 0)
	{
		b2 = abs(b2) + 180;
	}

	a1 = a1 * PI / 180;
	b1 = b1 * PI / 180;
	a2 = a2 * PI / 180;
	b2 = b2 * PI / 180;
	x = R_earth * cos(b1) * cos(a1) - R_earth * cos(b2) * cos(a2);
	y = R_earth * cos(b1) * sin(a1) - R_earth * cos(b2) * sin(a2);
	z = R_earth * sin(b1) - R_earth * sin(b2);
	AB = sqrt(x * x + y * y + z * z);
	A_B = 2 * asin(AB / (2 * R_earth)) * R_earth;
	return A_B;
}

double GPTree::Euclidean_Dist(int S, int T)// Calculate the length of the Euclidean distance of nodes S and T in the road network data
{
	return Distance_(coordinate[S].x, coordinate[S].y, coordinate[T].x, coordinate[T].y) / Unit;
}

void GPTree::init_rand()
{
	srand(747929791);
}
void GPTree::read()
{
	printf("begin read\n");
	FILE *in = NULL;
	in = fopen(EdgeWeightsFile.c_str(), "r");
	fscanf(in, "%d %d\n", &G.n, &G.m);
	cout << G.n << " " << G.m << endl;
	G.init(G.n, G.m);
	for (int i = 0; i<G.n; i++)G.id[i] = i;
	int i, j, k;
	float l, m;
	for (i = 0; i<G.m; i++)//Iterate over edges
	{
		//int temp;
		fscanf(in, "%d %d %f %f\n", &j, &k, &l, &m);
		if (RevE == false)G.add_D(j - 1, k - 1, l, m);//One-way edge
		else G.add(j - 1, k - 1, l, m);//Two-way edge
	}
	fclose(in);

    print_line(outDir, logFile, "Reading node areas");

	node_areas.clear();
	node_areas.reserve(G.n);
	in = fopen(nodeFile.c_str(), "r");
	double d1, d2;
	int area = 0;
	for (i = 0; i < G.n; i++)//读取边
	{
		//int temp;
		int x = fscanf(in, "%d %lf %lf %d\n", &j, &d1, &d2, &area);
		node_areas.push_back(area);
		if (Optimization_Euclidean_Cut) //TODO revisit; would this help?
		{
			coordinate.push_back(coor(d1, d2));
		}
	}
	fclose(in);
	
    print_line(outDir, logFile, "Reading medoids");
	
	in = fopen(medoidFile.c_str(), "r");
	int medoid = 0;
	area_medoids.clear();
	while (2 == fscanf(in, "%d %d\n", &area, &medoid)) {
		area_medoids.push_back(medoid);
	}
	fclose(in);

    print_line(outDir, logFile, "Reading forecasts");
	in = fopen(forecastFile.c_str(), "r");
	int time = 0; area = 0; int forecast5 = 0, forecast10 = 0, forecast15 = 0, forecast30 = 0;
	area_forecasts.clear();
	vector<pair<pair<int, int>, vector<int>>> vec_forecasts;
	vec_forecasts.reserve(12 * 24 * area_medoids.size()); //*7 for Chicago
	while (6 == fscanf(in, "%d %d %d %d %d %d\n", &time, &area, &forecast5, &forecast10, &forecast15, &forecast30)) {
		vector<int> temp{ forecast5, forecast10, forecast15, forecast30 };
		vec_forecasts.push_back(make_pair(make_pair(time, area), temp));
	}
	fclose(in);
	area_forecasts.insert(vec_forecasts.begin(), vec_forecasts.end());
	vector<pair<pair<int, int>, vector<int>>>().swap(vec_forecasts);

	in = fopen(cityForecastFile.c_str(), "r");
	vector<std::pair<int,int>> vec_citywide;
	vec_citywide.reserve(12 * 24); //*7 for Chicago
	time = 0; forecast30 = 0;
	while (2 == fscanf(in, "%d %d\n", &time, &forecast30)) {
		vec_citywide.push_back(make_pair(time, forecast30));
	}
	city_forecasts_max.insert(vec_citywide.begin(), vec_citywide.end());
	vector<pair<int,int>>().swap(vec_citywide);
	fclose(in);
	cout << "read over" << endl;
	printf("read over\n");
}
void GPTree::save()
{
	printf("begin save\n");
	freopen(GPTreeFile.c_str(), "w", stdout);
	G.save();
	printf("%d %d %d\n", root, node_tot, node_size);
	save_vector(id_in_node);
	save_vector_vector(car_in_node);
	for (int i = 0; i < node_size; i++)
	{
		printf("\n");
		node[i].save();
	}
	freopen(stdOutput.c_str(), "w", stdout);
	printf("save_over\n");
}
void GPTree::load()
{
	FILE* in = freopen(GPTreeFile.c_str(), "r", stdin);
	printf("load_begin\n");
	G.load();
	scanf("%d%d%d", &root, &node_tot, &node_size);
	load_vector(id_in_node);
	load_vector_vector(car_in_node);
	node = new Node[G.n * 2 + 2];
	for (int i = 0; i < node_size; i++)
	{
		node[i].load();
	}
	fclose(in);
	freopen(stdOutput.c_str(), "r", stdin);
}

void GPTree::initialize(bool load_cache) {
	if(load_cache){
		//TIME_TICK_START
		init_rand();
		read();
		Additional_Memory = 2 * G.n*log2(G.n);
		printf("G.real_border:%d\n", G.real_node());
		//TIME_TICK_END
		//TIME_TICK_PRINT("load from cache")
		load();
	}else{
		//TIME_TICK_START
		init_rand();
		read();
		Additional_Memory = 2 * G.n*log2(G.n);
		printf("G.real_border:%d\n", G.real_node());
		build();
		init_dist_map();
		//TIME_TICK_END
		//TIME_TICK_PRINT("build from scratch")
		//save();
	}
}

void GPTree::init_dist_map() {
	dist_map.clear();
	std::ifstream in_file(DistMapFile, std::ifstream::binary);
	in_file.seekg(0, in_file.end);
	size_t serialized_size = in_file.tellg();
	in_file.seekg(0, in_file.beg);
	dist_map = hps::from_stream<std::vector<pair<int, int>>>(in_file);
	int nCombos = max_node * (max_node - 1) / 2;
	if (dist_map.size() < nCombos) {
		dist_map.clear();
		dist_map.reserve(max_node * (max_node - 1) / 2);
		for (int i = 1; i <= max_node; i++) {
			for (int j = i + 1; j <= max_node; j++) {
				dist_map.push_back(search_cache(i - 1, j - 1));
			}
		}
		std::ofstream out_file(DistMapFile, std::ofstream::binary);
		hps::to_stream(dist_map, out_file);
		out_file.close();
	}
}

void GPTree::write()
{
	printf("root=%d node_tot=%d\n", root, node_tot);
	for (int i = 1; i<node_tot; i++)
	{
		printf("node:%d\n", i);
		node[i].write();
	}
}

void GPTree::add_border(int x, int id, int id2)//向x点的border集合中加入一个新真实id,在子图的虚拟id为id2,并对其标号为border中的编号
{
	node[x].borders.try_emplace(id,pair<int,int>((int)node[x].borders.size(), id2));
}

void GPTree::make_border(int x, const vector<int>& color)//计算点x的border集合，二部图之间的边集为E
{
	for (int i = 0; i<node[x].G.n; i++)
	{
		int id = node[x].G.id[i];
		for (int j = node[x].G.head[i]; j; j = node[x].G.next[j])
			if (color[i] != color[node[x].G.list[j]])
			{
				add_border(x, id, i);
				break;
			}
	}
}

int GPTree::partition_root(int x)//Returns the maximum number of blocks that the node can be divided into within the limit of Additional_Memory
{
	if ((long long)node[x].G.n * node[x].G.n <= Additional_Memory)return node[x].G.n;
	int l = 2, r = max(2, (int)sqrt(Additional_Memory)), mid;//二分块数
	while (l<r)
	{
		mid = (l + r + 1) >> 1;
		int num = node[x].G.Split_Borders(mid);
		if (num * num>Additional_Memory)r = mid - 1;
		else l = mid;
	}

	return l;
}

void GPTree::build(int x, int f)//Recursive tree construction, current node x, leaf size f, subgraph g of current node
{
	if (x == 1)//x root
	{
		node = new Node[G.n * 2 + 2];
		node_size = G.n * 2;
		node_tot = 2;
		root = 1;
		node[x].deep = 1;
		node[1].G = G;
	}
	else//x not root
	{
		node[x].deep = node[node[x].father].deep + 1;
	}
	node[x].n = node[x].G.n;
	// Number of sons of decision root node
	if (x == root && Optimization_G_tree_Search)
	{
		node[x].init(partition_root(x));
		printf("root's part:%d\n", node[x].part);
	}
	else if (node[x].G.n<Naive_Split_Limit)node[x].init(node[x].n);
	else node[x].init(Partition_Part);
	if (node[x].n>f)
	{
		//Child node label
		int top = node_tot;
		for (int i = 0; i<node[x].part; i++)
		{
			node[x].son[i] = top + i;
			node[top + i].father = x;
		}
		node_tot += node[x].part;
		//Add between the two border
		Graph** graph;
		graph = new Graph * [node[x].part];
		for (int i = 0; i<node[x].part; i++)graph[i] = &node[node[x].son[i]].G;
		node[x].color = node[x].G.Split(graph, node[x].part);
		delete[] graph;
		make_border(x, node[x].color);
		//Transfer border to child node
		for (auto iter = node[x].borders.begin(); iter != node[x].borders.end(); iter++)
		{
			//printf("(%d,%d,%d)",iter->first,iter->second.first,iter->second.second);
			node[x].color[iter->second.second] = -node[x].color[iter->second.second] - 1;
		}
		//cout<<endl;
		vector<int>tot(node[x].part, 0);
		for (int i = 0; i<node[x].n; i++)
		{
			if (node[x].color[i]<0)
			{
				node[x].color[i] = -node[x].color[i] - 1;
				add_border(node[x].son[node[x].color[i]], node[x].G.id[i], tot[node[x].color[i]]);
			}
			tot[node[x].color[i]]++;
		}
		//Recursive child node
		for (int i = 0; i<node[x].part; i++)
			build(node[x].son[i]);
	}
	else if (node[x].n>50)cout << endl;
	node[x].dist.init(node[x].borders.size());
	node[x].dist2.init(node[x].borders.size());
	node[x].order.init(node[x].borders.size());
	node[x].order.cover(-INF);
	if (x == 1)//x build for root dist
	{
		build_border_in_father_son();
		build_dist_part1(root);
		build_dist_part2(root);
		//Calculate the leaf number where each node is located
		id_in_node.clear();
		for (int i = 0; i<node[root].G.n; i++)id_in_node.push_back(-1);
		for (int i = 1; i<node_tot; i++)
			if (node[i].G.n == 1)
				id_in_node[node[i].G.id[0]] = i;
		//Set up cache
		for (int i = 1; i <= node_tot; i++)
		{
			node[i].cache_id = -1;
			for (int j = 0; j<node[i].borders.size(); j++)
			{
				node[i].cache_dist.push_back(0);
				node[i].cache_dist2.push_back(0);
				node[i].min_car_dist.push_back(make_pair(INF, -1));
			}
		}
		{
			//Set up car_in_node;
			vector<int>empty_vector;
			empty_vector.clear();
			car_in_node.clear();
			//for (int i = 0; i<G.n; i++)car_in_node.push_back(empty_vector);
		}
	}
}

//Bottom-up merge subgraph interior dist
void GPTree::build_dist_part1(int x)
{
	//Calculate the internal dist of the child node and pass it to x
	for (int i = 0; i<node[x].part; i++)if (node[x].son[i])build_dist_part1(node[x].son[i]);
	if (node[x].son[0])//非叶子
	{
		//Create edges between x sub-nodes
		node[x].make_border_edge();
		//Calculate the internal real dist matrix		
		node[x].dist.floyd(node[x].order, node[x].dist2);
	}
	else;//leaf
		 //Pass internal edge weights to parent node
	if (node[x].father)
	{
		int y = node[x].father, i, j;
		vector<int>id_in_fa(node[x].borders.size());
		//Calculate the number of the border of the subgraph in the border sequence of the parent node, if it does not exist, it is -1
		for (auto x_iter1 = node[x].borders.begin(); x_iter1 != node[x].borders.end(); x_iter1++)
		{
			auto y_iter1 = node[y].borders.find(x_iter1->first);
			if (y_iter1 == node[y].borders.end())id_in_fa[x_iter1->second.first] = -1;
			else id_in_fa[x_iter1->second.first] = y_iter1->second.first;
		}
		//Pass the fully connected edge rights inside the subgraph to the father
		for (i = 0; i<(int)node[x].borders.size(); i++)
			for (j = 0; j<(int)node[x].borders.size(); j++)
				if (id_in_fa[i] != -1 && id_in_fa[j] != -1)
				{
					int* p = &node[y].dist.a[id_in_fa[i]][id_in_fa[j]];
					int* p2 = &node[y].dist2.a[id_in_fa[i]][id_in_fa[j]];
					if ((*p)>node[x].dist.a[i][j])
					{
						(*p) = node[x].dist.a[i][j];
						(*p2) = node[x].dist2.a[i][j];
						node[y].order.a[id_in_fa[i]][id_in_fa[j]] = -3;
					}
				}
	}
	return;
}

//Correct the subgraph external dist from top to bottom
void GPTree::build_dist_part2(int x)
{
	if (x != root)node[x].dist.floyd(node[x].order, node[x].dist2);
	if (node[x].son[0])
	{
		//Calculate the number of this node's border number in the border sequence of the subgraph
		vector<int>id_(node[x].borders.size());
		vector<int>color_(node[x].borders.size());
		for (auto iter1 = node[x].borders.begin(); iter1 != node[x].borders.end(); iter1++)
		{
			int c = node[x].color[iter1->second.second];
			color_[iter1->second.first] = c;
			int y = node[x].son[c];
			id_[iter1->second.first] = node[y].borders[iter1->first].first;
		}
		//Modified subgraph edge weight
		for (int i = 0; i<(int)node[x].borders.size(); i++)
			for (int j = 0; j<(int)node[x].borders.size(); j++)
				if (color_[i] == color_[j])
				{
					int y = node[x].son[color_[i]];
					int* p = &node[y].dist.a[id_[i]][id_[j]];
					int* p2 = &node[y].dist2.a[id_[i]][id_[j]];
					if ((*p)>node[x].dist.a[i][j])
					{
						(*p) = node[x].dist.a[i][j];
						(*p2) = node[x].dist2.a[i][j];
						node[y].order.a[id_[i]][id_[j]] = -2;
					}
				}
		//Recursive child node
		for (int i = 0; i<node[x].part; i++)
			if (node[x].son[i])build_dist_part2(node[x].son[i]);
	}
}

//Calculate the number of each node's border in the father/son borders array
void GPTree::build_border_in_father_son()
{
	int i, j, x, y;
	for (x = 1; x<node_tot; x++)
	{
		for (i = 0; i<node[x].borders.size(); i++)node[x].border_id.push_back(0);
		for (i = 0; i<node[x].borders.size(); i++)node[x].border_id_innode.push_back(0);
		for (auto iter = node[x].borders.begin(); iter != node[x].borders.end(); iter++)
		{
			node[x].border_id[iter->second.first] = iter->first;
			node[x].border_id_innode[iter->second.first] = iter->second.second;
		}
		node[x].border_in_father.clear();
		node[x].border_in_son.clear();
		for (i = 0; i<node[x].borders.size(); i++)
		{
			node[x].border_in_father.push_back(-1);
			node[x].border_in_son.push_back(-1);
		}
		if (node[x].father)
		{
			y = node[x].father;
			for (auto iter = node[x].borders.begin(); iter != node[x].borders.end(); iter++)
			{
				auto iter2 = node[y].borders.find(iter->first);
				if (iter2 != node[y].borders.end())
					node[x].border_in_father[iter->second.first] = iter2->second.first;
			}
		}
		if (node[x].son[0])
		{
			for (auto iter = node[x].borders.begin(); iter != node[x].borders.end(); iter++)
			{
				y = node[x].son[node[x].color[iter->second.second]];
				auto iter2 = node[y].borders.find(iter->first);
				if (iter2 != node[y].borders.end())
					node[x].border_in_son[iter->second.first] = iter2->second.first;
			}
			for (int i = 0; i<node[x].borders.size(); i++)
				node[x].border_son_id.push_back(node[x].son[node[x].color[node[x].border_id_innode[i]]]);
		}
	}
}

//Calculate the number of real borders of x 
//(ignoring the borders between internal subgraphs)
int GPTree::real_border_number(int x)
{
	int i, j, re = 0, id;
	map<int, int>vis;
	for (i = 0; i < node[x].G.n; i++)vis[node[x].G.id[i]] = 1;
	for (i = 0; i < node[x].G.n; i++)
	{
		id = node[x].G.id[i];
		for (j = G.head[id]; j; j = G.next[j])
			if (vis[G.list[j]] == 0)
			{
				re++;
				break;
			}
	}
	return re;
}

//FROM HERE BELOW: functions not currently in use

std::pair<int, int> GPTree::search(int S, int T)//Query the shortest path length of S-T
{
	if (S == T)return std::pair<int, int>{0, 0};

	int i, j, k, k2, p;
	int LCA, x = id_in_node[S], y = id_in_node[T];
	LCA = find_LCA(x, y);
	vector<int>dist[2];
	dist[0].push_back(0);
	dist[1].push_back(0);
	vector<int>dist2[2];
	dist2[0].push_back(0);
	dist2[1].push_back(0);
	//Naive G-Tree calculation
	for (int t = 0; t < 2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		while (node[p].father != LCA)
		{
			push_borders_up(p, dist[t], dist2[t], t);
			p = node[p].father;
		}
		if (t == 0)x = p;
		else y = p;
	}
	vector<int>id[2];//The border sequence number of the child node border in LCA
	for (int t = 0; t < 2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		for (i = j = 0; i < (int)dist[t].size(); i++)
			if (node[p].border_in_father[i] != -1)
			{
				id[t].push_back(node[p].border_in_father[i]);
				dist[t][j]  = dist[t][i];
				dist2[t][j] = dist2[t][i];
				j++;
			}
		while (dist[t].size() > id[t].size()) { dist[t].pop_back(); }
	}
	//Final match
	int MIN = INF;
	int min2 = INF;
	for (i = 0; i < dist[0].size(); i++)
	{
		int i_ = id[0][i];
		for (j = 0; j < dist[1].size(); j++)
		{
			k = dist[0][i] + dist[1][j] + node[LCA].dist.a[i_][id[1][j]];
			k2 = dist2[0][i] + dist2[1][j] + node[LCA].dist2.a[i_][id[1][j]];
			if (k < MIN) {
				MIN = k;
				min2 = k2;
			}
		}
	}
	return std::pair<int, int>{round(MIN/cost_scale_factor), min2};
}
//Record the shortest path length from S to the boundary point of node x in dist1, 
//calculate the distance from S to the real border of x.father,
//update dist1 
//type==0 push up, type==1 push down
void GPTree::push_borders_up(int x, vector<int>& dist1, vector<int>& dist2, int type)
{
	if (node[x].father == 0)return;

	int y = node[x].father;
	vector<int>dist1_2(node[y].borders.size(), INF);
	vector<int>dist2_2(node[y].borders.size(), INF);
	for (int i = 0; i < node[x].borders.size(); i++) {
		if (node[x].border_in_father[i] != -1) {
			dist1_2[node[x].border_in_father[i]] = dist1[i];
			dist2_2[node[x].border_in_father[i]] = dist2[i];
		}
	}

	int** dist = node[y].dist.a;
	int** dist_2ndary = node[y].dist2.a;
	int* begin, * end;
	begin = new int[node[x].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	for (int i = 0; i < dist1_2.size(); i++)
	{
		if (dist1_2[i] < INF)begin[tot0++] = i;
		else if (node[y].border_in_father[i] != -1)end[tot1++] = i;
	}
	if (type == 0)
	{
		for (int i = 0; i < tot0; i++)
		{
			int i_ = begin[i];
			for (int j = 0; j < tot1; j++)
			{
				int j_ = end[j];
				if (dist1_2[j_] > dist1_2[i_] + dist[i_][j_]) {
					dist1_2[j_] = dist1_2[i_] + dist[i_][j_];
					dist2_2[j_] = dist2_2[i_] + dist_2ndary[i_][j_];
				}
			}
		}
	}
	else {
		for (int i = 0; i < tot0; i++)
		{
			int i_ = begin[i];
			for (int j = 0; j < tot1; j++)
			{
				int j_ = end[j];
				if (dist1_2[j_] > dist1_2[i_] + dist[j_][i_])
					dist1_2[j_] = dist1_2[i_] + dist[j_][i_];
			}
		}
	}

	dist1 = dist1_2;
	dist2 = dist2_2;
	delete[] begin;
	delete[] end;
}

//FROM HERE BELOW: the functions that are actually called during the simulations
std::pair<int, int> GPTree::get_dist(int S, int T) {
	if (S == T) {
		return std::pair<int,int>{0,0};
	}
	int s_ = S; 
	int t_ = T;
	if (S > T) {
		s_ = T;
		t_ = S;
	}
/*	example where max_node = 100:
	1 to 2... 100 = 99 entries = 0:98
	2 to 3... 100 = 98 entries = 99 : 196
	3 to 4... 100 = 97 entries = 197 : 293
	4 to 5... 100 = 96 entries = 294 : 389
	5 to 6... 100 = 95 entries = 390 : 484
	6 to 7... 100 = 94 entries = 485 : 578
	7 to 8... 100 = 93 entries = 579 : 671
	index of first entries :
	entry 1, 2 = 0 = (1 - 1) * 100 - 1 * (1 - 0) / 2
	entry 2, 3 = 99 = (2 - 1) * 100 - 2 * (2 - 1) / 2
	entry 3, 4 = 197 = (3 - 1) * 100 - 3 * (3 - 1) / 2
	entry 4, 5 = 294 = (4 - 1) * 100 - 4 * (4 - 1) / 2
	entry 5, 6 = 390 = (5 - 1) * 100 - 5 * (5 - 1) / 2
	entry 6, 7 = 485 = (6 - 1) * 100 - 6 * (6 - 1) / 2
	entry 7, 8 = 579 = (7 - 1) * 100 - 7 * (7 - 1) / 2
	so s->t is at(s - 1) * max_node - s * (s - 1) / 2 + (t - s - 1) */
	return dist_map[(s_ - 1) * max_node - s_ * (s_ - 1) / 2 + (t_ - s_ - 1)];
}

//Query the shortest path length of S-T, 
//and process the cache of the nodes along the way as the result of S. 
//The part with weight >=bound is not calculated. 
//If not, the pruning returns to INF
std::pair<int,int> GPTree::search_cache(int S, int T, int bound){
	//Simple G-Tree calculation and cache maintenance
	if (S == T)return std::pair<int,int>{0,0};
	int i, j, k, p;
	int x = id_in_node[S], y = id_in_node[T];
	int LCA = find_LCA(x, y);

	//Calculate the number of nodes along the way from two leaves to LCA
	vector<int>node_path[2];
	for (int t = 0; t<2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		while (node[p].father != LCA)
		{
			node_path[t].push_back(p);
			p = node[p].father;
		}
		node_path[t].push_back(p);
		if (t == 0)x = p;
		else y = p;
	}

	//Push S from the leaf to the lower layer of the LCA
	node[id_in_node[S]].cache_id = S;
	node[id_in_node[S]].cache_bound = bound;
	node[id_in_node[S]].min_border_dist = 0;
	node[id_in_node[S]].cache_dist[0] = 0;
	node[id_in_node[S]].cache_dist2[0] = 0;
	for (i = 0; i + 1<node_path[0].size(); i++)
	{
		if (node[node_path[0][i]].min_border_dist >= bound)return std::pair<int,int>{INF,INF};
		#pragma omp critical (pushborderscache)
		push_borders_up_cache(node_path[0][i]);
	}

	//Calculate the cache of T at the lower node of LCA
	if (node[x].min_border_dist >= bound)return std::pair<int, int>{INF, INF};
	#pragma omp critical (pushborderscache)
	push_borders_brother_cache(x, y);
	//Push the data of T in the lower layer of LCA to the bottom node T
for (int i = node_path[1].size() - 1; i > 0; i--)
{
	if (node[node_path[1][i]].min_border_dist >= bound)return std::pair<int, int>{INF, INF};

#pragma omp critical (pushborderscache)
	push_borders_down_cache(node_path[1][i], node_path[1][i - 1]);
}

//Final answer
return std::pair<int, int>{ round(node[id_in_node[T]].cache_dist[0] / cost_scale_factor), node[id_in_node[T]].cache_dist2[0]};
}

//Cache the shortest path length from S to the boundary point of node x in x.cache_dist, 
//calculate the distance from S to the real border of x.father,
//and update x.father.cache
void GPTree::push_borders_up_cache(int x, int bound)
{
	if (node[x].father == 0)return;
	int y = node[x].father;
	if (node[x].cache_id == node[y].cache_id && bound <= node[y].cache_bound)return;
	node[y].cache_id = node[x].cache_id;
	node[y].cache_bound = bound;
	vector<int>* dist1 = &node[x].cache_dist, * dist2 = &node[y].cache_dist;
	vector<int>* dist1_2ndary = &node[x].cache_dist2, * dist2_2ndary = &node[y].cache_dist2;
	for (int i = 0; i < (*dist2).size(); i++)(*dist2)[i] = INF;
	for (int i = 0; i < (*dist2_2ndary).size(); i++)(*dist2_2ndary)[i] = INF;
	for (int i = 0; i < node[x].borders.size(); i++)
		if (node[x].border_in_father[i] != -1)
		{
			if (node[x].cache_dist[i] < bound) { //cache within bounds
				(*dist2)[node[x].border_in_father[i]] = (*dist1)[i];
				(*dist2_2ndary)[node[x].border_in_father[i]] = (*dist1_2ndary)[i];
			}
			else { //cache outside of bounds
				(*dist2)[node[x].border_in_father[i]] = -1;
				(*dist2_2ndary)[node[x].border_in_father[i]] = -1;
			}
		}
	int** dist = node[y].dist.a;
	int** dist_2ndary = node[y].dist2.a;
	int* begin, * end;//Calculated serial number, uncalculated serial number
	begin = new int[node[x].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	for (int i = 0; i < (*dist2).size(); i++)
	{
		if ((*dist2)[i] == -1) {
			(*dist2)[i] = INF; 
			(*dist2_2ndary)[i] = INF;
		}
		else if ((*dist2)[i] < INF){
			begin[tot0++] = i;
		}		
		else if (node[y].border_in_father[i] != -1)
		{
			if (Optimization_Euclidean_Cut == false || Euclidean_Dist(node[x].cache_id, node[y].border_id[i]) < bound)
				end[tot1++] = i;
		}
	}
	for (int i = 0; i < tot0; i++)
	{
		int i_ = begin[i];
		for (int j = 0; j < tot1; j++)
		{
			if ((*dist2)[end[j]] > (*dist2)[i_] + dist[i_][end[j]]) {
				(*dist2)[end[j]] = (*dist2)[i_] + dist[i_][end[j]];
				(*dist2_2ndary)[end[j]] = (*dist2_2ndary)[i_] + dist_2ndary[i_][end[j]];

			}
		}
	}
	delete[] begin;
	delete[] end;
	node[y].min_border_dist = INF;
	for (int i = 0; i < node[y].cache_dist.size(); i++)
		if (node[y].border_in_father[i] != -1)
			node[y].min_border_dist = min(node[y].min_border_dist, node[y].cache_dist[i]);
}

//Cache the shortest path length from S to the boundary point of node x in x.cache_dist, 
//calculate the distance from S to the real border of x's son y, 
//update y.cache
void GPTree::push_borders_down_cache(int x, int y, int bound)
{
	if (node[x].cache_id == node[y].cache_id && bound <= node[y].cache_bound)return;
	node[y].cache_id = node[x].cache_id;
	node[y].cache_bound = bound;
	vector<int>* dist1 = &node[x].cache_dist, * dist2 = &node[y].cache_dist;
	vector<int>* dist1_2ndary = &node[x].cache_dist2, * dist2_2ndary = &node[y].cache_dist2;
	for (int i = 0; i < (*dist2).size(); i++)(*dist2)[i] = INF;
	for (int i = 0; i < (*dist2_2ndary).size(); i++)(*dist2_2ndary)[i] = INF;
	for (int i = 0; i < node[x].borders.size(); i++)
		if (node[x].son[node[x].color[node[x].border_id_innode[i]]] == y)
		{
			if (node[x].cache_dist[i] < bound) { //begin within the bound
				(*dist2)[node[x].border_in_son[i]] = (*dist1)[i];
				(*dist2_2ndary)[node[x].border_in_son[i]] = (*dist1_2ndary)[i];
			}
			else { //begin outside the bound
				(*dist2)[node[x].border_in_son[i]] = -1;
				(*dist2_2ndary)[node[x].border_in_son[i]] = -1;
			}
		}
	int** dist = node[y].dist.a;
	int** dist_2ndary = node[y].dist2.a;
	int* begin, * end;//Calculated serial number, uncalculated serial number
	begin = new int[node[y].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	for (int i = 0; i < (*dist2).size(); i++)
	{
		if ((*dist2)[i] == -1) {
			(*dist2)[i] = INF;
			(*dist2_2ndary)[i] = INF;
		}
		else if ((*dist2)[i] < INF) {
			begin[tot0++] = i;
		}
		else
		{
			if (Optimization_Euclidean_Cut == false || Euclidean_Dist(node[x].cache_id, node[y].border_id[i]) < bound)
				end[tot1++] = i;
		}
	}
	for (int i = 0; i < tot0; i++)
	{
		int i_ = begin[i];
		int j_ = 0;
		int tocompare = 0;
		int tocompare2 = 0;
		int dist2i = (*dist2)[i_];
		int* disti = dist[i_];
		int dist2i_2ndary = (*dist2_2ndary)[i_];
		int* disti_2ndary = dist_2ndary[i_];
		for (int j = 0; j < tot1; j++)
		{
			j_ = end[j];
			tocompare = dist2i + disti[j_];
			tocompare2 = dist2i_2ndary + disti_2ndary[j_];
			if ((*dist2)[j_] > tocompare) {
				(*dist2)[j_] = tocompare;
				(*dist2_2ndary)[j_] = tocompare2;
			}
		}
	}
	delete[] begin;
	delete[] end;
	//Update y.cache
	node[y].min_border_dist = INF;
	for (int i = 0; i < node[y].cache_dist.size(); i++)
		if (node[y].border_in_father[i] != -1)
			node[y].min_border_dist = min(node[y].min_border_dist, node[y].cache_dist[i]);
}

void GPTree::push_borders_brother_cache(int x, int y, int bound)//Cache the shortest path length from S to the boundary point of node x in x.cache_dist, calculate the distance from S to the real border of x's sibling node y, update y.cache
{
	int S = node[x].cache_id, LCA = node[x].father, i, j;
	if (node[y].cache_id == S && node[y].cache_bound >= bound)return;
	int p;
	node[y].cache_id = S;
	node[y].cache_bound = bound;
	vector<int>id_LCA[2], id_now[2];//The border sequence number of the child node candidate border in LCA, the number of the internal border sequence of the child node candidate border
	int nodexcacheid = node[x].cache_id;
	int nodepbifi = 0;
	for (int t = 0; t < 2; t++)
	{
		Node& nodep = (t == 0) ? node[x] : node[y];
		for (i = j = 0; i < (int)nodep.borders.size(); i++)
		{
			nodepbifi = nodep.border_in_father[i];
			if (nodepbifi != -1)
				if ((t == 1 && (Optimization_Euclidean_Cut == false || Euclidean_Dist(nodexcacheid, nodep.border_id[i]) < bound))
					||
					(t == 0 && nodep.cache_dist[i] < bound)
					)
				{
					id_LCA[t].push_back(nodepbifi);
					id_now[t].push_back(i);
				}
		}

	}
	for (int i = 0; i < node[y].cache_dist.size(); i++)node[y].cache_dist[i] = INF;
	for (int i = 0; i < node[y].cache_dist2.size(); i++)node[y].cache_dist2[i] = INF;

	vector<int>& idnow1 = id_now[1];
	vector<int>& idlca1 = id_LCA[1];
	for (int i = 0; i < id_LCA[0].size(); i++) {
		int xdist = node[x].cache_dist[id_now[0][i]];
		int* thisA = node[LCA].dist.a[id_LCA[0][i]];
		int xdist2 = node[x].cache_dist2[id_now[0][i]];
		int* thisA2 = node[LCA].dist2.a[id_LCA[0][i]];
		for (int j = 0; j < id_LCA[1].size(); j++)
		{
			int k = xdist + thisA[idlca1[j]];
			int k2 = xdist2 + thisA2[idlca1[j]];
			if (k < node[y].cache_dist[idnow1[j]]) {
				node[y].cache_dist[idnow1[j]] = k;
				node[y].cache_dist2[idnow1[j]] = k2;
			}
		}
	}

	int** dist = node[y].dist.a;
	int** dist_2ndary = node[y].dist2.a;
	//vector<int>begin,end;//已算出的序列编号,未算出的序列编号
	int* begin, * end;
	begin = new int[node[y].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	for (int i = 0; i < node[y].cache_dist.size(); i++)
	{
		if (node[y].cache_dist[i] < bound)begin[tot0++] = i;
		else if (node[y].cache_dist[i] == INF)
		{
			if (Optimization_Euclidean_Cut == false || Euclidean_Dist(node[x].cache_id, node[y].border_id[i]) < bound)
				end[tot1++] = i;
		}
	}
	for (int i = 0; i < tot0; i++)
	{
		int i_ = begin[i];
		int j_ = 0;
		int tocompare = 0;
		int tocompare2 = 0;
		int dist2i = node[y].cache_dist[i_];
		int* disti = dist[i_];
		int dist2i_2ndary = node[y].cache_dist2[i_];
		int* disti_2ndary = dist_2ndary[i_];
		for (int j = 0; j < tot1; j++)
		{
			j_ = end[j];
			tocompare = dist2i + disti[j_];
			tocompare2 = dist2i_2ndary + disti_2ndary[j_];
			if (node[y].cache_dist[j_] > tocompare) {
				node[y].cache_dist[j_] = tocompare;
				node[y].cache_dist2[j_] = tocompare2;

			}
		}
	}

	delete[] begin;
	delete[] end;
	node[y].min_border_dist = INF;
	for (int i = 0; i < node[y].cache_dist.size(); i++)
		if (node[y].border_in_father[i] != -1)
			node[y].min_border_dist = min(node[y].min_border_dist, node[y].cache_dist[i]);
}

//Return the shortest path length of S-T, and store the scheme of nodes passing along the way into the order array
std::pair<int, int> GPTree::find_path(const int S, const int T, vector<int>& order)
{
	order.clear();
	if (S == T)
	{
		order.push_back(S);
		order.push_back(T);
		return std::pair<int, int>{0, 0};
	}

	int i, j, k, k2, p;

	int LCA, x = id_in_node[S], y = id_in_node[T];
	LCA = find_LCA(x, y);
	vector<int>dist[2], dist_, dist2[2],dist2_;
	dist[0].push_back(0);
	dist[1].push_back(0);
	dist2[0].push_back(0);
	dist2[1].push_back(0);


	//Naive G-Tree calculation
	for (int t = 0; t < 2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		while (node[p].father != LCA)
		{
			push_borders_up_path(p, dist[t], dist2[t]);
			p = node[p].father;
		}
		if (t == 0)x = p;
		else y = p;
	}

	vector<int>id[2];//The border sequence number of the child node's border in the LCA
	for (int t = 0; t < 2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		for (i = j = 0; i < (int)dist[t].size(); i++)
			if (node[p].border_in_father[i] != -1)
			{
				id[t].push_back(node[p].border_in_father[i]);
				dist[t][j] = dist[t][i];
				dist2[t][j] = dist2[t][i];
				j++;
			}
		while (dist[t].size() > id[t].size()) { 
			dist[t].pop_back(); 
			dist2[t].pop_back();
		}
	}
	//Final match
	int MIN = INF;
	int min2 = INF;
	int S_ = -1, T_ = -1;// The number of the optimal path connected by borders in LCA
	for (i = 0; i < (int)dist[0].size(); i++)
		for (j = 0; j < (int)dist[1].size(); j++)
		{
			k = dist[0][i] + dist[1][j] + node[LCA].dist.a[id[0][i]][id[1][j]];
			k2 = dist2[0][i] + dist2[1][j] + node[LCA].dist2.a[id[0][i]][id[1][j]];
			if (k < MIN)
			{
				MIN = k;
				min2 = k2;
				S_ = id[0][i];
				T_ = id[1][j];
			}
		}
	if (MIN < INF)//Existing path, restoring path
	{
		for (int t = 0; t < 2; t++)
		{
			int p, now;
			if (t == 0)p = x, now = node[LCA].border_in_son[S_];
			else p = y, now = node[LCA].border_in_son[T_];
			while (node[p].n > 1)
			{
				if (node[p].path_record[now] >= 0)
				{
					find_path_border(p, now, node[p].path_record[now], order, 0);
					now = node[p].path_record[now];
				}
				else if (node[p].path_record[now] > -INF)
				{
					int temp = now;
					now = node[p].border_in_son[now];
					p = -node[p].path_record[temp];
				}
				else break;
			}
			if (t == 0)//Flip, Supplemental connection path
			{
				reverse(order.begin(), order.end());
				order.push_back(node[LCA].border_id[S_]);
				find_path_border(LCA, S_, T_, order, 0);
			}
		}
	}
	return std::pair<int, int>{ round(MIN/cost_scale_factor),min2 };
}


//Return the shortest path length of S-T, and store the scheme of nodes passing along the way into the order array
std::pair<int, int> GPTree::find_path_simple(const int S, const int T, vector<int>& order)
{
	order.clear();
	if (S == T)
	{
		order.push_back(S);
		order.push_back(T);
		return std::pair<int, int>{0,0};
	}

	int i, j, k, k2, p;

	int LCA, x = id_in_node[S], y = id_in_node[T];
	LCA = find_LCA(x, y);
	vector<int>dist[2], dist_, dist2[2],dist2_;
	dist[0].push_back(0);
	dist[1].push_back(0);
	dist2[0].push_back(0);
	dist2[1].push_back(0);


	//Naive G-Tree calculation
	for (int t = 0; t < 2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		while (node[p].father != LCA)
		{
			p = node[p].father;
		}
		if (t == 0)x = p;
		else y = p;
	}

	vector<int>id[2];//The border sequence number of the child node's border in the LCA
	for (int t = 0; t < 2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		for (i = j = 0; i < (int)dist[t].size(); i++)
			if (node[p].border_in_father[i] != -1)
			{
				id[t].push_back(node[p].border_in_father[i]);
				dist[t][j] = dist[t][i];
				dist2[t][j] = dist2[t][i];
				j++;
			}
		while (dist[t].size() > id[t].size()) { 
			dist[t].pop_back(); 
			dist2[t].pop_back();
		}
	}
	//Final match
	int MIN = INF;
	int min2 = INF;
	int S_ = -1, T_ = -1;// The number of the optimal path connected by borders in LCA
	for (i = 0; i < (int)dist[0].size(); i++)
		for (j = 0; j < (int)dist[1].size(); j++)
		{
			k = dist[0][i] + dist[1][j] + node[LCA].dist.a[id[0][i]][id[1][j]];
			k2 = dist2[0][i] + dist2[1][j] + node[LCA].dist2.a[id[0][i]][id[1][j]];
			if (k < MIN)
			{
				MIN = k;
				min2 = k2;
				S_ = id[0][i];
				T_ = id[1][j];
			}
		}
	if (MIN < INF)//Existing path, restoring path
	{
		find_path_border(LCA, S_, T_, order, 0);
	}
	return std::pair<int,int>{ round(MIN / cost_scale_factor),min2};
}


//Return the node path of the border numbered from S to T in node x, store it in vector<int>, 
//push the part S+1~T except the starting point S to the end of v, 
//rev=0 means positive order, rev=1 means reverse order
const void GPTree::find_path_border(int x, int S, int T, vector<int>& v, int rev)
{
	if (node[x].order.a[S][T] == -1)
	{
		if (rev == 0)v.push_back(node[x].border_id[T]);
		else v.push_back(node[x].border_id[S]);
	}
	else if (node[x].order.a[S][T] == -2)
	{
		find_path_border(node[x].father, node[x].border_in_father[S], node[x].border_in_father[T], v, rev);
	}
	else if (node[x].order.a[S][T] == -3)
	{
		find_path_border(node[x].son[node[x].color[node[x].border_id_innode[S]]], node[x].border_in_son[S], node[x].border_in_son[T], v, rev);
	}
	else if (node[x].order.a[S][T] >= 0)
	{
		int k = node[x].order.a[S][T];
		if (rev == 0)
		{
			find_path_border(x, S, k, v, rev);
			find_path_border(x, k, T, v, rev);
		}
		else
		{
			find_path_border(x, k, T, v, rev);
			find_path_border(x, S, k, v, rev);
		}
	}
}

//Record the shortest path length from S to the boundary point of node x in dist1, 
//calculate the distance from S to the real border of x.father, 
// update dist1, and
// record the path to x.father in x.father.path_record 
// (>=0 Indicates the node, 
// <0 indicates that it is passed on to the node's son, 
// -INF indicates no predecessor)
void GPTree::push_borders_up_path(int x, vector<int>& dist1, vector<int>& dist2)
{
	if (node[x].father == 0)return;
	int y = node[x].father;
	vector<int>dist1_replacement(node[y].borders.size(), INF);
	vector<int>dist2_replacement(node[y].borders.size(), INF);
	vector<int>* order = &node[y].path_record;
	(*order).clear();
	for (int i = 0; i < node[y].borders.size(); i++)(*order).push_back(-INF);
	for (int i = 0; i < node[x].borders.size(); i++)
		if (node[x].border_in_father[i] != -1)
		{
			dist1_replacement[node[x].border_in_father[i]] = dist1[i];
			dist2_replacement[node[x].border_in_father[i]] = dist2[i];
			(*order)[node[x].border_in_father[i]] = -x;
		}
	int** dist = node[y].dist.a;
	int** dist_2ndary = node[y].dist2.a;
	int* begin, * end; //Calculated serial number, uncalculated serial number
	begin = new int[node[x].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	for (int i = 0; i < dist1_replacement.size(); i++)
	{
		if (dist1_replacement[i] < INF)begin[tot0++] = i;
		else if (node[y].border_in_father[i] != -1)end[tot1++] = i;
	}
	for (int i = 0; i < tot0; i++)
	{
		int i_ = begin[i];
		for (int j = 0; j < tot1; j++)
		{
			if (dist1_replacement[end[j]] > dist1_replacement[i_] + dist[i_][end[j]])
			{
				dist1_replacement[end[j]] = dist1_replacement[i_] + dist[i_][end[j]];
				dist2_replacement[end[j]] = dist2_replacement[i_] + dist_2ndary[i_][end[j]];
				(*order)[end[j]] = i_;
			}
		}
	}
	dist1 = dist1_replacement;
	dist2 = dist2_replacement;
	delete[] begin;
	delete[] end;
}
//Calculate the Lowest Common Ancestor of two nodes xy on the tree
int GPTree::find_LCA(int x, int y)
{
	if (node[x].deep < node[y].deep)swap(x, y);
	while (node[x].deep > node[y].deep)x = node[x].father;
	while (x != y) { x = node[x].father; y = node[y].father; }
	return x;
}

