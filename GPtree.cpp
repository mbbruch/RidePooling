#include <sstream>
#include<iostream>
#include<cstdio>
#include<cstdlib>
#include<cstring>
#include<vector>
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
#include<metis.h>
#include "GPtree.h"
#include "globals.h"
int times[10];//辅助计时变量；
int cnt_type0, cnt_type1;

using namespace std;

const bool DEBUG_ = false;
const bool Optimization_G_tree_Search = true;//Whether to enable full connection acceleration algorithm
const bool Optimization_KNN_Cut = true;//Whether to enable the KNN pruning query algorithm
const bool Optimization_Euclidean_Cut = false;//Whether to enable pruning algorithm based on Euclidean distance in cache query
const char Edge_File[] = "/ocean/projects/eng200002p/mbruchon/RidePooling/In/chicago.edge";// In the first line, two integers n, m represent the number of points and edges, 
									//and in the next m lines, three integers U, V, C represent U->V has an edge of length C
const char Node_File[] = "/ocean/projects/eng200002p/mbruchon/RidePooling/In/chicago.co"; //A total of N lines, an integer and two real numbers for each line id, x, y represents the longitude and latitude 
									//of the id node (but the input does not consider the id, only the order is read from 0 to n-1, the integer N is in the Edge file)

const char ST_File[] = "/ocean/projects/eng200002p/mbruchon/RidePooling/In/s_t_for_map.csv";
const char GPTree_File[] = "/ocean/projects/eng200002p/mbruchon/RidePooling/In/GP_Tree_chicago.data";
const char DistMap_FileShort[] = "dist_map_chicago.data";
const char DistMap_File[] = "/ocean/projects/eng200002p/mbruchon/RidePooling/In/dist_map_chicago.data";

const int Global_Scheduling_Cars_Per_Request = 30000000;//Each plan accurately counts up to the number of vehicles reserved(time overhead)
const double Unit = 0.1;//路网文件的单位长度/m
const double R_earth = 6371000.0;//地球半径，用于输入经纬度转化为x,y坐标
const double PI = acos(-1.0);
const int Partition_Part = 4;//K Fork Tree
long long Additional_Memory = 0;//Additional space for building auxiliary matrices (int)
const int Naive_Split_Limit = 33;//The sub-graph size is smaller than this value
const bool RevE = true;//false represents a directed graph，true Represents an undirected graph read edge copy reverse an edge
const bool Distance_Offset = false;//KNN是否考虑车辆距离结点的修正距离
const bool DEBUG1 = false;
//#define //TIME_TICK_START gettimeofday( &tv, NULL ); ts = tv.tv_sec * 100000 + tv.tv_usec / 10;
//#define //TIME_TICK_END gettimeofday( &tv, NULL ); te = tv.tv_sec * 100000 + tv.tv_usec / 10;
//#define //TIME_TICK_PRINT(T) printf("%s RESULT: %lld (0.01MS)\r\n", (#T), te - ts );
//struct timeval tv;
long long ts, te;
static int rootp = 0;
Graph G;

Graph::state::state(int a, int b, int c) :id(a), len(b), index(c) {};
bool Graph::cmp::operator()(const state& a, const state& b) { 
	return a.len > b.len; 
}

void save_vector(const vector<int> &v)
{
	printf("%d ", (int)v.size());
	for (int i = 0; i<(int)v.size(); i++)printf("%d ", v[i]);
	printf("\n");
}
void load_vector(vector<int>& v)
{
	v.clear();
	int n, i, j;
	int result = scanf("%d", &n);
	if (result <= 0) {
		int x = 5;
		x = 5 + 5;;
		exit(0);
	}
	if (n < 1000000000 && n >= 0) { 
		v.reserve(n); 
	} else { 
		printf("%d\n", result);
		v.reserve(n / n);
		exit(0); 
	}
	for (i = 0; i<n; i++)
	{
		scanf("%d", &j);
		v.push_back(j);
	}
}
void save_vector_vector(const vector<vector<int> > &v)
{
	printf("%d\n", (int)v.size());
	for (int i = 0; i<(int)v.size(); i++)save_vector(v[i]);
	printf("\n");
}
void load_vector_vector(vector<vector<int> > &v)
{
	v.clear();
	int n, i, j;
	scanf("%d", &n);
	v.reserve(n);
	vector<int>ls;
	for (i = 0; i<n; i++)
	{
		load_vector(ls);
		v.push_back(ls);
	}
}
void save_vector_pair(const vector<pair<int, int> > &v)
{
	printf("%d ", (int)v.size());
	for (int i = 0; i<(int)v.size(); i++)printf("%d %d ", v[i].first, v[i].second);
	printf("\n");
}
void load_vector_pair(vector<pair<int, int> > &v)
{
	v.clear();
	int n, i, j, k;
	scanf("%d", &n);
	v.reserve(n);
	for (i = 0; i<n; i++)
	{
		scanf("%d%d", &j, &k);
		v.push_back(make_pair(j, k));
	}
}
void save_map_int_pair(map<int, pair<int, int> > &h)
{
	printf("%d\n", h.size());
	for (auto iter = h.begin(); iter != h.end(); iter++)
		printf("%d %d %d\n", iter->first, iter->second.first, iter->second.second);
}
void load_map_int_pair(map<int, pair<int, int> > &h)
{
	int n, i, j, k, l;
	scanf("%d", &n);
	vector<pair<int, pair<int, int>>> temp;
	temp.reserve(n);
	for (i = 0; i<n; i++)
	{
		scanf("%d%d%d", &j, &k, &l);
		temp.push_back(make_pair(j, pair<int, int>{k, l}));
		//h[j] = make_pair(k, l);
	}
	map<int, pair<int, int>> tempMap((temp.begin()), temp.end());
	h = tempMap;
}
void save_map_int_int(map<int, int> &h)
{
	printf("%d\n", h.size());
	for (auto iter = h.begin(); iter != h.end(); iter++)
		printf("%d %d\n", iter->first, iter->second);
}
void load_map_int_int(map<int, int> &h)
{
	int n, i, j, k;
	scanf("%d", &n);
	vector<pair<int, int>> temp;
	temp.reserve(n);
	for (i = 0; i<n; i++)
	{
		scanf("%d%d", &j, &k);
		temp.push_back(make_pair(j,k));
		h[j] = k;
	}
	map<int, int> tempMap((temp.begin()), temp.end());
	h = tempMap;
}
void save_map_intpair_int(map_of_pairs& h, FILE* out)
{
	fprintf(out, "%d\n", h.size());
	for (auto iter = h.begin(); iter != h.end(); iter++)
		fprintf(out, "%d %d %d\n", iter->first.first, iter->first.second, iter->second);
}
void load_map_intpair_int(map_of_pairs& h)
{
	int n, i, j, k;
	scanf("%d", &n);
	vector<pair<pair<int, int>, int>> temp;
	temp.reserve(n);
	for (i = 0; i < n; i++)
	{
		scanf("%d%d%d", &i, &j, &k);
		temp.push_back(make_pair(pair<int, int>{i, j}, k));
		//h[make_pair(i, j)] = k;
	}
	map_of_pairs tempMap((temp.begin()), temp.end());
	h = tempMap;
}

void save_dist_map(map_of_pairs& dist_map) {

	FILE* out = fopen(DistMap_File, "w");
	save_map_intpair_int(dist_map, out);
	fclose(out);
}

void load_dist_map(map_of_pairs& dist_map)
{
	FILE* in = fopen(DistMap_File, "r");
	load_map_intpair_int(dist_map);
	fclose(in);
}



struct coor { coor(double a = 0.0, double b = 0.0) :x(a), y(b) {}double x, y; };
vector<coor>coordinate;
double coor_dist(const coor &a, const coor &b)
{
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
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
	x = R_earth * cos(b1)*cos(a1) - R_earth * cos(b2)*cos(a2);
	y = R_earth * cos(b1)*sin(a1) - R_earth * cos(b2)*sin(a2);
	z = R_earth * sin(b1) - R_earth * sin(b2);
	AB = sqrt(x*x + y*y + z*z);
	A_B = 2 * asin(AB / (2 * R_earth)) * R_earth;
	return A_B;
}
double Euclidean_Dist(int S, int T)// Calculate the length of the Euclidean distance of nodes S and T in the road network data
{
	return Distance_(coordinate[S].x, coordinate[S].y, coordinate[T].x, coordinate[T].y) / Unit;
}
double Euclidean_Dist_LatLong(int S, int T)// Calculate the length of the Euclidean distance of nodes S and T in the road network data
{

//	printf("S: %d, T: %d\n", S, T);
	double toReturn =  round(Distance_(coordinate[S].x, coordinate[S].y, coordinate[T].x, coordinate[T].y));

	printf("S: %d, T: %d, distance: %f\n", S, T, toReturn);
	return toReturn;
}

struct Matrix//矩阵
{
	int n;//矩阵长宽
	int** a;

	Matrix() :n(0), a(NULL) {}

	~Matrix() { clear(); }

	void save()
	{
		printf("%d\n", n);
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
				printf("%d ", a[i][j]);
			printf("\n");
		}
	}

	void load()
	{
		scanf("%d", &n);
		a = new int* [n];
		for (int i = 0; i < n; i++)a[i] = new int[n];
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				scanf("%d", &a[i][j]);
	}

	void cover(int x)
	{
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				a[i][j] = x;
	}

	void init(int N)
	{
		clear();
		n = N;
		a = new int* [n];
		for (int i = 0; i < n; i++)a[i] = new int[n];
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				a[i][j] = INF;
		for (int i = 0; i < n; i++)a[i][i] = 0;
	}

	void clear()
	{
		for (int i = 0; i < n; i++)delete[] a[i];
		delete[] a;
	}

	void floyd()//对矩阵a进行floyd
	{
		int i, j, k;
		for (k = 0; k < n; k++)
			for (i = 0; i < n; i++)
				for (j = 0; j < n; j++)
					if (a[i][j] > a[i][k] + a[k][j])a[i][j] = a[i][k] + a[k][j];
	}

	void floyd(Matrix& order)//对矩阵a进行floyd,将方案记录到order中
	{
		int i, j, k;
		for (k = 0; k < n; k++)
			for (i = 0; i < n; i++)
				for (j = 0; j < n; j++)
					if (a[i][j] > a[i][k] + a[k][j])
					{
						a[i][j] = a[i][k] + a[k][j];
						order.a[i][j] = k;
					}
	}

	void write()
	{
		printf("n=%d\n", n);
		for (int i = 0; i < n; i++, cout << endl)
			for (int j = 0; j < n; j++)printf("%d ", a[i][j]);
	}

	Matrix& operator =(const Matrix& m)
	{
		if (this != (&m))
		{
			init(m.n);
			for (int i = 0; i < n; i++)
				for (int j = 0; j < n; j++)
					a[i][j] = m.a[i][j];
		}
		return *this;
	}
};

struct G_Tree
{
	int root;
	vector<int>id_in_node;//The leaf node number where the real node is
	vector<vector<int> >car_in_node;//Used to hang border method KNN, record the number of each node boarding
	vector<int>car_offset;//Used to record the distance between the car id and the node where the car is located
	struct Node
	{
		Node() { clear(); }
		int part;//Number of nodes
		int n = 0, father =0, *son =0, deep =0;//n: Number of subgraph nodes,father Parent node number,
								//son[2] Left and right son #,deep Depth of tree where node is located
		Graph G;//Subgraph
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
	void save();
	void load();
	void write();
	void add_border(int x, int id, int id2);
	void make_border(int x, const vector<int>& color);
	int partition_root(int x = 1);
	void build(int x = 1, int f = 1, const Graph& g = G);
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
	int search_cache(int S, int T, int bound = INF);
	int find_path(int S, int T, vector<int>& order);
	int real_border_number(int x);
	void find_path_border(int x, int S, int T, vector<int>& v, int rev);


	int begin[10000], end[10000];//已算出的序列编号,未算出的序列编号

}tree;

void init()
{
	srand(747929791);
}
void read()
{
	printf("begin read\n");
	FILE *in = NULL;
	in = fopen(Edge_File, "r");
	fscanf(in, "%d %d\n", &G.n, &G.m);
	cout << G.n << " " << G.m << endl;
	G.init(G.n, G.m);
	for (int i = 0; i<G.n; i++)G.id[i] = i;
	int i, j, k, l;
	for (i = 0; i<G.m; i++)//Iterate over edges
	{
		//int temp;
		fscanf(in, "%d %d %d\n", &j, &k, &l);
		if (RevE == false)G.add_D(j - 1, k - 1, l);//One-way edge
		else G.add(j - 1, k - 1, l);//Two-way edge
	}
	fclose(in);
	if (Optimization_Euclidean_Cut) //TODO revisit; would this help?
	{
		in = fopen(Node_File, "r");
		double d1, d2;
		for (i = 0; i<G.n; i++)//读取边
		{
			//int temp;
			fscanf(in, "%d %lf %lf\n", &j, &d1, &d2);
			coordinate.push_back(coor(d1, d2));
		}
		fclose(in);
		printf("read over\n");
	}
}
void save()
{
	printf("begin save\n");
	freopen(GPTree_File, "w", stdout);
	tree.save();
//	freopen("/dev/tty", "w", stdout);
	printf("save_over\n");
}
void load()
{
	FILE* in = freopen(GPTree_File, "r", stdin);
	tree.load();
	fclose(in);
//	freopen("/dev/tty", "r", stdin);
}

void initialize(bool load_cache, map_of_pairs& dist) {
	if(load_cache){
		//TIME_TICK_START
		init();
		read();
		Additional_Memory = 2 * G.n*log2(G.n);
		printf("G.real_border:%d\n", G.real_node());
		//TIME_TICK_END
		//TIME_TICK_PRINT("load from cache")
		load();
	}else{
		//TIME_TICK_START
		init();
		read();
		Additional_Memory = 2 * G.n*log2(G.n);
		printf("G.real_border:%d\n", G.real_node());
		tree.build();
		//TIME_TICK_END
		//TIME_TICK_PRINT("build from scratch")
		save();
	}

}

void init_dist_map(map_of_pairs& dist_map)
{
	FILE* in = fopen(ST_File, "r");
	int size;
	fscanf(in, "%d\n", &size);
	vector<int> nodes;
	nodes.reserve(size);
	int j;
	for (int i = 0; i < size; i++)
	{
		fscanf(in, "%d\n", &j);
		nodes.push_back(j);
	}
	fclose(in);

	dist_map.clear();
	dist_map.reserve(size * (size - 1) / 2);
	dist_map.rehash(1.0 * (nodes.size() * (nodes.size() - 1) / 2));
	int this_s, this_t;
	for (int s = 0; s < size; s++) {
		this_s = nodes[s];
		for (int t = s + 1; t < size; t++) {
			this_t = nodes[t];
			dist_map[pair<int, int>{this_s, this_t}] = tree.search_cache(this_s - 1, this_t - 1);
		}
	}
	dist_map.rehash(1.0 * (nodes.size() * (nodes.size() - 1) / 2));
	save_dist_map(dist_map);
}

void reinitialize_dist_map(set<pair<int, int>>& ongoingLocs,
	set<int>& allToAll, map_of_pairs& mop) {

	vector<pair<int, int>> vec1(ongoingLocs.begin(), ongoingLocs.end());
	set<pair<int, int>>().swap(ongoingLocs);
	vector<int> vec2( allToAll.begin(), allToAll.end() );
	set<int>().swap(allToAll);
	
	int i = 0;
	const int vec1Size = vec1.size();
	#pragma omp parallel for default(none) private(i) shared(vec1, tree, mop)
	for (i = 0;  i < vec1Size; i++) {
		int dist = tree.search_cache(vec1[i].first - 1, vec1[i].second - 1);
		#pragma omp critical(updatemop)
		mop.insert(pair<pair<int, int>, int>(make_pair(vec1[i].first, vec1[i].second), dist));
	}
	vector<pair<int, int>>().swap(vec1);
	i = 0;
	const int vec2Size = vec2.size();
	#pragma omp parallel for default(none) private(i) shared(vec2, tree, mop)
	for (i = 0; i < vec2Size; i++) {
		int this_s = vec2[i];
		int this_t = 0;
		for (int j = i + 1; j < vec2Size; j++) {
			this_t = vec2[j];
			int dist = tree.search_cache(this_s - 1, this_t - 1);
			#pragma omp critical(updatemop)
			mop.insert(pair<pair<int, int>, int>(make_pair(this_s, this_t), dist));
		}
	}
	vector<int>().swap(vec2);
	mop.rehash(mop.size());
}

int search(int s,int t) {
	return tree.search(s, t);
}

int search_cache(int s,int t) {
	return tree.search_cache(s, t);
}

int find_path(int S, int T, vector<int> &order) {
	return tree.find_path(S, T, order);
}

//TODO: this assume distances are the same in both directions
int get_dist(int S, int T, const map_of_pairs& dist, bool simplestCheck) {
	pair<int, int> st;
	if (S < T) {
		st = make_pair(S, T);
	} else if (S > T){
		st = make_pair(T, S);
	} else {
		return 0;
	}
	auto found = dist.find(st);
	if (found != dist.end()) {
		return found->second;
	}
	else {
		return search_cache(S - 1, T - 1);
	}
}

Heap::Heap() { clear(); }

//id[i]表示编号为i的元素在a中的位置，iid[i]表示a[i]的id（id:[0~n-1],a/iid:[1~n]）

void Heap::clear()
{
	n = 1;
	id.clear();
	iid.clear();
	a.clear();
	iid.push_back(0);
	a.push_back(0);
}

void Heap::swap_(int x, int y)//交换a,id,iid
{
	swap(a[x], a[y]);
	id[iid[x]] = y;
	id[iid[y]] = x;
	swap(iid[x], iid[y]);
}

void Heap::up(int x)//向上调整a[x]至合适位置
{
	while (x>1)
	{
		if (a[x >> 1]<a[x]) { swap_(x >> 1, x); x >>= 1; }
		else return;
	}
}

void Heap::down(int x)//向上调整a[x]至合适位置
{
	while ((x << 1)<n)
	{
		int k;
		if ((x << 1) + 1 >= n || a[x << 1]>a[(x << 1) + 1])k = x << 1;
		else k = (x << 1) + 1;
		if (a[x]<a[k]) { swap_(x, k); x = k; }
		else return;
	}
}

int Heap::top() { return a[1]; }

int Heap::top_id() { return iid[1]; }

int Heap::size() { return n; }

void Heap::change(int x, int num)//将编号x的点改为num
{
	a[id[x]] = num;
	up(id[x]);
	down(id[x]);
}

void Heap::add(int x, int num)//向编号x的点加num
{
	a[id[x]] += num;
	up(id[x]);
	down(id[x]);
}

void Heap::push(int num)//压入一个数值为num的元素
{
	id.push_back(n);
	iid.push_back(n - 1);
	a.push_back(num);
	n++;
	up(n - 1);
}

void Heap::draw()
{
	printf("Heap:%d n=%d\n", this, n - 1);
	printf("a:"); for (int i = 1; i<n; i++)printf(" %d", a[i]); cout << endl;
	printf("id:"); for (int i = 1; i<n - 1; i++)printf(" %d", id[i]); cout << endl;
	printf("iid:"); for (int i = 1; i<n; i++)printf(" %d", iid[i]); cout << endl;
	printf("draw_end\n");
}

//Adjacency list

Graph::Graph() { clear(); }

Graph::~Graph() { clear(); }

void Graph::save()//保存结构信息(stdout输出)
{
	printf("%d %d %d\n", n, m, tot);
	save_vector(id);
	save_vector(head);
	save_vector(list);
	save_vector(next);
	save_vector(cost);
}

void Graph::load()//读取结构信息(stdout输出)
{
	scanf("%d%d%d", &n, &m, &tot);
	load_vector(id);
	load_vector(head);
	load_vector(list);
	load_vector(next);
	load_vector(cost);
}

void Graph::add_D(int a, int b, int c)//Add a directed edge with a->b weight c
{
	//tot: counter initialized to 1
	tot++;
	//list: one entry per edge
	list[tot] = b;
	//cost: one entry per edge
	cost[tot] = c;
	//next: one entry per edge
	next[tot] = head[a];
	//head: one entry per node. Indexed by the node # at start of edge, it 
	head[a] = tot;
}

void Graph::add(int a, int b, int c)//Add an undirected edge with a<->b weight c
{
	add_D(a, b, c);
	add_D(b, a, c);
}

void Graph::init(int N, int M, int t)
{
	clear();
	n = N; m = M;
	tot = t;
	head = vector<int>(N);
	id = vector<int>(N);
	list = vector<int>(M * 2 + 2);
	next = vector<int>(M * 2 + 2);
	cost = vector<int>(M * 2 + 2);
}

void Graph::clear()
{
	n = m = tot = 0;
	head.clear();
	list.clear();
	next.clear();
	cost.clear();
	id.clear();
}

void Graph::draw()//输出图结构
{
	printf("Graph:%d n=%d m=%d\n", this, n, m);
	for (int i = 0; i<n; i++)cout << id[i] << ' '; cout << endl;
	for (int i = 0; i<n; i++)
	{
		printf("%d:", i);
		for (int j = head[i]; j; j = next[j])printf(" %d", list[j]);
		cout << endl;
	}
	printf("Graph_draw_end\n");
}

//连通性

vector<int> Graph::Split(Graph* G[], int nparts)//将子图一分为二返回color数组，并将两部分分别存至G1，G2 METIS algorithm,npart表示划分块数
{

	vector<int>color(n);
	int i;
	/*if(n<Naive_Split_Limit)
	{
	return Split_Naive(*G[0],*G[1]);
	}*/

	if (DEBUG1)printf("Begin-Split\n");
	if (n == nparts)
	{
		for (i = 0; i<n; i++)color[i] = i;
	}
	else
	{
		idx_t options[METIS_NOPTIONS];
		memset(options, 0, sizeof(options));
		{
			METIS_SetDefaultOptions(options);
			options[METIS_OPTION_PTYPE] = METIS_PTYPE_KWAY; // _RB
			options[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT; // _VOL
			options[METIS_OPTION_CTYPE] = METIS_CTYPE_SHEM; // _RM
			options[METIS_OPTION_IPTYPE] = METIS_IPTYPE_RANDOM; // _GROW _EDGE _NODE
			options[METIS_OPTION_RTYPE] = METIS_RTYPE_FM; // _GREEDY _SEP2SIDED _SEP1SIDED
														  // options[METIS_OPTION_NCUTS] = 1;
														  // options[METIS_OPTION_NITER] = 10;
														  /* balance factor, used to be 500 */
			options[METIS_OPTION_UFACTOR] = 500;
			// options[METIS_OPTION_MINCONN];
			options[METIS_OPTION_CONTIG] = 1;
			// options[METIS_OPTION_SEED];
			options[METIS_OPTION_NUMBERING] = 0;
			// options[METIS_OPTION_DBGLVL] = 0;
		}
		idx_t nvtxs = n;
		idx_t ncon = 1;
		//transform
		int* xadj = new idx_t[n + 1];
		int* adj = new idx_t[n + 1];
		int* adjncy = new idx_t[tot - 1];
		int* adjwgt = new idx_t[tot - 1];
		int* part = new idx_t[n];


		int xadj_pos = 1;
		int xadj_accum = 0;
		int adjncy_pos = 0;

		// xadj, adjncy, adjwgt
		xadj[0] = 0;
		int i = 0;
		for (int i = 0; i<n; i++) {
			// init node map

			/*int fanout = Nodes[nid].adjnodes.size();
			for ( int j = 0; j < fanout; j++ ){
			int enid = Nodes[nid].adjnodes[j];
			// ensure edges within
			if ( nset.find( enid ) != nset.end() ){
			xadj_accum ++;
			adjncy[adjncy_pos] = enid;
			adjwgt[adjncy_pos] = Nodes[nid].adjweight[j];
			adjncy_pos ++;
			}
			}
			xadj[xadj_pos++] = xadj_accum;*/
			for (int j = head[i]; j; j = next[j])
			{
				int enid = list[j];
				xadj_accum++;
				adjncy[adjncy_pos] = enid;
				adjwgt[adjncy_pos] = cost[j];
				adjncy_pos++;
			}
			xadj[xadj_pos++] = xadj_accum;
		}


		// adjust nodes number started by 0

		// adjwgt -> 1
		for (int i = 0; i < adjncy_pos; i++) {
			adjwgt[i] = 1;
		}

		// nparts
		int objval = 0;
		//METIS
		METIS_PartGraphKway(
			&nvtxs,
			&ncon,
			xadj,
			adjncy,
			NULL,
			NULL,
			adjwgt,
			&nparts,
			NULL,
			NULL,
			options,
			&objval,
			part
		);
		for (int i = 0; i<n; i++)color[i] = part[i];
		delete[] xadj;
		delete[] adj;
		delete[] adjncy;
		delete[] adjwgt;
		delete[] part;
	}
	//划分
	int j;
	vector<int>new_id;
	vector<int>tot(nparts, 0), m(nparts, 0);
	for (i = 0; i<n; i++)
		new_id.push_back(tot[color[i]]++);
	for (i = 0; i<n; i++)
		for (j = head[i]; j; j = next[j])
			if (color[list[j]] == color[i])
				m[color[i]]++;
	for (int t = 0; t<nparts; t++)
	{
		(*G[t]).init(tot[t], m[t]);
		for (i = 0; i<n; i++)
			if (color[i] == t)
				for (j = head[i]; j; j = next[j])
					if (color[list[j]] == color[i])
						(*G[t]).add_D(new_id[i], new_id[list[j]], cost[j]);
	}
	for (i = 0; i<tot.size(); i++)tot[i] = 0;
	for (i = 0; i<n; i++)
		(*G[color[i]]).id[tot[color[i]]++] = id[i];
	if (DEBUG1)printf("Split_over\n");
	return color;
}

vector<int> Graph::Split_Naive(Graph& G1, Graph& G2)//将子图一分为二返回color数组，并将两部分分别存至G1，G2 Kernighan-Lin algorithm
{
	color.clear();
	con.clear();
	if (DEBUG1)printf("Begin Split\n");
	int i, j, k = n / 2, l;
	for (i = 0; i<n; i++)//初始随机染色
	{
		if (n - i == k/*||(k&&rand()&1)*/) { color.push_back(1); k--; }
		else color.push_back(0);
	}
	if (DEBUG1)printf("Split_Color over\n");

	for (i = 0; i<n; i++)
	{
		k = 0;
		for (j = head[i]; j; j = next[j])
			if (color[list[j]] ^ color[i])k++;
		con.push_back(k);
	}
	if (DEBUG1)printf("Split_Con over\n");

	//优先队列划分,每个点的分数邻域不同数量-相同数量
	Heap q[2]; int ans = 0;
	for (i = 0; i<n; i++)
	{
		k = 0;
		for (j = head[i]; j; j = next[j])
			if (color[list[j]] ^ color[i])ans++, k++;
			else k--;
		q[color[i]].push(k);
		q[color[i] ^ 1].push(-k - INF);
	}
	ans /= 2;
	if (DEBUG1)printf("Split_Build_Heap over\n");
	//for(i=0;i<n;i++)printf("%d %d %d\n",i,color[i],con[i]);
	if (DEBUG1)cout << "start_ans=" << ans << endl;
	while (q[0].top() + q[1].top()>0)
	{
		int save_ans = ans;
		for (l = 0; l<2; l++)
		{
			i = q[l].top_id();
			k = 0;
			for (j = head[i]; j; j = next[j])
			{
				if (color[list[j]] ^ color[i])k--;
				else k++;
				q[color[i] ^ 1].add(list[j], -2);
				q[color[i]].add(list[j], 2);
			}
			ans += k;
			color[i] ^= 1;
			q[color[i]].change(i, k);
			q[color[i] ^ 1].change(i, -k - INF);
		}
		if (DEBUG1)if (rand() % 100 == 0)printf("now_ans=%d\n", save_ans);
		if (save_ans == ans)break;
	}
	//DEBUG
	if (DEBUG1)
	{
		int border_num = 0;
		for (i = 0; i<n; i++)
			for (j = head[i]; j; j = next[j])
				if (color[i] != color[list[j]]) { border_num++; break; }
		printf("ans=%d border_number=%d\n", ans, border_num);
	}
	//for(i=0;i<n;i++)cout<<"i="<<i<<" color="<<color[i]<<endl;

	//划分
	vector<int>new_id;
	int tot0 = 0, tot1 = 0;
	int m1 = 0, m0 = 0;
	for (i = 0; i<n; i++)
	{
		if (color[i] == 0)new_id.push_back(tot0++);
		else new_id.push_back(tot1++);
	}
	for (i = 0; i<n; i++)
		for (j = head[i]; j; j = next[j])
			if (1 ^ color[list[j]] ^ color[i])
			{
				if (color[i] == 0)m0++;
				else m1++;
			}
	G1.init(tot0, m0);
	for (i = 0; i<n; i++)
		if (color[i] == 0)
			for (j = head[i]; j; j = next[j])
				if (color[list[j]] == color[i])
					G1.add_D(new_id[i], new_id[list[j]], cost[j]);
	G2.init(tot1, m1);
	for (i = 0; i<n; i++)
		if (color[i] == 1)
			for (j = head[i]; j; j = next[j])
				if (color[list[j]] == color[i])
					G2.add_D(new_id[i], new_id[list[j]], cost[j]);
	tot0 = tot1 = 0;
	for (i = 0; i<n; i++)
	{
		if (color[i] == 0)G1.id[tot0++] = id[i];
		else G2.id[tot1++] = id[i];
	}
	if (DEBUG1)printf("Split_over\n");
	return color;
}

int Graph::Split_Borders(int nparts)//The number of borders that will be generated after dividing the graph into nparts blocks

{
	if (n<Naive_Split_Limit)
	{
		return Naive_Split_Limit;
	}
	idx_t options[METIS_NOPTIONS];
	memset(options, 0, sizeof(options));
	{
		METIS_SetDefaultOptions(options);
		options[METIS_OPTION_PTYPE] = METIS_PTYPE_KWAY; // _RB
		options[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT; // _VOL
		options[METIS_OPTION_CTYPE] = METIS_CTYPE_SHEM; // _RM
		options[METIS_OPTION_IPTYPE] = METIS_IPTYPE_RANDOM; // _GROW _EDGE _NODE
		options[METIS_OPTION_RTYPE] = METIS_RTYPE_FM; // _GREEDY _SEP2SIDED _SEP1SIDED
													  // options[METIS_OPTION_NCUTS] = 1;
													  // options[METIS_OPTION_NITER] = 10;
													  /* balance factor, used to be 500 */
		options[METIS_OPTION_UFACTOR] = 500;
		// options[METIS_OPTION_MINCONN];
		options[METIS_OPTION_CONTIG] = 1;
		// options[METIS_OPTION_SEED];
		options[METIS_OPTION_NUMBERING] = 0;
		// options[METIS_OPTION_DBGLVL] = 0;
	}
	idx_t nvtxs = n;
	idx_t ncon = 1;
	vector<int>color(n);
	int* xadj = new idx_t[n + 1];
	int* adj = new idx_t[n + 1];
	int* adjncy = new idx_t[tot - 1];
	int* adjwgt = new idx_t[tot - 1];
	int* part = new idx_t[n];


	int xadj_pos = 1;
	int xadj_accum = 0;
	int adjncy_pos = 0;


	xadj[0] = 0;
	int i = 0;
	for (int i = 0; i<n; i++) {
		for (int j = head[i]; j; j = next[j])
		{
			int enid = list[j];
			xadj_accum++;
			adjncy[adjncy_pos] = enid;
			adjwgt[adjncy_pos] = cost[j];
			adjncy_pos++;
		}
		xadj[xadj_pos++] = xadj_accum;
	}
	for (int i = 0; i < adjncy_pos; i++) {
		adjwgt[i] = 1;
	}
	int objval = 0;
	METIS_PartGraphKway(
		&nvtxs,
		&ncon,
		xadj,
		adjncy,
		NULL,
		NULL,
		adjwgt,
		&nparts,
		NULL,
		NULL,
		options,
		&objval,
		part
	);
	for (int i = 0; i<n; i++)color[i] = part[i];
	//divide
	int j, re = 0;
	for (i = 0; i<n; i++)
		for (j = head[i]; j; j = next[j])
			if (color[i] != color[list[j]])
			{
				re++;
				break;
			}
	delete[] xadj;
	delete[] adj;
	delete[] adjncy;
	delete[] adjwgt;
	delete[] part;
	return re;
}

//重载priority_queue的比较函数

void Graph::dijkstra(int S, vector<int>& dist)//依据本图计算以S为起点的全局最短路将结果存入dist
{
	priority_queue<state, vector<state>, cmp>q;
	state now;
	int i;
	dist.clear();
	while ((int)dist.size()<n)dist.push_back(INF);
	q.push(state(S, 0));
	while (q.size())
	{
		now = q.top();
		q.pop();
		if (dist[now.id] == INF)
		{
			dist[now.id] = now.len;
			for (i = head[now.id]; i; i = next[i])
				if (dist[list[i]] == INF)q.push(state(list[i], dist[now.id] + cost[i]));
		}
	}
}

vector<int> Graph::KNN(int S, int K, vector<int> T)//暴力dijkstra计算S到T数组中的前K小并返回其在T数组中的下标
{
	int i;
	vector<int>dist(n, INF), Cnt(n, 0);
	for (i = 0; i<T.size(); i++)Cnt[T[i]]++;
	priority_queue<state, vector<state>, cmp>q;
	state now;
	q.push(state(S, 0));
	int bound, cnt = 0;
	while (q.size() && cnt<K)
	{
		now = q.top();
		q.pop();
		if (dist[now.id] == INF)
		{
			dist[now.id] = now.len;
			cnt += Cnt[now.id];
			if (cnt >= K)bound = now.len;
			for (i = head[now.id]; i; i = next[i])
				if (dist[list[i]] == INF)q.push(state(list[i], dist[now.id] + cost[i]));
		}
	}
	vector<int>re;
	for (int i = 0; i<T.size() && re.size()<K; i++)
		if (dist[T[i]] <= bound)
			re.push_back(i);
	return re;
}

vector<int> Graph::find_path(int S, int T)//依据本图计算以S为起点的全局最短路将结果存入dist
{
	vector<int>dist, re, last;
	priority_queue<state, vector<state>, cmp>q;
	state now;
	int i;
	dist.clear();
	last.clear();
	re.clear();
	while ((int)dist.size()<n)
	{
		dist.push_back(INF);
		last.push_back(0);
	}
	q.push(state(S, 0));
	while (q.size())
	{
		now = q.top();
		q.pop();
		if (dist[now.id] == INF)
		{
			dist[now.id] = now.len;
			for (i = head[now.id]; i; i = next[i])
			{
				if (dist[list[i]] == INF)q.push(state(list[i], dist[now.id] + cost[i]));
				if (dist[list[i]] + cost[i] == dist[now.id])last[now.id] = list[i];
			}
		}
	}
	if (dist[T] == INF)return re;
	else
	{
		for (i = T; i != S; i = last[i])re.push_back(i);
		re.push_back(S);
		reverse(re.begin(), re.end());
		return re;
	}
}

int Graph::real_node()
{
	int ans = 0;
	for (int i = 0; i<n; i++)
	{
		int k = 0;
		for (int j = head[i]; j; j = next[j])k++;
		if (k != 2)ans++;
	}
	return ans;
}

void Graph::KNN_init(const vector<int>& S, int K)
{
	priority_queue<state, vector<state>, cmp>q;
	state now;
	int i;
	vector<int>empty;
	K_Near_Dist.clear();
	K_Near_Order.clear();
	while ((int)K_Near_Dist.size()<n)
	{
		K_Near_Dist.push_back(empty);
		K_Near_Order.push_back(empty);
	}
	for (int i = 0; i<S.size(); i++)
		q.push(state(S[i], 0, i));
	while (q.size())
	{
		now = q.top();
		q.pop();
		if (K_Near_Dist[now.id].size()<K)
		{
			K_Near_Dist[now.id].push_back(now.len);
			K_Near_Order[now.id].push_back(now.index);
			for (i = head[now.id]; i; i = next[i])
				if (K_Near_Dist[list[i]].size()<K)q.push(state(list[i], now.len + cost[i]));
		}
	}
}

vector<int>* Graph::KNN_Dijkstra(int S) { return &K_Near_Order[S]; }

void G_Tree::save()
{
	G.save();
	printf("%d %d %d\n", root, node_tot, node_size);
	save_vector(id_in_node);
	save_vector_vector(car_in_node);
	save_vector(car_offset);
	for (int i = 0; i<node_size; i++)
	{
		printf("\n");
		node[i].save();
	}
}

void G_Tree::load()
{
	printf("load_begin\n");
	G.load();
	scanf("%d%d%d", &root, &node_tot, &node_size);
	load_vector(id_in_node);
	load_vector_vector(car_in_node);
	load_vector(car_offset);
	node = new Node[G.n * 2 + 2];
	for (int i = 0; i<node_size; i++)
	{
		node[i].load();
	}
}

void G_Tree::write()
{
	printf("root=%d node_tot=%d\n", root, node_tot);
	for (int i = 1; i<node_tot; i++)
	{
		printf("node:%d\n", i);
		node[i].write();
		cout << endl;
	}
}

void G_Tree::add_border(int x, int id, int id2)//向x点的border集合中加入一个新真实id,在子图的虚拟id为id2,并对其标号为border中的编号
{
	node[x].borders.try_emplace(id,pair<int,int>((int)node[x].borders.size(), id2));
}

void G_Tree::make_border(int x, const vector<int>& color)//计算点x的border集合，二部图之间的边集为E
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

int G_Tree::partition_root(int x)//Returns the maximum number of blocks that the node can be divided into within the limit of Additional_Memory
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

void G_Tree::build(int x, int f, const Graph& g)//Recursive tree construction, current node x, leaf size f, subgraph g of current node
{
	if (x == 1)//x root
	{
		node = new Node[G.n * 2 + 2];
		node_size = G.n * 2;
		node_tot = 2;
		root = 1;
		node[x].deep = 1;
		node[1].G = g;
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
		rootp = node[x].part;
	}
	else if (node[x].G.n<Naive_Split_Limit)node[x].init(node[x].n);
	else node[x].init(Partition_Part);
	if (node[x].n>50)printf("x=%d deep=%d n=%d ", x, node[x].deep, node[x].G.n);
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
		if (node[x].n>50)printf("border=%d\n", node[x].borders.size());
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
	node[x].order.init(node[x].borders.size());
	node[x].order.cover(-INF);
	if (x == 1)//x build for root dist
	{
		for (int i = 1; i<min(1000, node_tot - 1); i++)
			if (node[i].n>50)
			{
				printf("x=%d deep=%d n=%d ", i, node[i].deep, node[i].G.n);
				printf("border=%d real_border=%d\n", node[i].borders.size(), real_border_number(i));
			}
		printf("begin_build_border_in_father_son\n");
		build_border_in_father_son();
		printf("begin_build_dist\n");
		build_dist1(root);
		printf("begin_build_dist2\n");
		build_dist2(root);
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
				node[i].min_car_dist.push_back(make_pair(INF, -1));
			}
		}
		{
			//Set up car_in_node;
			vector<int>empty_vector;
			empty_vector.clear();
			car_in_node.clear();
			for (int i = 0; i<G.n; i++)car_in_node.push_back(empty_vector);
		}
	}

}

void G_Tree::build_dist1(int x)//自下而上归并子图内部dist
{
	//计算子结点内部dist并传递给x
	for (int i = 0; i<node[x].part; i++)if (node[x].son[i])build_dist1(node[x].son[i]);
	if (node[x].son[0])//非叶子
	{
		//建立x子结点之间的边
		node[x].make_border_edge();
		//计算内部真实dist矩阵
		node[x].dist.floyd(node[x].order);
	}
	else;//叶子
		 //向父节点传递内部边权
	if (node[x].father)
	{
		int y = node[x].father, i, j;
		vector<int>id_in_fa(node[x].borders.size());
		//计算子图border在父节点border序列中的编号,不存在为-1
		for (auto x_iter1 = node[x].borders.begin(); x_iter1 != node[x].borders.end(); x_iter1++)
		{
			auto y_iter1 = node[y].borders.find(x_iter1->first);
			if (y_iter1 == node[y].borders.end())id_in_fa[x_iter1->second.first] = -1;
			else id_in_fa[x_iter1->second.first] = y_iter1->second.first;
		}
		//将子图内部的全连接边权传递给父亲
		for (i = 0; i<(int)node[x].borders.size(); i++)
			for (j = 0; j<(int)node[x].borders.size(); j++)
				if (id_in_fa[i] != -1 && id_in_fa[j] != -1)
				{
					int* p = &node[y].dist.a[id_in_fa[i]][id_in_fa[j]];
					if ((*p)>node[x].dist.a[i][j])
					{
						(*p) = node[x].dist.a[i][j];
						node[y].order.a[id_in_fa[i]][id_in_fa[j]] = -3;
					}
				}
	}
	return;
}

void G_Tree::build_dist2(int x)//自上而下修正子图外部dist
{
	if (x != root)node[x].dist.floyd(node[x].order);
	if (node[x].son[0])
	{
		//计算此节点border编号在子图中border序列的编号
		vector<int>id_(node[x].borders.size());
		vector<int>color_(node[x].borders.size());
		for (auto iter1 = node[x].borders.begin(); iter1 != node[x].borders.end(); iter1++)
		{
			int c = node[x].color[iter1->second.second];
			color_[iter1->second.first] = c;
			int y = node[x].son[c];
			id_[iter1->second.first] = node[y].borders[iter1->first].first;
		}
		//修正子图边权
		for (int i = 0; i<(int)node[x].borders.size(); i++)
			for (int j = 0; j<(int)node[x].borders.size(); j++)
				if (color_[i] == color_[j])
				{
					int y = node[x].son[color_[i]];
					int* p = &node[y].dist.a[id_[i]][id_[j]];
					if ((*p)>node[x].dist.a[i][j])
					{
						(*p) = node[x].dist.a[i][j];
						node[y].order.a[id_[i]][id_[j]] = -2;
					}
				}
		//递归子节点
		for (int i = 0; i<node[x].part; i++)
			if (node[x].son[i])build_dist2(node[x].son[i]);
	}
}

void G_Tree::build_border_in_father_son()//计算每个结点border在父亲/儿子borders数组中的编号
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

void G_Tree::push_borders_up(int x, vector<int>& dist1, int type)//将S到结点x边界点的最短路长度记录在dist1中，计算S到x.father真实border的距离更新dist1 type==0上推,type==1下推
{
	if (node[x].father == 0)return;
	//times[5] -= clock();

	int y = node[x].father;
	vector<int>dist2(node[y].borders.size(), INF);
	for (int i = 0; i<node[x].borders.size(); i++)
		if (node[x].border_in_father[i] != -1)
			dist2[node[x].border_in_father[i]] = dist1[i];
	//printf("dist2:");save_vector(dist2);
	int** dist = node[y].dist.a;
	//vector<int>begin,end;//已算出的序列编号,未算出的序列编号
	int* begin, * end;
	begin = new int[node[x].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	//times[5] += clock();
	//times[6] -= clock();
	for (int i = 0; i<dist2.size(); i++)
	{
		if (dist2[i]<INF)begin[tot0++] = i;
		else if (node[y].border_in_father[i] != -1)end[tot1++] = i;
	}
	//times[6] += clock();
	//times[7] -= clock();
	if (type == 0)
	{
		for (int i = 0; i<tot0; i++)
		{
			int i_ = begin[i];
			for (int j = 0; j<tot1; j++)
			{
				int j_ = end[j];
				if (dist2[j_]>dist2[i_] + dist[i_][j_])
					dist2[j_] = dist2[i_] + dist[i_][j_];
			}
		}
	}
	else {
		for (int i = 0; i<tot0; i++)
		{
			int i_ = begin[i];
			for (int j = 0; j<tot1; j++)
			{
				int j_ = end[j];
				if (dist2[j_]>dist2[i_] + dist[j_][i_])
					dist2[j_] = dist2[i_] + dist[j_][i_];
			}
		}
	}

	//times[7] += clock();
	dist1 = dist2;
	delete[] begin;
	delete[] end;
}

void G_Tree::push_borders_up_cache(int x, int bound)//将S到结点x边界点缓存在x.cache_dist的最短路长度，计算S到x.father真实border的距离更新x.father.cache
{
	if (node[x].father == 0)return;
	int y = node[x].father;
	if (node[x].cache_id == node[y].cache_id && bound <= node[y].cache_bound)return;
	node[y].cache_id = node[x].cache_id;
	node[y].cache_bound = bound;
	vector<int>* dist1 = &node[x].cache_dist, * dist2 = &node[y].cache_dist;
	for (int i = 0; i<(*dist2).size(); i++)(*dist2)[i] = INF;
	for (int i = 0; i<node[x].borders.size(); i++)
		if (node[x].border_in_father[i] != -1)
		{
			if (node[x].cache_dist[i]<bound)//bound界内的begin
				(*dist2)[node[x].border_in_father[i]] = (*dist1)[i];
			else (*dist2)[node[x].border_in_father[i]] = -1;//bound界外的begin
		}
	int** dist = node[y].dist.a;
	int* begin, * end;//已算出的序列编号,未算出的序列编号
	begin = new int[node[x].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	for (int i = 0; i<(*dist2).size(); i++)
	{
		if ((*dist2)[i] == -1)(*dist2)[i] = INF;
		else if ((*dist2)[i]<INF)begin[tot0++] = i;
		else if (node[y].border_in_father[i] != -1)
		{
			if (Optimization_Euclidean_Cut == false || Euclidean_Dist(node[x].cache_id, node[y].border_id[i])<bound)
				end[tot1++] = i;
		}
	}
	for (int i = 0; i<tot0; i++)
	{
		int i_ = begin[i];
		for (int j = 0; j<tot1; j++)
		{
			if ((*dist2)[end[j]]>(*dist2)[i_] + dist[i_][end[j]])
				(*dist2)[end[j]] = (*dist2)[i_] + dist[i_][end[j]];
		}
	}
	delete[] begin;
	delete[] end;
	node[y].min_border_dist = INF;
	for (int i = 0; i<node[y].cache_dist.size(); i++)
		if (node[y].border_in_father[i] != -1)
			node[y].min_border_dist = min(node[y].min_border_dist, node[y].cache_dist[i]);
}

void G_Tree::push_borders_down_cache(int x, int y, int bound)//将S到结点x边界点缓存在x.cache_dist的最短路长度，计算S到x的儿子y真实border的距离更新y.cache
{
	if (node[x].cache_id == node[y].cache_id && bound <= node[y].cache_bound)return;
	node[y].cache_id = node[x].cache_id;
	node[y].cache_bound = bound;
	vector<int>* dist1 = &node[x].cache_dist, * dist2 = &node[y].cache_dist;
	for (int i = 0; i<(*dist2).size(); i++)(*dist2)[i] = INF;
	for (int i = 0; i<node[x].borders.size(); i++)
		if (node[x].son[node[x].color[node[x].border_id_innode[i]]] == y)
		{
			if (node[x].cache_dist[i]<bound)//bound界内的begin
				(*dist2)[node[x].border_in_son[i]] = (*dist1)[i];
			else (*dist2)[node[x].border_in_son[i]] = -1;//bound界外的begin
		}
	int** dist = node[y].dist.a;
	int* begin, * end;//已算出的序列编号,未算出的序列编号
	begin = new int[node[y].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	for (int i = 0; i<(*dist2).size(); i++)
	{
		if ((*dist2)[i] == -1)(*dist2)[i] = INF;
		else if ((*dist2)[i]<INF)begin[tot0++] = i;
		else
		{
			if (Optimization_Euclidean_Cut == false || Euclidean_Dist(node[x].cache_id, node[y].border_id[i])<bound)
				end[tot1++] = i;
		}
	}
	for (int i = 0; i<tot0; i++)
	{
		int i_ = begin[i];
		for (int j = 0; j<tot1; j++)
		{
			if ((*dist2)[end[j]]>(*dist2)[i_] + dist[i_][end[j]])
				(*dist2)[end[j]] = (*dist2)[i_] + dist[i_][end[j]];
		}
	}
	delete[] begin;
	delete[] end;
	node[y].min_border_dist = INF;
	for (int i = 0; i<node[y].cache_dist.size(); i++)
		if (node[y].border_in_father[i] != -1)
			node[y].min_border_dist = min(node[y].min_border_dist, node[y].cache_dist[i]);
}

void G_Tree::push_borders_brother_cache(int x, int y, int bound)//将S到结点x边界点缓存在x.cache_dist的最短路长度，计算S到x的兄弟结点y真实border的距离更新y.cache
{
	int S = node[x].cache_id, LCA = node[x].father, i, j;
	if (node[y].cache_id == S && node[y].cache_bound >= bound)return;
	int p;
	node[y].cache_id = S;
	node[y].cache_bound = bound;
	vector<int>id_LCA[2], id_now[2];//子结点候选border在LCA中的border序列编号,子结点候选border在内部的border序列的编号
	for (int t = 0; t<2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		for (i = j = 0; i<(int)node[p].borders.size(); i++)
			if (node[p].border_in_father[i] != -1)
				if ((t == 1 && (Optimization_Euclidean_Cut == false || Euclidean_Dist(node[x].cache_id, node[p].border_id[i])<bound)) || (t == 0 && node[p].cache_dist[i]<bound))
				{
					id_LCA[t].push_back(node[p].border_in_father[i]);
					id_now[t].push_back(i);
				}
	}
	for (int i = 0; i<node[y].cache_dist.size(); i++)node[y].cache_dist[i] = INF;
	for (int i = 0; i<id_LCA[0].size(); i++)
		for (int j = 0; j<id_LCA[1].size(); j++)
		{
			int k = node[x].cache_dist[id_now[0][i]] + node[LCA].dist.a[id_LCA[0][i]][id_LCA[1][j]];
			if (k<node[y].cache_dist[id_now[1][j]])node[y].cache_dist[id_now[1][j]] = k;
		}
	int** dist = node[y].dist.a;
	//vector<int>begin,end;//已算出的序列编号,未算出的序列编号
	int* begin, * end;
	begin = new int[node[y].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	for (int i = 0; i<node[y].cache_dist.size(); i++)
	{
		if (node[y].cache_dist[i]<bound)begin[tot0++] = i;
		else if (node[y].cache_dist[i] == INF)
		{
			if (Optimization_Euclidean_Cut == false || Euclidean_Dist(node[x].cache_id, node[y].border_id[i])<bound)
				end[tot1++] = i;
		}
	}
	for (int i = 0; i<tot0; i++)
	{
		int i_ = begin[i];
		for (int j = 0; j<tot1; j++)
		{
			if (node[y].cache_dist[end[j]]>node[y].cache_dist[i_] + dist[i_][end[j]])
				node[y].cache_dist[end[j]] = node[y].cache_dist[i_] + dist[i_][end[j]];
		}
	}
	delete[] begin;
	delete[] end;
	node[y].min_border_dist = INF;
	for (int i = 0; i<node[y].cache_dist.size(); i++)
		if (node[y].border_in_father[i] != -1)
			node[y].min_border_dist = min(node[y].min_border_dist, node[y].cache_dist[i]);
}

void G_Tree::push_borders_up_path(int x, vector<int>& dist1)//将S到结点x边界点的最短路长度记录在dist1中，计算S到x.father真实border的距离更新dist1,并将到x.father的方案记录到x.father.path_record中(>=0表示结点，<0表示传递于那个结点儿子,-INF表示无前驱)
{
	if (node[x].father == 0)return;
	//times[5] -= clock();
	int y = node[x].father;
	vector<int>dist3(node[y].borders.size(), INF);
	vector<int>* order = &node[y].path_record;
	(*order).clear();
	for (int i = 0; i<node[y].borders.size(); i++)(*order).push_back(-INF);
	for (int i = 0; i<node[x].borders.size(); i++)
		if (node[x].border_in_father[i] != -1)
		{
			dist3[node[x].border_in_father[i]] = dist1[i];
			(*order)[node[x].border_in_father[i]] = -x;
		}
	//printf("dist3:");save_vector(dist3);
	int** dist = node[y].dist.a;
	//vector<int>begin,end;//已算出的序列编号,未算出的序列编号
	int* begin, * end;
	begin = new int[node[x].borders.size()];
	end = new int[node[y].borders.size()];
	int tot0 = 0, tot1 = 0;
	//times[5] += clock();
	//times[6] -= clock();
	for (int i = 0; i<dist3.size(); i++)
	{
		if (dist3[i]<INF)begin[tot0++] = i;
		else if (node[y].border_in_father[i] != -1)end[tot1++] = i;
	}
	//times[6] += clock();
	//times[7] -= clock();
	for (int i = 0; i<tot0; i++)
	{
		int i_ = begin[i];
		for (int j = 0; j<tot1; j++)
		{
			if (dist3[end[j]]>dist3[i_] + dist[i_][end[j]])
			{
				dist3[end[j]] = dist3[i_] + dist[i_][end[j]];
				(*order)[end[j]] = i_;
			}
		}
	}
	//times[7] += clock();
	dist1 = dist3;
	delete[] begin;
	delete[] end;
}

int G_Tree::find_LCA(int x, int y)//计算树上两节点xy的LCA
{
	if (node[x].deep<node[y].deep)swap(x, y);
	while (node[x].deep>node[y].deep)x = node[x].father;
	while (x != y) { x = node[x].father; y = node[y].father; }
	return x;
}

int G_Tree::search(int S, int T)//查询S-T最短路长度
{
	if (S == T)return 0;
	//计算LCA
	int i, j, k, p;
	int LCA, x = id_in_node[S], y = id_in_node[T];
	if (node[x].deep<node[y].deep)swap(x, y);
	while (node[x].deep>node[y].deep)x = node[x].father;
	while (x != y) { x = node[x].father; y = node[y].father; }
	LCA = x;
	vector<int>dist[2], dist_;
	dist[0].push_back(0);
	dist[1].push_back(0);
	x = id_in_node[S], y = id_in_node[T];
	//朴素G-Tree计算
	for (int t = 0; t<2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		while (node[p].father != LCA)
		{
			push_borders_up(p, dist[t], t);
			p = node[p].father;
		}
		if (t == 0)x = p;
		else y = p;
	}
	vector<int>id[2];//子结点border在LCA中的border序列编号
	for (int t = 0; t<2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		for (i = j = 0; i<(int)dist[t].size(); i++)
			if (node[p].border_in_father[i] != -1)
			{
				id[t].push_back(node[p].border_in_father[i]);
				dist[t][j] = dist[t][i];
				j++;
			}
		while (dist[t].size()>id[t].size()) { dist[t].pop_back(); }
	}
	//最终配对
	int MIN = INF;
	for (i = 0; i<dist[0].size(); i++)
	{
		int i_ = id[0][i];
		for (j = 0; j<dist[1].size(); j++)
		{
			k = dist[0][i] + dist[1][j] + node[LCA].dist.a[i_][id[1][j]];
			if (k<MIN)MIN = k;
		}
	}
	return MIN;
}

int G_Tree::search_cache(int S, int T, int bound)//查询S-T最短路长度,并将沿途结点的cache处理为S的结果，其中不计算权值>=bound的部分，若没有则剪枝返回INF
{
	//朴素G-Tree计算,维护cache
	if (S == T)return 0;
	//计算LCA
	int i, j, k, p;
	int x = id_in_node[S], y = id_in_node[T];
	int LCA = find_LCA(x, y);

	//计算两个叶子到LCA沿途结点编号
	vector<int>node_path[2];//点x/y到LCA之前依次会经过的树结点编号
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

	//将S从叶子push到LCA下层
	node[id_in_node[S]].cache_id = S;
	node[id_in_node[S]].cache_bound = bound;
	node[id_in_node[S]].min_border_dist = 0;
	node[id_in_node[S]].cache_dist[0] = 0;
	for (i = 0; i + 1<node_path[0].size(); i++)
	{
		if (node[node_path[0][i]].min_border_dist >= bound)return INF;
		#pragma omp critical (pushborders)
		push_borders_up_cache(node_path[0][i]);
	}

	//计算T在LCA下层结点的cache
	if (node[x].min_border_dist >= bound)return INF;
	#pragma omp critical (pushborders)
	push_borders_brother_cache(x, y);
	//将T在LCA下层的数据push到底层结点T
	for (int i = node_path[1].size() - 1; i>0; i--)
	{
		if (node[node_path[1][i]].min_border_dist >= bound)return INF;

		#pragma omp critical (pushborders)
		push_borders_down_cache(node_path[1][i], node_path[1][i - 1]);
	}

	//最终答案
	return node[id_in_node[T]].cache_dist[0];
}

int G_Tree::find_path(int S, int T, vector<int>& order)//返回S-T最短路长度，并将沿途经过的结点方案存储到order数组中
{
	order.clear();
	if (S == T)
	{
		order.push_back(S);
		return 0;
	}
	//计算LCA
	//times[0] -= clock();
	//times[4] -= clock();
	int i, j, k, p;
	int LCA, x = id_in_node[S], y = id_in_node[T];
	if (node[x].deep<node[y].deep)swap(x, y);
	while (node[x].deep>node[y].deep)x = node[x].father;
	while (x != y) { x = node[x].father; y = node[y].father; }
	LCA = x;
	vector<int>dist[2], dist_;
	dist[0].push_back(0);
	dist[1].push_back(0);
	x = id_in_node[S], y = id_in_node[T];
	//朴素G-Tree计算
	cnt_type1++;
	//printf("LCA=%d x=%d y=%d\n",LCA,x,y);
	//times[4] += clock();
	//times[2] -= clock();
	for (int t = 0; t<2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		while (node[p].father != LCA)
		{
			push_borders_up_path(p, dist[t]);
			p = node[p].father;
		}
		if (t == 0)x = p;
		else y = p;
	}
	//times[2] += clock();
	//times[3] -= clock();
	vector<int>id[2];//子结点border在LCA中的border序列编号
	for (int t = 0; t<2; t++)
	{
		if (t == 0)p = x;
		else p = y;
		for (i = j = 0; i<(int)dist[t].size(); i++)
			if (node[p].border_in_father[i] != -1)
			{
				id[t].push_back(node[p].border_in_father[i]);
				dist[t][j] = dist[t][i];
				j++;
			}
		while (dist[t].size()>id[t].size()) { dist[t].pop_back(); }
	}
	//times[3] += clock();
	//最终配对
	//times[1] -= clock();
	int MIN = INF;
	int S_ = -1, T_ = -1;//最优路径在LCA中borders连接的编号
	for (i = 0; i<(int)dist[0].size(); i++)
		for (j = 0; j<(int)dist[1].size(); j++)
		{
			k = dist[0][i] + dist[1][j] + node[LCA].dist.a[id[0][i]][id[1][j]];
			if (k<MIN)
			{
				MIN = k;
				S_ = id[0][i];
				T_ = id[1][j];
			}
		}
	if (MIN<INF)//存在路径，恢复路径
	{
		for (int t = 0; t<2; t++)
		{
			int p, now;
			if (t == 0)p = x, now = node[LCA].border_in_son[S_];
			else p = y, now = node[LCA].border_in_son[T_];
			while (node[p].n>1)
			{
				//printf("t=%d p=%d now=%d node[p].path_record[now]=%d\n",t,p,now,node[p].path_record[now]);
				if (node[p].path_record[now] >= 0)
				{
					find_path_border(p, now, node[p].path_record[now], order, 0);
					now = node[p].path_record[now];
				}
				else if (node[p].path_record[now]>-INF)
				{
					int temp = now;
					now = node[p].border_in_son[now];
					p = -node[p].path_record[temp];
				}
				else break;
			}
			if (t == 0)//翻转，补充连接路径
			{
				reverse(order.begin(), order.end());
				order.push_back(node[LCA].border_id[S_]);
				find_path_border(LCA, S_, T_, order, 0);
			}
		}
	}
	//times[1] += clock();
	//times[0] += clock();
	return MIN;
	//cout<<"QY5";
}

int G_Tree::real_border_number(int x)//计算x真实的border数(忽略内部子图之间的border)
{
	int i, j, re = 0, id;
	map<int, int>vis;
	for (i = 0; i<node[x].G.n; i++)vis[node[x].G.id[i]] = 1;
	for (i = 0; i<node[x].G.n; i++)
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

void G_Tree::find_path_border(int x, int S, int T, vector<int>& v, int rev)//返回结点x中编号为S到T的border的结点路径，存储在vector<int>中，将除了起点S以外的部分S+1~T，push到v尾部,rev=0表示正序，rev=1表示逆序
{
	/*printf("find:x=%d S=%d T=%d\n",x,S,T);
	printf("node:%d\n",x);
	node[x].write();
	printf("\n\n\n\n");*/
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
