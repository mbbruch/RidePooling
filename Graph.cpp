#include<iostream>
#include<cstring>
#include<algorithm>
#include<metis.h>
#include<math.h>
#include "Graph.h"
#include "util.h"
#include "globals.h"

const bool DEBUG1 = false;

bool Graph::cmp::operator()(const state& a, const state& b) {
	return a.len > b.len;
}

Graph::state::state(int a, int b, int c) :id(a), len(b), index(c) {};

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
	save_vector(cost2);
}

void Graph::load()//读取结构信息(stdout输出)
{
	scanf("%d%d%d", &n, &m, &tot);
	load_vector(id);
	load_vector(head);
	load_vector(list);
	load_vector(next);
	load_vector(cost);
	load_vector(cost2);
}

void Graph::add_D(int a, int b, float c, float d)//Add a directed edge with a->b weight c
{
	//tot: counter initialized to 1
	tot++;
	//list: one entry per edge
	list[tot] = b;
	//cost: one entry per edge
	cost[tot] = (int)c; //round(c);
	//cost2: one entry per edge
	cost2[tot] = round(d);
	//next: one entry per edge
	next[tot] = head[a];
	//head: one entry per node. Indexed by the node # at start of edge, it 
	head[a] = tot;
}

void Graph::add(int a, int b, float c, float d)//Add an undirected edge with a<->b weight c
{
	add_D(a, b, c, d);
	add_D(b, a, c, d);
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
	cost2 = vector<int>(M * 2 + 2);
}

void Graph::clear()
{
	n = m = tot = 0;
	head.clear();
	list.clear();
	next.clear();
	cost.clear();
	cost2.clear();
	id.clear();
}

void Graph::draw()//输出图结构
{
	//printf("Graph:%d n=%d m=%d\n", this, n, m);
	for (int i = 0; i < n; i++)cout << id[i] << ' '; cout << endl;
	for (int i = 0; i < n; i++)
	{
		printf("%d:", i);
		for (int j = head[i]; j; j = next[j])printf(" %d", list[j]);
		cout << endl;
	}
	printf("Graph_draw_end\n");
}

vector<int> Graph::Split(Graph* G[], int nparts)//将子图一分为二返回color数组，并将两部分分别存至G1，G2 METIS algorithm,npart表示划分块数
{
	vector<int>color(n);
	int i;
	if (DEBUG1)printf("Begin-Split\n");
	if (n == nparts)
	{
		for (i = 0; i < n; i++)color[i] = i;
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
		for (int i = 0; i < n; i++) {
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
		for (int i = 0; i < n; i++)color[i] = part[i];
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
	for (i = 0; i < n; i++)
		new_id.push_back(tot[color[i]]++);
	for (i = 0; i < n; i++)
		for (j = head[i]; j; j = next[j])
			if (color[list[j]] == color[i])
				m[color[i]]++;
	for (int t = 0; t < nparts; t++)
	{
		(*G[t]).init(tot[t], m[t]);
		for (i = 0; i < n; i++)
			if (color[i] == t)
				for (j = head[i]; j; j = next[j])
					if (color[list[j]] == color[i])
						(*G[t]).add_D(new_id[i], new_id[list[j]], cost[j], cost2[j]);
	}
	for (i = 0; i < tot.size(); i++)tot[i] = 0;
	for (i = 0; i < n; i++)
		(*G[color[i]]).id[tot[color[i]]++] = id[i];
	if (DEBUG1)printf("Split_over\n");
	return color;
}

int Graph::Split_Borders(int nparts)//The number of borders that will be generated after dividing the graph into nparts blocks

{
	if (n < Naive_Split_Limit)
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
	for (int i = 0; i < n; i++) {
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
	for (int i = 0; i < n; i++)color[i] = part[i];
	//divide
	int j, re = 0;
	for (i = 0; i < n; i++)
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

//According to this figure, 
//calculate the global shortest path starting from Sand store the result in dist
void Graph::dijkstra(int S, vector<int>& dist)//依据本图计算以S为起点的全局最短路将结果存入dist
{
	priority_queue<state, vector<state>, cmp>q;
	state now;
	int i;
	dist.clear();
	while ((int)dist.size() < n)dist.push_back(INF);
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
	for (i = 0; i < T.size(); i++)Cnt[T[i]]++;
	priority_queue<state, vector<state>, cmp>q;
	state now;
	q.push(state(S, 0));
	int bound, cnt = 0;
	while (q.size() && cnt < K)
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
	for (int i = 0; i < T.size() && re.size() < K; i++)
		if (dist[T[i]] <= bound)
			re.push_back(i);
	return re;
}

vector<int> Graph::find_path(int S, int T)
{
	vector<int>dist, re, last;
	priority_queue<state, vector<state>, cmp>q;
	state now;
	int i;
	dist.clear();
	last.clear();
	re.clear();
	while ((int)dist.size() < n)
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
	for (int i = 0; i < n; i++)
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
	while ((int)K_Near_Dist.size() < n)
	{
		K_Near_Dist.push_back(empty);
		K_Near_Order.push_back(empty);
	}
	for (int i = 0; i < S.size(); i++)
		q.push(state(S[i], 0, i));
	while (q.size())
	{
		now = q.top();
		q.pop();
		if (K_Near_Dist[now.id].size() < K)
		{
			K_Near_Dist[now.id].push_back(now.len);
			K_Near_Order[now.id].push_back(now.index);
			for (i = head[now.id]; i; i = next[i])
				if (K_Near_Dist[list[i]].size() < K)q.push(state(list[i], now.len + cost[i]));
		}
	}
}

vector<int>* Graph::KNN_Dijkstra(int S) { return &K_Near_Order[S]; }