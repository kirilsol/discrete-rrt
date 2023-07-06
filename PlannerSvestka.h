#include "Scene.h"
#include <QPointF>
#include <QVector>
#include <QPolygonF>
#include "basic_typedef.h"
#include "CompositeConf.h"
#include "basic_typedef.h"
#include "Sampler.h"
#include "SimpleCDwObstacles.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/config.hpp>
//#include "Stats.h"

using namespace boost;
using namespace std;

typedef pair<double, int> PriorityTypeS;
typedef		adjacency_list<vecS,vecS,undirectedS>	BGraph;

class pred_recorder2 : public default_bfs_visitor
{
public:
	
	pred_recorder2(vector<int>* pred) : m_p(pred) { }

	// IMPORTANT: this template is MANDATORY!
	template <typename Edge, typename Graph>
	void tree_edge(Edge e, const Graph& g) const
	{
		typename graph_traits<Graph>::vertex_descriptor
			u = source(e,g),
			v = target(e,g);

		(*m_p)[v] = u;
	}
private:
	vector<int>* m_p;
};

pred_recorder2 record_pred2(vector<int>* pred)
{
  return pred_recorder2(pred);
}

class ComparePairsS {
public:
    bool operator()(PriorityTypeS& p1, PriorityTypeS&  p2)
    {
		return p1.first < p2.first;
    }
};

struct IndexEdgeKey
{
	bool operator<(const IndexEdgeKey& key) const
	{
		if ((i1 < key.i1) 
			|| (i1 == key.i1 && i2 < key.i2))
			return true;
		return false;
	}

	int i1;
	int i2;
};

template <class NewGraph, class Tag>
struct graph_copier 
  : public boost::base_visitor<graph_copier<NewGraph, Tag> >
{
  typedef Tag event_filter;

  graph_copier(NewGraph& graph) : new_g(graph) { }

  template <class Edge, class Graph>
  void operator()(Edge e, Graph& g) {
    boost::add_edge(boost::source(e, g), boost::target(e, g), new_g);
  }
private:
  NewGraph& new_g;
};

template <class NewGraph, class Tag>
inline graph_copier<NewGraph, Tag>
copy_graph(NewGraph& g, Tag) {
  return graph_copier<NewGraph, Tag>(g);
}

class PlannerSvestka
{
public:
	
	////////////////////////
	// C'tors & D'tors
	////////////////////////

	PlannerSvestka(SimpleCDwObstacles* collision, Sampler* sampler,
		vector<ConfSet> startConfSet, vector<ConfSet> targetConfSet, 
		vector<int> robotsNum, Stats* stats)
	{
		m_collision = collision;
		m_sampler = sampler;
		m_startConfSet = startConfSet;
		m_targetConfSet = targetConfSet;
		m_robotNum = robotsNum;
		m_totalRobotNum = 0;
		foreach(int r, m_robotNum)
			m_totalRobotNum = m_totalRobotNum + r;
		m_stats = stats;
		m_kClosest = m_stats->svestka_KNEAREST;
		m_numVerticesSingle = m_stats->svestka_NUM_VERTICES;
	}

	~PlannerSvestka(){}

	////////////////////////
	// modifiers
	////////////////////////

	vector<vector<pair<QTPath,GeometricGraph>>> run()
	{
		clock_t t_total=clock();
		// path section number - group - info
		vector<vector<pair<QTPath,GeometricGraph>>> result;
		
		vector<pair<Conf,Conf>> dummyPairVector;
		ConfSet		dummyCS;
		GeometricGraph dummyGG(dummyPairVector, dummyCS);
		
		clock_t t_single=clock();
		generateSingleRoadmap();
		clock_t t_end=clock();
		double e_single = difftime(t_end, t_single)/1000;

		clock_t t_STMulti=clock();
		generateSTMultiVertices();
		t_end=clock();
		double e_STMulti = difftime(t_end, t_STMulti)/1000;
		
		clock_t t_multi=clock();
		generateMultiVertices();
		t_end=clock();
		double e_multi = difftime(t_end, t_multi)/1000;

		clock_t t_edges=clock();
		generateMultiEdges();
		t_end=clock();
		double e_edges = difftime(t_end, t_edges)/1000;

		vector<ConfSet> inter_result;

		clock_t t_generate=clock();
		bool b_generate = !generateMultiRoadmap(&inter_result);
		t_end=clock();
		double e_generate = difftime(t_end, t_generate)/1000;
		double e_total = difftime(t_end, t_total)/1000;

		FILE* fOutput = fopen("svestka_time.txt", "w+");
		fprintf(fOutput, "\nBENCHMARK\n");
		fprintf(fOutput, "=========\n");
		fprintf(fOutput, "Single Roadmap: %f\n", e_single);
		fprintf(fOutput, "STVertices: %f\n", e_STMulti);
		fprintf(fOutput, "MultiVertices: %f\n", e_multi);
		fprintf(fOutput, "Edges: %f\n", e_edges);
		fprintf(fOutput, "Generate: %f\n", e_generate);
		fprintf(fOutput, "TOTAL: %f\n", e_total);
		fclose(fOutput);

		if (b_generate)
			return result;

		foreach(ConfSet cs, inter_result)
		{
			vector<pair<QTPath,GeometricGraph>> path_graph_section;
			vector<ConfSet> strip_cs = stripConfSet(cs);
			foreach(ConfSet group_cs, strip_cs)
			{
				list<ConfSet> dummyPath;
				dummyPath.push_back(group_cs);
				QVector<QVector<QPointF>> qt_path = transformToQT(dummyPath);
				pair<QTPath,GeometricGraph> p(qt_path, dummyGG);
				path_graph_section.push_back(p);
			}
			result.push_back(path_graph_section);
		}

		return result;
	}

private:
	
	void		generateSingleRoadmapOLD()
	{
		foreach(ConfSet cs, m_startConfSet)
			foreach(Conf c, cs)
				m_singleRoadmapVertices.push_back(c);

		foreach(ConfSet cs, m_targetConfSet)
			foreach(Conf c, cs)
				m_singleRoadmapVertices.push_back(c);

		for (int i = 0; i < m_numVerticesSingle; i++)
			m_singleRoadmapVertices.push_back(m_sampler->generateConf());

		for (int i = 0; i < m_singleRoadmapVertices.size(); i++)
		{
			priority_queue<PriorityTypeS, vector<PriorityTypeS>, ComparePairsS> pq;
			vector<int> edges;

			for (int j = i + 1; j < m_singleRoadmapVertices.size(); j++)
			{
				if (j == i) 
					continue;

				double dist = distance(m_singleRoadmapVertices[i], m_singleRoadmapVertices[j]);
				PriorityTypeS pt(dist, j);
				pq.push(pt);
			}
				

			int num = m_kClosest;
				
			while (!pq.empty() && num != 0)
			{
				PriorityTypeS pt = pq.top();
				int j = pt.second;
				pq.pop();
				num--;

				// try connect i with j
				bool valid_edge = m_collision->testSingleLocalPlanner(m_singleRoadmapVertices[i], 
					m_singleRoadmapVertices[j]);

				if	(valid_edge)
					edges.push_back(j);
			}

			m_singleRoadmapEdges.push_back(edges);
		}
	}

	void		generateSingleRoadmap()
	{
		foreach(ConfSet cs, m_startConfSet)
			foreach(Conf c, cs)
				m_singleRoadmapVertices.push_back(c);

		foreach(ConfSet cs, m_targetConfSet)
			foreach(Conf c, cs)
				m_singleRoadmapVertices.push_back(c);

		for (int i = 0; i < m_numVerticesSingle; i++)
			m_singleRoadmapVertices.push_back(m_sampler->generateConf());

		vector<pair<int,int>> fake_edges;

		for (int i = 0; i < m_singleRoadmapVertices.size() - 1; i++)
		{
			for (int j = i + 1; j < m_singleRoadmapVertices.size(); j++)
			{
				bool valid_edge = m_collision->testSingleLocalPlanner(m_singleRoadmapVertices[i], 
					m_singleRoadmapVertices[j]);

				if	(valid_edge)
					fake_edges.push_back(pair<int,int>(i,j));
			}
		}

		// keep only edges from MST
		BGraph graph = BGraph(fake_edges.begin(), fake_edges.end(), m_singleRoadmapVertices.size());
		
		//	create connected component mapping of the graph's vertices
		vector<int> v_cc = vector<int>(num_vertices(graph));
		int ccNum = connected_components(graph, &v_cc[0]);

		vector<bool> is_cc_visited;
		for (int i = 0; i < ccNum; i++)
			is_cc_visited.push_back(false);

		//	for every cc update the predecessor map (represents a forest)
		vector<int> predMap = vector<int>(num_vertices(graph));
		for(int vertID = 0; vertID < v_cc.size(); vertID++)
		{
			int cc = v_cc[vertID];
			if (is_cc_visited[cc])
				continue;

			is_cc_visited[cc] = true;
			breadth_first_search(graph, vertex(vertID,graph), 
				visitor(record_pred2(&predMap)));
		}

		//	for every cc tree construct a roadmap solver
		std::vector<Edges> edgesOfCCTree(ccNum);

		for(int vertID = 0; vertID < v_cc.size(); vertID++)
		{
			int pred = predMap[vertID];
			if (pred == vertID)
				continue;

			Edge e(pred,vertID);

			int vertCC = v_cc[vertID];

			edgesOfCCTree[vertCC].push_back(e);
		} 

		for (int i = 0; i < m_singleRoadmapVertices.size(); i++)
		{
			vector<int> dummy;
			m_singleRoadmapEdges.push_back(dummy);
		}
		
		for (int i = 0; i < edgesOfCCTree.size(); i++)
		{
			foreach(Edge e, edgesOfCCTree[i])
			{
				m_singleRoadmapEdges[e.first].push_back(e.second);
				m_singleRoadmapEdges[e.second].push_back(e.first);
			}
		}
	}

	void	generateSTMultiVertices()
	{
		// start vertex
		m_startMultiVertex = 0;
		int total_robots = 0;
		foreach(int r, m_robotNum)
			total_robots = total_robots + r;
		vector<int> start;
		for (int i = 0; i < total_robots; i++)
		{
			start.push_back(i);
		}

		m_multiVertices.push_back(start);

		// target vertices
		vector<vector<int>> targetConfs; 
		for (int i = 0; i < m_robotNum.size(); i++)
		{
			vector<int> currConfs;
			for (int j = 0; j < m_robotNum[i]; j++)
			{
				currConfs.push_back(total_robots);
				total_robots++;
			}

			targetConfs.push_back(currConfs);
		}
		
		vector<vector<int>> targetConfsPermutations;
		createPermutations(&targetConfsPermutations, targetConfs);
		m_multiVertices.insert(m_multiVertices.end(), 
			targetConfsPermutations.begin(), targetConfsPermutations.end());
		m_targetMultiVertices=pair<int,int>(1, m_multiVertices.size() - 1);
	}

	void generateMultiVertices()
	{
		vector<vector<int>> container;
		allMultiVertexPermutations(&container);
		foreach(vector<int> vertex_candidate, container)
		{
			if (isMultiVertexValid(vertex_candidate))
				m_multiVertices.push_back(vertex_candidate);
		}

		for(int i = 0; i < m_multiVertices.size(); i++)
		{
			vector<int> mult_v = m_multiVertices[i];
			string sig = signature(&mult_v);
			m_vertexNameIDMap.insert(pair<string,int>(sig, i));
		}
	}

	void generateMultiEdgesOLD()
	{
		for (int i = 0; i < m_multiVertices.size(); i ++)
		{
			for (int j = i + 1; j < m_multiVertices.size(); j ++)
			{
				if (canConnectMultiVertices(i,j))
					m_multiEdges.push_back(pair<int,int>(i,j));
			}
		}
	}

	void generateMultiEdges()
	{
		for (int i = 0; i < m_multiVertices.size(); i ++)
		{
			vector<vector<int>> allNeighbors = findAllNeighbors(i);
			foreach(vector<int> neigh, allNeighbors)
			{
				string sig = signature(&neigh);
				pair<multimap<string, int>::iterator, multimap<string, int>::iterator> ppp;
				ppp = m_vertexNameIDMap.equal_range(sig);
				for (multimap<string, int>::iterator it2 = ppp.first;
					it2 != ppp.second; ++it2)
				{
					int neigh_ID = (*it2).second;
					if (canConnectMultiVertices(i,neigh_ID))
						m_multiEdges.push_back(pair<int,int>(i,neigh_ID));
				}
			}
		}
	}

	bool generateMultiRoadmap(vector<ConfSet>* p_result)
	{
		/* First verify that s and some t' are in the same connected component	*/
		BGraph cc_graph = BGraph(m_multiEdges.begin(), m_multiEdges.end(), 
			m_multiVertices.size());

		//	create connected component mapping of the graph's vertices
		vector<int> ccMap = vector<int>(num_vertices(cc_graph));
		int ccNum = connected_components(cc_graph, &ccMap[0]);

		int ccStart = ccMap[0];
		bool same_component = false;
		int targetID;
		for (int i = m_targetMultiVertices.first; i <= m_targetMultiVertices.second; i++)
		{
			if (ccMap[i] == ccStart)
			{
				targetID = i;
				same_component = true;
				break;
			}
		}

		// all target vertices in different components
		if (!same_component)
			return false;

		/************************************/
		/* find path from start to target	*/
		/************************************/
		typedef boost::adjacency_list< 
		boost::mapS, boost::vecS, boost::undirectedS,
		boost::property<boost::vertex_color_t, boost::default_color_type,
			boost::property<boost::vertex_degree_t, int,
			  boost::property<boost::vertex_in_degree_t, int,
		boost::property<boost::vertex_out_degree_t, int> > > >
	  > Graph;

		typedef graph_traits < Graph >::edge_descriptor edge_descriptor;
		typedef graph_traits < Graph >::vertex_descriptor Vertex;

		Graph g;
	
		/*	create a mapping between a pair <int,int> of edges to 
			the index of this edge	*/
		map<IndexEdgeKey, int> edgeMap; 
		for (int i = 0; i < m_multiEdges.size(); i++)
		{
			int sig1 = m_multiEdges[i].first;
			int sig2 = m_multiEdges[i].second;

			add_edge(sig1, sig2, g);

			IndexEdgeKey key, key2;
			key.i1 = sig1;
			key.i2 = sig2;

			key2.i1 = sig2;
			key2.i2 = sig1;

			edgeMap.insert(pair<IndexEdgeKey,int>(key, i));
			edgeMap.insert(pair<IndexEdgeKey,int>(key2, i));
		}

		int numVert = m_multiVertices.size();

		/*	create data structures used in the bfs algorithm
			as the predecessor map	*/
		vector<Vertex> p(numVert);
		vector<graph_traits<Graph>::vertices_size_type> d;
		for (int i = 0; i < numVert; i++)
			d.push_back(0);

		int startID = 0;
		//	start vertex
		Vertex s_v = vertex(startID, g);
		Graph G_copy(numVert);

		p[s_v] = s_v;

		/*	run bfs and update the predecessor map.
			p[v] = u, then u is the father of v in the bfs tree.
			if p[v] = 0, then v is in a different cc then the start node
			(or its father is the target vertex)	*/
		breadth_first_search (g, s_v, 
			visitor(make_bfs_visitor
					(std::make_pair(record_distances(&d[0], boost::on_tree_edge()),
										std::make_pair
											(boost::record_predecessors(&p[0], 
												boost::on_tree_edge()),
												copy_graph(G_copy, boost::on_examine_edge()))))));
	
		/*	extract the extConf path from the predecessor map	*/
		vector<int> vertexPath;
		int currVert = targetID;
		vertexPath.push_back(currVert);
		while (p[currVert] != 0)
		{
			currVert = p[currVert];
			vertexPath.push_back(currVert);
		}
		vertexPath.push_back(0);
		currVert = 0;

		p_result->clear();
		for(int i = vertexPath.size() - 1; i >= 0; i--)
		{
			ConfSet cs = getConfsFromMultiVertex(vertexPath[i]);
			p_result->push_back(cs);
		}

		return true;
	}

	////////////////////////
	// helpers
	////////////////////////

	vector<vector<int>>	findAllNeighbors(int id)
	{
		vector<int> vertex = m_multiVertices[id];
		vector<vector<int>> single_neighbors;
		foreach(int v, vertex)
		{
			single_neighbors.push_back(m_singleRoadmapEdges[v]);
		}

		vector<vector<int>> container;
		foreach(int n, single_neighbors[0])
		{
			vector<int> dummy;
			dummy.push_back(n);
			container.push_back(dummy);
		}

		for (int i = 1; i < single_neighbors.size(); i++)
		{
			vector<vector<int>> new_container;
			vector<int> curr_neighbors = single_neighbors[i];
			foreach(vector<int> cont, container)
			{
				foreach(int curr_n, curr_neighbors)
				{
					vector<int> new_vec(cont.begin(), cont.end());
					new_vec.push_back(curr_n);
					new_container.push_back(new_vec);
				}
			}
			container = new_container;
		}

		return container;
	}

	ConfSet getConfsFromMultiVertex(int id)
	{
		ConfSet cs;
		foreach(int i, m_multiVertices[id])
		{
			cs.push_back(m_singleRoadmapVertices[i]);
		}
		return cs;
	}

	void allPermutations(vector<vector<int>>* container, 
		vector<int> prefix, vector<int> suffix)
	{
		if (suffix.empty())
		{
			container->push_back(prefix);
			return;
		}

		for (int i = 0; i < suffix.size(); i++)
		{
			vector<int> new_suffix;
			int element = suffix[i];
			vector<int> new_prefix(prefix.begin(),prefix.end());
			new_prefix.push_back(element);
			
			// copy suffix without one element
			for (int j = 0; j < suffix.size(); j++)
			{
				if (j == i)
					continue;

				new_suffix.push_back(suffix[j]);
			}

			allPermutations(container, new_prefix, new_suffix);
		}
	}

	void allSuperPermutations(vector<vector<int>>* container, 
		vector<int> prefix, vector<vector<vector<int>>> suffix)
	{
		if (suffix.empty())
		{
			container->push_back(prefix);
			return;
		}

		vector<vector<int>> current_permutations = suffix[0];
		suffix.erase(suffix.begin());

		for (int i = 0; i < current_permutations.size(); i++)
		{
			vector<int> perm = current_permutations[i];
			vector<int> new_prefix(prefix.begin(),prefix.end());
			new_prefix.insert(new_prefix.end(),perm.begin(),perm.end());

			allSuperPermutations(container, new_prefix, suffix);
		}
	}

	void createPermutations(vector<vector<int>>* container, 
		vector<vector<int>> configuration_sets)
	{
		vector<vector<vector<int>>> suffix;
		vector<int> dummyVec;

		foreach(vector<int> v, configuration_sets)
		{
			vector<vector<int>> unabeled_permutation;
			

			allPermutations(&unabeled_permutation, dummyVec, v);
			suffix.push_back(unabeled_permutation);
		}

		allSuperPermutations(container, dummyVec, suffix);
	}

	void allMultiVertexPermutations(vector<vector<int>>* container,
		vector<int> prefix, int depth)
	{
		if (depth == 0)
		{
			container->push_back(prefix);
			return;
		}
		
		for (int i = 0; i < m_singleRoadmapVertices.size(); i++)
		{
			vector<int> new_prefix(prefix.begin(), prefix.end());
			new_prefix.push_back(i);
			allMultiVertexPermutations(container, new_prefix, depth - 1);
		}
	}

	void allMultiVertexPermutations(vector<vector<int>>* container)
	{
		int depth = m_totalRobotNum;
		vector<int> dummy;
		allMultiVertexPermutations(container, dummy,m_totalRobotNum);
	}

	bool canConnectMultiVertices(int a, int b)
	{
		vector<int> m_vert_a = m_multiVertices[a];
		vector<int> m_vert_b = m_multiVertices[b];

		for (int i = 0; i < m_vert_a.size(); i++)
		{
			if (!isConnected(m_vert_a[i],m_vert_b[i]))
				return false;
		}

		for (int i = 0; i < m_vert_a.size() - 1; i++)
		{
			for (int j = i + 1; j < m_vert_a.size(); j++)
			{
				ConfSet cs1,cs2;
				cs1.push_back(m_singleRoadmapVertices[m_vert_a[i]]);
				cs1.push_back(m_singleRoadmapVertices[m_vert_a[j]]);

				cs2.push_back(m_singleRoadmapVertices[m_vert_b[i]]);
				cs2.push_back(m_singleRoadmapVertices[m_vert_b[j]]);

				vector<int> sub;
				sub.push_back(0);
				sub.push_back(1);

				bool free = m_collision->testLocalPlannerNoObstacles(&cs1, &sub, &cs2, &sub);
				if (!free)
					return false;
			}
		}

		return true;
	}

	bool isMultiVertexValid(vector<int> v)
	{
		// first check that no element appears twice
		for(int i = 0; i < v.size() - 1; i++)
		{
			for (int j = i + 1; j < v.size(); j++)
			{
				if (v[i] == v[j])
					return false;
			}
		}

		ConfSet cs;
		foreach(int i, v)
			cs.push_back(m_singleRoadmapVertices[i]);
		
		return m_collision->testConfSet(&cs);
	}

	double		distance(Conf c1, Conf c2)
	{
		double x1 = c1.x() ;
		double y1 = c1.y() ;
		double x2 = c2.x() ;
		double y2 = c2.y() ;

		return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
	} 

	bool	isConnected(double i, double j)
	{
		foreach(int k, m_singleRoadmapEdges[i])
		{
			if (k == j)
				return true;
		}
		return false;
	}

	vector<ConfSet> stripConfSet(ConfSet cs)
	{
		vector<ConfSet> result;
		int robot_count = 0;
		for(int g = 0; g < m_robotNum.size(); g++)
		{
			ConfSet new_cs; 
			for (int r = 0; r < m_robotNum[g]; r++)
			{
				new_cs.push_back(cs[robot_count]);
				robot_count++;
			}
			result.push_back(new_cs);
		}
		return result;
	}

	string		signature(vector<int>	*vertex)
	{
		string str;
		foreach(int val, *vertex)
		{
			string curr = ts(val);
			str+=curr;
			str+="a";
		}
		return str;
	}

	Point_2		transformToCGAL(QPointF qp)
	{
		// x and y coordinates of the point
		double x = qp.x();
		double y = qp.y();

		Point_2 cpoint(x,y);
		return cpoint;
	}

	QPointF		transformToQT(Conf cp)
	{
		return QPointF(cp.x() , cp.y() );
	}

	QVector<QPointF> transformToQT(ConfSet cs)
	{
		QVector<QPointF> result;

		foreach(Conf c, cs)
		{
			result.push_back(transformToQT(c));
		}
		return result;
	}

	QVector<QVector<QPointF>> transformToQT(Path path)
	{
		QVector<QVector<QPointF>> result;

		foreach(ConfSet cs, path)
		{
			result.push_back(transformToQT(cs));
		}
		return result;
	}

	string ts(int number)
	{
	   stringstream ss;//create a stringstream
	   ss << number;//add number to the stream
	   return ss.str();//return a string with the contents of the stream
	}


	///////////////////////
	// Data members
	///////////////////////

	// input
	vector<int>			m_robotNum;
	int					m_totalRobotNum;
	vector<ConfSet>		m_startConfSet, m_targetConfSet;

	// components
	SimpleCDwObstacles*			m_collision;
	Sampler*					m_sampler;

	Stats*						m_stats;

	// Svestka & Overmars
	vector<Conf>				m_singleRoadmapVertices;	
	vector<vector<int>>			m_singleRoadmapEdges;		//Adjacency lists

	vector<vector<int>>			m_multiVertices;
	vector<pair<int,int>>		m_multiEdges;

	multimap<string, int>			m_vertexNameIDMap;

	int							m_startMultiVertex;
	pair<int,int>				m_targetMultiVertices;

	// parameters
	int							m_numVerticesSingle;
	int							m_kClosest;
};