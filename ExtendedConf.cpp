#include "ExtendedConf.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/config.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

class pred_recorder : public default_bfs_visitor
{
public:
	
	pred_recorder(vector<int>* pred) : m_p(pred) { }

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

pred_recorder record_pred(vector<int>* pred)
{
  return pred_recorder(pred);
}

///////////////////////
// CTORS
////////////////////////

ExtendedConf::ExtendedConf(ConfGraph cgraph, IDFactory* vertexCounter, bool basic) : 
	m_manager(vertexCounter), m_isBasic(basic)
{
	m_vertexMap = cgraph.first;

	if (!m_isBasic)
	{

		m_edges = cgraph.second;

		//	create an undirected unweighted graph
		m_graph = BGraph(m_edges.begin(), m_edges.end(), m_vertexMap.size());

		//	create connected component mapping of the graph's vertices
		m_cc = vector<int>(num_vertices(m_graph));
		m_ccNum = connected_components(m_graph, &m_cc[0]);

		vector<bool> is_cc_visited;
		for (int i = 0; i < m_ccNum; i++)
			is_cc_visited.push_back(false);

		//	for every cc update the predecessor map (represents a forest)
		m_predMap = vector<int>(num_vertices(m_graph));
		for(int vertID = 0; vertID < m_cc.size(); vertID++)
		{
			int cc = m_cc[vertID];
			if (is_cc_visited[cc])
				continue;

			is_cc_visited[cc] = true;
			breadth_first_search(m_graph, vertex(vertID,m_graph), 
				visitor(record_pred(&m_predMap)));
		}

		//	for every cc tree construct a roadmap solver
		std::vector<Edges> edgesOfCCTree(m_ccNum);

		for(int vertID = 0; vertID < m_cc.size(); vertID++)
		{
			int pred = m_predMap[vertID];
			if (pred == vertID)
				continue;

			Edge e(pred,vertID);

			int vertCC = m_cc[vertID];

			edgesOfCCTree[vertCC].push_back(e);
		} 

		for (int i = 0; i < m_ccNum; i++)
		{
			RoadmapSolver* rm = new RoadmapSolverTree(&edgesOfCCTree[i]);
			m_solvers.push_back(rm);
		}
	}
}



ExtendedConf::~ExtendedConf(void)
{
	if (m_isBasic)
		return; 

	foreach(RoadmapSolver* s, m_solvers)
	{
		//delete s;
	}
}

////////////////////////
// Queries
////////////////////////

Path		ExtendedConf::getPath(vector<int>* startConf, vector<int>* targetConf, 
	vector<int>* robotPosition)
{
	
	Path path;

	if (m_isBasic)
	{
		ConfSet cs;
		for (int r = 0; r < robotPosition->size(); r++)
		{
			cs.push_back(m_vertexMap[(*robotPosition)[r]]);
		}

		path.push_back(cs);
		return path;
	}

	vector<list<int>> startConfCC(m_ccNum), targetConfCC(m_ccNum);
	m_robotPos.clear();

	map<int,int> confsToRobotID;
	for (int r = 0; r < robotPosition->size(); r++)
	{
		confsToRobotID.insert(pair<int,int>((*robotPosition)[r],r));
		m_robotPos.push_back(m_vertexMap[(*robotPosition)[r]]);
	}

	path.push_back(m_robotPos);
	
	foreach(int s, *startConf)
	{
		int cc = m_cc[s];
		startConfCC[cc].push_back(s);
	}

	foreach(int t, *targetConf)
	{
		int cc = m_cc[t];
		targetConfCC[cc].push_back(t);
	}

	for(int i = 0; i < m_ccNum; i++)
	{
		if (startConfCC[i].empty())
			continue;

		list<Edge> pathForCC = m_solvers[i]->calculatePath(startConfCC[i], targetConfCC[i]);
		if (pathForCC.empty())
		{
			CGAL_assertion(startConfCC[i].size() == 1
				&& startConfCC[i].front() == targetConfCC[i].front());
			// KIRIL
			//pathForCC.push_back(Edge(startConfCC[i].front(),startConfCC[i].front()));
		}

		Path modCCPath = transformCCPathToPath(&pathForCC, &confsToRobotID);
		foreach(vector<Conf> v, modCCPath)
		{
			path.push_back(v);
		}
	}

	// update robot's positions
	for (int i = 0; i < targetConf->size(); i++)
	{
		int p = (*targetConf)[i];
		Conf c = m_vertexMap[p];

		for (int j = 0; j < m_robotPos.size(); j++)
		{
			Conf robConf = m_robotPos[j];
			if (robConf == c)
			{
				(*robotPosition)[j] = p;
				break;
			}
		}
	}

	return path;
}

vector<int> ExtendedConf::generateRandomSubset(int robotNum)
{
	int confNum = m_vertexMap.size();
	
	std::vector<int> permutation;
	for (int i = 0; i < confNum; i++)
	{
		permutation.push_back(i);
	}
	
	std::random_shuffle(permutation.begin(), permutation.end());
	
	vector<int> result;

	for (int i = 0; i < robotNum; i++)
	{
		result.push_back(permutation[i]);
	}
	
	return result;
}

int			ExtendedConf::getInstanceID(vector<int> vertices)
{
	vector<int> sig;
	if (!m_isBasic)
	{
		for (int i = 0; i < m_ccNum; i++)
			sig.push_back(0);

		foreach (int v, vertices)
		{
			int vcc = m_cc[v];
			sig[vcc]++;
		}
	}

	return m_manager.getInstanceID(sig);
}

ConfSet	ExtendedConf::getConfSubset(vector<int>* ind)
{
	ConfSet result;
	for (int i = 0; i < ind->size(); i++)
	{
		result.push_back(m_vertexMap[(*ind)[i]]);
	}

	return result;
}

pair<vector<pair<Conf,Conf>>,ConfSet> ExtendedConf::getGeometricGraph()
{
	vector<pair<Conf,Conf>> geoEdges;

	if (!m_isBasic)
	{	
		foreach(Edge e,m_edges)
		{
			geoEdges.push_back(pair<Conf,Conf>(m_vertexMap[e.first], m_vertexMap[e.second]));
		}
	}

	return pair<vector<pair<Conf,Conf>>,ConfSet>(geoEdges, m_vertexMap);
}

////////////////////////
// Helper
////////////////////////
Path		ExtendedConf::transformCCPathToPath(list<Edge>* ccPath, 
	map<int,int>* confRobotMap)
{
	Path path;
	
	foreach(Edge e, (*ccPath))
	{
		int s = e.first;
		int t = e.second;

		map<int,int>::iterator iter = confRobotMap->find(s);
		CGAL_precondition(iter != confRobotMap->end());

		int movedRobot = iter->second;
		confRobotMap->erase(s);
		confRobotMap->insert(pair<int,int>(t,movedRobot));

		m_robotPos[movedRobot] = m_vertexMap[t];
		path.push_back(m_robotPos);
	}

	return path;
}

