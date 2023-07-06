#pragma once

#include "basic_typedef.h"
#include "RoadmapSolverTree.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "InstancesManager.h"

using namespace boost;

typedef		adjacency_list<vecS,vecS,undirectedS>	BGraph;
typedef		vector<int>								PredecessorMap;

class ExtendedConf
{

public:
	///////////////////////
	// CTORS
	////////////////////////
	ExtendedConf(ConfGraph cgraph, IDFactory* vertexCounter, bool basic);

	~ExtendedConf(void);

	////////////////////////
	// Queries
	////////////////////////

	ConfSet*	getConfSet() {return &m_vertexMap;}
	int			getNumOfConf(){return m_vertexMap.size();}
	ConfSet	getConfSubset(vector<int>* ind);
	
	Path		getPath(vector<int>* startConf, vector<int>* targetConf, vector<int>* robotsOrder);

	vector<int> generateRandomSubset(int robotNum);

	bool		isBasic(){ return m_isBasic; }

	int			getInstanceID(vector<int> vertices);

	pair<vector<pair<Conf,Conf>>,ConfSet> getGeometricGraph();
	
	////////////////////////
	// Helper
	////////////////////////
	Path		transformCCPathToPath(list<Edge>* ccPath, map<int,int>* confRobotMap);

	bool random_compare(int a, int b)
	{ 
		return rand() % 2; 
	}

private:
	BGraph					m_graph;
	vector<RoadmapSolver*>	m_solvers;
	ConfSet					m_vertexMap;
	vector<int>				m_cc;
	int						m_ccNum;
	vector<int>				m_predMap;
	vector<Conf>			m_robotPos;
	InstancesManager		m_manager;
	bool					m_isBasic;
	Edges					m_edges;

};

struct ExtendedConfEdge
{
	ExtendedConfEdge reverseDirection()
	{
		ExtendedConfEdge rev;
		rev.ex1 = ex2;
		rev.vert1 = vert2;
		rev.ex1 = ex2;
		rev.vert1 = vert2;

		return rev;
	}
	
	ExtendedConf* ex1;
	ExtendedConf* ex2;
	vector<int>	vert1, vert2;
};