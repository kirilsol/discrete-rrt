/*******************************************************************
*	File name: 		Planner.h
*	Description:	Implementation of the Planner class
*					This class implements the motion planning algorithm
*					and communicates with a Scene object to get the input
*					from the gui and return the result.
*
*	Authors:		Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "Planner.h"
#include "BrutForceConnector.h"
#include "HausdorffConnector.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <time.h>
#include <cstdlib>
#include <ctime>
#include <PlannerSvestka.h>

#include <iostream>
#include <fstream>
using namespace std;

//struct IndexEdgeKey
//{
//	bool operator<(const IndexEdgeKey& key) const
//	{
//		if ((i1 < key.i1) 
//			|| (i1 == key.i1 && i2 < key.i2))
//			return true;
//		return false;
//	}
//
//	int i1;
//	int i2;
//};
//
//template <class NewGraph, class Tag>
//struct graph_copier 
//  : public boost::base_visitor<graph_copier<NewGraph, Tag> >
//{
//  typedef Tag event_filter;
//
//  graph_copier(NewGraph& graph) : new_g(graph) { }
//
//  template <class Edge, class Graph>
//  void operator()(Edge e, Graph& g) {
//    boost::add_edge(boost::source(e, g), boost::target(e, g), new_g);
//  }
//private:
//  NewGraph& new_g;
//};
//
//template <class NewGraph, class Tag>
//inline graph_copier<NewGraph, Tag>
//copy_graph(NewGraph& g, Tag) {
//  return graph_copier<NewGraph, Tag>(g);
//}

Planner::Planner(Scene* scene)
{
	m_stats = Stats();
	m_isBasic = m_stats.IS_BASIC;

	// transform data from Scene
	m_scene = scene;
	m_radius = m_scene->getRadius();

	QVector<QPolygonF> obstacles = m_scene->getObstacles();
	foreach(QPolygonF qp, obstacles)
	{
		Polygon_2 cp;
		for (int j=0; j < qp.size(); j++)
		{
			cp.push_back(transformToCGAL(qp[j]));
		}

		m_obstacles.push_back(cp);
	}
	
	QVector<QVector<QPointF>> startPositions = m_scene->getStartPositions();
	QVector<QVector<QPointF>> targetPositions = m_scene->getTargetPositions();

	m_groups = startPositions.size();

	for (int g = 0; g < m_groups; g++)
	{
		m_robotNum.push_back(startPositions[g].size());
	}
	m_startConfSet = vector<ConfSet>(m_groups);
	m_targetConfSet = vector<ConfSet>(m_groups);

	for (int g = 0; g < m_groups; g++)
	{
		for (int i=0; i < m_robotNum[g]; i++)
		{
			m_startConfSet[g].push_back(transformToCGAL(startPositions[g][i]));
			m_targetConfSet[g].push_back(transformToCGAL(targetPositions[g][i]));
		}
	}

	Point_2 ptl = transformToCGAL(m_scene->getRoomTopLeft());
	Point_2 pbr = transformToCGAL(m_scene->getRoomBottomRight());
	m_room = Room(ptl,pbr);

	m_collision = new SimpleCDwObstacles(m_radius, &m_obstacles, &m_stats);
	m_sampler = new Sampler(m_radius, &m_obstacles, m_room, m_collision, m_robotNum, &m_stats);

	m_extConfCounter = IDFactory();

	generateStartTargetExt();
}

Planner::~Planner()
{
	delete m_collision;
	delete m_sampler;
	delete m_startExt;
	delete m_targetExt;
}

/*	This function is invoked by the GUI and is assumed to update the resulting */
void Planner::run()
{
	if (!m_stats.IS_SVESTKA)
	{
		srand(time(NULL));
		clock_t time_begin=clock();
	
		//	TIME: generateExtConfs();
		clock_t time_begin1=clock();
		generateExtConfs();
		clock_t time_end1=clock();
		double elapsedSec1 = difftime(time_end1,time_begin1)/1000;

		//	TIME: connectExtConfs();
		clock_t time_begin2=clock();
		connectExtConfs();
		clock_t time_end2=clock();
		double elapsedSec2 = difftime(time_end2,time_begin2)/1000;

		// path section number - group - info
		vector<vector<pair<QTPath,GeometricGraph>>>  result;

		int groups = m_scene->getGroupCount();

		//	TIME: consturctRoadmap
		clock_t time_begin3=clock();

		if (m_extEdgeSet.empty())
		return;

		// feed edges to graph
		// find path from start to goal in graph
		vector<CompPath> extPath;
		constructRoadmapPath(&extPath);

		clock_t time_end3=clock();
		double elapsedSec3 = difftime(time_end3,time_begin3)/1000;

		// source and target in different connected component
		if (extPath.empty())
		{
			clock_t time_end=clock();
			double elapsedSec = difftime(time_end,time_begin)/1000;
	
			/*FILE* fOutput = fopen("benchmark.txt", "a+");
			fprintf(fOutput, "################################\n");
			fprintf(fOutput, "TOTAL: %f\n", elapsedSec);
			fprintf(fOutput, "~~~~~~~~~~\n");
			fprintf(fOutput, "CONF_CREATION: %f\n", elapsedSec1);
			fprintf(fOutput, "CONF_CONNECTION: %f\n", elapsedSec2);
			fprintf(fOutput, "ROADMAP_CONSTUCTION: %f\n\n", elapsedSec3);
			fclose(fOutput);*/

			FILE* fOutput = fopen("time.txt", "w+");
			fprintf(fOutput, "%f", elapsedSec);
			fclose(fOutput);
				return;
		}

		/*	The result path is represented by a vector of configurations.
		Each configuration describes the position of all robots and implemented
		by a vector of points	*/

		//	TIME: pathGeneration
		clock_t time_begin4=clock();

		// trasform path into a real path
		if (extPath[0].cc == m_startExt)
		{
		
			typedef pair<QTPath, GeometricGraph> MovementSet;
			list<MovementSet> path;

			GeometricGraph dummyGraph;

			vector<Path> dummyPath(groups);
			for (int g = 0; g < groups; g++)
				dummyPath[g].push_back(m_startConfSet[g]);
			vector<MovementSet>	dummyMovementCollection(groups);
			for (int g = 0; g < groups; g++)
			{
				dummyMovementCollection[g] = MovementSet(transformToQT(dummyPath[g]), dummyGraph);
			}
		
			result.push_back(dummyMovementCollection);
			vector<vector<int>> robotPos(groups);
			for (int g = 0; g < groups; g++)
			{
				for (int i = 0; i < m_robotNum[g]; i++)
				robotPos[g].push_back(i);
			}
		
			//duplicate final extConf
			CompPath last = extPath[extPath.size()-1];
			extPath.push_back(last);

			for (int i = 1; i < extPath.size(); i = i + 2)
			{
				// calculate path within roadmap 
				CompPath prev = extPath[i - 1];
				CompPath source = extPath[i];
				CompPath target = extPath[i + 1];

				for (int g = 0; g < groups; g++)
				{
					for (int r = 0; r < m_robotNum[g]; r++)
					{
						for (int l = 0; l < m_robotNum[g]; l++)
						{
							if ((*(prev.sub))[g][l] == robotPos[g][r])
							{
								robotPos[g][r] = (*(source.sub))[g][l];
								break;
							}
						}
					}
				}

				CGAL_precondition(source.cc == target.cc);
				vector<Path> subpath = source.cc->getPath(source.sub, target.sub, &robotPos);
				vector<MovementSet>	movementCollection(groups);
				vector<GeometricGraph> graphs = source.cc->getGeometricGraph();
			
				for (int g = 0; g < groups; g++)
				{
					movementCollection[g] = MovementSet(transformToQT(subpath[g]), graphs[g]);
				}
				result.push_back(movementCollection);
			}
		}
	
		/*	If "result" is empty, the GUI assumes that a solution haven't been found */
		m_scene->setPath(result);

		clock_t time_end4=clock();
		double elapsedSec4 = difftime(time_end4,time_begin4)/1000;

		clock_t time_end=clock();
		double elapsedSec = difftime(time_end,time_begin)/1000;
	
		/*
		FILE* fOutput = fopen("benchmark.txt", "a+");
		fprintf(fOutput, "################################\n");
		fprintf(fOutput, "TOTAL: %f\n", elapsedSec);
		fprintf(fOutput, "~~~~~~~~~~\n");
		fprintf(fOutput, "CONF_CREATION: %f\n", elapsedSec1);
		fprintf(fOutput, "CONF_CONNECTION: %f\n", elapsedSec2);
		fprintf(fOutput, "ROADMAP_CONSTUCTION: %f\n", elapsedSec3);
		fprintf(fOutput, "PATH_GENERATION: %f\n\n", elapsedSec4); */
		FILE* fOutput = fopen("time.txt", "w+");
		fprintf(fOutput, "%f", elapsedSec);
		fclose(fOutput);
		return;
	}
	srand(time(NULL));
	clock_t time_begin=clock();
	PlannerSvestka planner_svestka(m_collision, m_sampler,
		m_startConfSet, m_targetConfSet, m_robotNum, &m_stats);
		
	vector<vector<pair<QTPath,GeometricGraph>>>  result = planner_svestka.run();
	clock_t time_end=clock();
	double elapsedSec = difftime(time_end,time_begin)/1000;
	
	FILE* fOutput = fopen("time.txt", "w+");
	fprintf(fOutput, "%f", elapsedSec);
	fclose(fOutput);
	m_scene->setPath(result);
}

void	Planner::generateExtConfs()
{
	int conf_num;
	if (m_stats.IS_BASIC)
		conf_num = m_stats.basic_EXTCONFS;
	else
		conf_num = m_stats.complex_EXTCONFS;

	for (int i = 0; i < conf_num; i++)
	{
		vector<ConfGraph> cg = m_sampler->generateGroupGraphs();
		if (cg.empty())
		{
			i--;
			continue;
		}
		CompositeConf ec(cg, &m_extConfCounter, m_isBasic, m_groups);
		m_extConfSet.push_back(ec);
	}
}

void	Planner::connectExtConfs()
{
	
	Connector* con;
	if (m_isBasic)
		con = new BrutForceConnector(&m_extConfSet, &m_extEdgeSet,
			m_startExt, m_targetExt, m_robotNum, m_collision, &m_stats);
	else
		con = new BrutForceConnector(&m_extConfSet, &m_extEdgeSet,
			m_startExt, m_targetExt, m_robotNum, m_collision, &m_stats);
	con->generateExtConfEdges();
	con->connectStartTarget();

	// delete con;
}

void	Planner::generateStartTargetExt()
{
	Edges dummyEdges;
	vector<ConfGraph> s, t;

	for (int g = 0; g < m_groups; g++)
	{
		s.push_back(ConfGraph(m_startConfSet[g],dummyEdges));
		t.push_back(ConfGraph(m_targetConfSet[g],dummyEdges));

	}

	m_startExt = new CompositeConf(s, &m_extConfCounter,true, m_groups);
	m_targetExt = new CompositeConf(t, &m_extConfCounter,true, m_groups);

	vector<vector<int>> dummy(m_groups);
	for(int g = 0; g < m_groups; g++)
	{
		for (int i = 0; i < m_robotNum[g]; i++)
		{
			dummy[g].push_back(i);
		}
	}

	m_targetID = m_targetExt->getInstanceID(dummy);
	m_startID = m_startExt->getInstanceID(dummy);
}

void Planner::constructRoadmapPath(vector<CompPath>*	result)
{
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
	for (int i = 0; i < m_extEdgeSet.size(); i++)
	{
		int sig1 = m_extEdgeSet[i].ex1->getInstanceID(m_extEdgeSet[i].vert1);
		int sig2 = m_extEdgeSet[i].ex2->getInstanceID(m_extEdgeSet[i].vert2);

		add_edge(sig1, sig2, g);

		IndexEdgeKey key, key2;
		key.i1 = sig1;
		key.i2 = sig2;

		key2.i1 = sig2;
		key2.i2 = sig1;

		edgeMap.insert(pair<IndexEdgeKey,int>(key, i));
		edgeMap.insert(pair<IndexEdgeKey,int>(key2, i));
	}

	int numVert = m_extConfCounter.count();

	/*	create data structures used in the bfs algorithm
		as the predecessor map	*/
	vector<Vertex> p(numVert);
	vector<graph_traits<Graph>::vertices_size_type> d;
	for (int i = 0; i < numVert; i++)
		d.push_back(0);

	//	start vertex
	Vertex s_v = vertex(m_startID, g);
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
	int currVert = m_targetID;
	vertexPath.push_back(currVert);
	while (p[currVert] != 0)
	{
		currVert = p[currVert];
		vertexPath.push_back(currVert);
		if (currVert == s_v)
			break;
	}

	if (currVert != m_startID)
	{
		result->clear();
		return;
	}

	for (int i = vertexPath.size() - 2; i >= 0; i--)
	{
		IndexEdgeKey key;
		key.i1 = vertexPath[i + 1];
		key.i2 = vertexPath[i];
		map<IndexEdgeKey, int>::iterator edgeIter = edgeMap.find(key);
		CGAL_precondition( edgeIter != edgeMap.end());
		//ExtendedConfEdge e = m_extEdgeSet[edgeIter->second];

		CompPath prev, curr;

		int ID = m_extEdgeSet[edgeIter->second].ex1->getInstanceID(m_extEdgeSet[edgeIter->second].vert1);

		if (ID == vertexPath[i + 1])
		{
			prev.cc = m_extEdgeSet[edgeIter->second].ex2;
			prev.sub = &(m_extEdgeSet[edgeIter->second].vert2);
			curr.cc = m_extEdgeSet[edgeIter->second].ex1;
			curr.sub = &(m_extEdgeSet[edgeIter->second].vert1);
		}
		else
		{
			prev.cc = m_extEdgeSet[edgeIter->second].ex1;
			prev.sub = &(m_extEdgeSet[edgeIter->second].vert1);
			curr.cc = m_extEdgeSet[edgeIter->second].ex2;
			curr.sub = &(m_extEdgeSet[edgeIter->second].vert2);
		}

		result->push_back(curr);
		result->push_back(prev);
	}
}

void Planner::test()
{
}