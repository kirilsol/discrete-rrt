#include "RoadmapSolverTree.h"



////////////////////////
// CTORS
////////////////////////

RoadmapSolverTree::RoadmapSolverTree(Edges* edges) 
	: RoadmapSolver(edges)
{
	if (m_degenerate)
		return;

	foreach(Edge e, *m_edges)
	{
		int source = e.first;
		int target = e.second;

		Tree* sourceNode;
		Tree* targetNode;

		map<int,Tree*>::iterator iter = m_treeMap.find(source);
		if (iter == m_treeMap.end())
		{
			sourceNode = new Tree(source,&m_recorder);
			m_treeMap.insert(pair<int,Tree*>(source,sourceNode));
		} 
		else
			sourceNode = iter->second;

		map<int,Tree*>::iterator iter2 = m_treeMap.find(target);
		if (iter2 == m_treeMap.end())
		{
			targetNode = new Tree(target,&m_recorder);
			m_treeMap.insert(pair<int,Tree*>(target,targetNode));
		} 
		else
			targetNode = iter2->second;

		sourceNode->addChild(targetNode);
		targetNode->setFather(sourceNode);
	}

	map<int,Tree*>::iterator iter = m_treeMap.begin();
	m_root = iter->second->getRoot();
}

RoadmapSolverTree::~RoadmapSolverTree(void)
{
	for(map<int, Tree*>::iterator iter = m_treeMap.begin(); iter != m_treeMap.end(); iter++)
	{
		 //delete iter->second;
	}
}

MoveRecorder	RoadmapSolverTree::calculatePath(list<int> startPos, list<int> targetPos)
{
	m_recorder.clear();

	if (!m_degenerate && 
		((startPos.size() != 1) || (startPos.front() != targetPos.front())))
	{
		foreach(int s, startPos)
		{
			map<int,Tree*>::iterator iter = m_treeMap.find(s);
			CGAL_precondition(iter != m_treeMap.end());

			iter->second->putToken();
		}

		foreach(int t, targetPos)
		{
			map<int,Tree*>::iterator iter = m_treeMap.find(t);
			CGAL_precondition(iter != m_treeMap.end());

			iter->second->makeTarget();
		}

		m_root->run();
	}

	return m_recorder;
}