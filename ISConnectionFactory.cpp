#include "ISConnectionFactory.h"

void	ISConnectionFactory::generateEdges()
{
	// generate all vertex pair edges from both extConfs
	vector<pair<int,int>>	intEdges;
	vector<list<int>>		adjacency;
	vector<vector<int>>		edgeSets;
	//vector<pair<Conf,Conf>> confEdges;

	vector<list<int>>		vertEdgeMap1(m_cs1->size());
	vector<list<int>>		vertEdgeMap2(m_cs2->size());

	for (int i = 0; i < m_cs1->size(); i++)
	{
		for (int j = 0; j < m_cs2->size(); j++)
		{
			bool valid = m_col->testSingleLocalPlanner((*m_cs1)[i], (*m_cs2)[j]);
			if (valid)
			{
				intEdges.push_back(Edge(i,j));
				vertEdgeMap1[i].push_back(intEdges.size() - 1);
				vertEdgeMap2[j].push_back(intEdges.size() - 1);
			}
		}
	}

	if (intEdges.size() < m_robotNum)
		return;

	adjacency = vector<list<int>>(intEdges.size());

	int countInterferences = 0;
	
	//	find all interferences
	for (int i = 0; i < intEdges.size() - 1; i++)
	{
		for (int j = i + 1; j < intEdges.size(); j++)
		{
			vector<int> sub1,sub2;
			Edge ei = intEdges[i];
			Edge ej = intEdges[j];

			sub1.push_back(ei.first);
			sub1.push_back(ej.first);

			sub2.push_back(ei.second);
			sub2.push_back(ej.second);

			bool free = m_col->testLocalPlannerNoObstacles(m_cs1, &sub1, m_cs2, &sub2);
			if (!free)
			{
				adjacency[i].push_back(j);
				adjacency[j].push_back(i);
				countInterferences++;
			}
		}
	}

	vector<bool> is_visited;
	vector<int> live;
	
	
	for (int i = 0; i < intEdges.size(); i++)
	{
		is_visited.push_back(false);
		live.push_back(i);	
	}

	// find independent sets
	for (int i = 0; i < m_connNum; i++)
	{
		std::random_shuffle(live.begin(), live.end());
		vector<int> selected;
		vector<int> clean;
		int pos = 0;
		vector<bool> selectedV1(m_cs1->size());
		vector<bool> selectedV2(m_cs2->size());

		for (int i = 0; i < selectedV1.size(); i++)
			selectedV1[i] = false;
		for (int i = 0; i < selectedV2.size(); i++)
			selectedV2[i] = false;

		while (selected.size() < m_robotNum)
		{
			bool found = false;
			int newSelect;
			while (pos < live.size())
			{
				newSelect = live[pos];
				pos++;
				int vert1 = intEdges[newSelect].first;
				int vert2 = intEdges[newSelect].second;
				if (!is_visited[newSelect] && !selectedV1[vert1] && !selectedV2[vert2])
				{
					found = true;
					selectedV1[vert1] = true;
					selectedV2[vert2] = true;
					break;
				}
			}
			
			if (!found)
				break;

			selected.push_back(newSelect);
			clean.push_back(newSelect);
			is_visited[newSelect] = true;

			foreach(int neighbor, adjacency[newSelect])
			{
				is_visited[neighbor] = true;
				clean.push_back(neighbor);
			}
		}

		// reset ds's
		foreach(int visited, clean)
		{
			is_visited[visited] = false;
		}
		
		if (selected.size() != m_robotNum)
			continue;
		
		edgeSets.push_back(selected);
		if (m_basic)
			break;
	}

	// transform edges to confEdges
	foreach(vector<int> edgeSet, edgeSets)
	{
		vector<int> ss1,ss2;
		
		foreach(int e, edgeSet)
		{
			int v1 = intEdges[e].first;
			int v2 = intEdges[e].second;

			ss1.push_back(v1);
			ss2.push_back(v2);
		}

		ExtendedConfEdge ece;
		ece.ex1 = m_ec1;
		ece.ex2 = m_ec2;
		ece.vert1 = ss1;
		ece.vert2 = ss2;

		m_edges->push_back(ece);
	}
}