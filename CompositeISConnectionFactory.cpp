#include "CompositeISConnectionFactory.h"

void	CompositeISConnectionFactory::generateEdges()
{
	int groups = m_cs1.size();
	// generate all vertex pair edges from both extConfs
	vector<pair<int,int>>	intEdges;				// edge index to edge map
	vector<int>				edgeGroupMap;			// edge index to group map
	vector<vector<int>>		groupEdgeMap(groups);	// group to edges map
	vector<list<int>>		adjacency;				// edge index to neighbors map
	vector<vector<int>>		edgeSets;				// intermidiate result is stored here 

	// information for linear programming
	int num_edges;
	vector<vector<int>> interferences;

	vector<vector<list<int>>>		vertEdgeMap1(groups); // group index to vertex to all incident edges of this vertex
	vector<vector<list<int>>>		vertEdgeMap2(groups);

	for (int g = 0; g < groups; g++)
	{
		vertEdgeMap1[g] = vector<list<int>>(m_cs1[g]->size());
		vertEdgeMap2[g] = vector<list<int>>(m_cs2[g]->size());
	}

	// generate all valid (do not collide with obstacles)
	//edges within the same group 
	for (int g = 0; g < groups; g++)
	{
		for (int i = 0; i < m_cs1[g]->size(); i++)
		{
			for (int j = 0; j < m_cs2[g]->size(); j++)
			{
				bool valid = m_col->testSingleLocalPlanner((*(m_cs1[g]))[i], (*(m_cs2[g]))[j]);
				if (valid)
				{
					intEdges.push_back(Edge(i,j));
					int edgeIndex = intEdges.size() - 1;

					vertEdgeMap1[g][i].push_back(edgeIndex);
					vertEdgeMap2[g][j].push_back(edgeIndex);
					edgeGroupMap.push_back(g);
					groupEdgeMap[g].push_back(edgeIndex);
				}
			}
		}
	}

	num_edges = intEdges.size();

	for (int i = 0; i < num_edges; i++)
	{
		vector<int> dummy;
		for (int j=0; j < num_edges; j++)
			dummy.push_back(0);
		interferences.push_back(dummy);
	}

	int totalRobots = 0; // total number of robots
	foreach(int rn, m_robotNum)
		totalRobots = totalRobots + rn;

	if (intEdges.size() < totalRobots)
		return; // too little edges

	adjacency = vector<list<int>>(intEdges.size());

	int countInterferences = 0;
	
	//	find all interferences
	for (int i = 0; i < intEdges.size() - 1; i++)
	{
		for (int j = i + 1; j < intEdges.size(); j++)
		{
			Edge ei = intEdges[i];
			int gi = edgeGroupMap[i];
			Edge ej = intEdges[j];
			int gj = edgeGroupMap[j];

			// TODO: Remove unessesary checks when two edges are from the same vertex

			vector<int> sub;
			sub.push_back(0);
			sub.push_back(1);

			ConfSet cs1,cs2;
			cs1.push_back((*(m_cs1[gi]))[ei.first]);
			cs2.push_back((*(m_cs2[gi]))[ei.second]);

			cs1.push_back((*(m_cs1[gj]))[ej.first]);
			cs2.push_back((*(m_cs2[gj]))[ej.second]);

			bool free = m_col->testLocalPlannerNoObstacles(&cs1, &sub, &cs2, &sub);
			if (!free)
			{
				interferences[i][j]=1;
				interferences[j][i]=1;
				adjacency[i].push_back(j);
				adjacency[j].push_back(i);
				countInterferences++;
			}
		}
	}

	print_ip(num_edges,interferences);

	// true if edge was visited during the last iteration of IS algorithm
	vector<bool> is_visited; 
	// contains a shuffle of the edge indices
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

		vector<int> selected;	// IS edges
		vector<int> clean;		// edges that were visited during the run
		int pos = 0;
		
		/*	DS that records the vertices that were picked during the run,
			in order to avoid situations where two edges from the same vertex
			are selected	*/
		vector<vector<bool>> selectedV1(groups); 
		vector<vector<bool>> selectedV2(groups);
		
		//	number of edges collected so far within each group
		vector<int>		 groupOccupancy(groups);

		for (int g = 0; g < groups; g++)
		{
			for (int j = 0; j < m_cs1[g]->size(); j++)
				selectedV1[g].push_back(false);
			for (int j = 0; j < m_cs2[g]->size(); j++)
				selectedV2[g].push_back(false);
			groupOccupancy[g] = 0;
		}
		
		while (selected.size() < totalRobots)
		{
			bool found = false;
			int newSelect, newGroup;
			while (pos < live.size())
			{
				newSelect = live[pos];
				newGroup = edgeGroupMap[newSelect];
				pos++;

				int vert1 = intEdges[newSelect].first;
				int vert2 = intEdges[newSelect].second;
				if (!is_visited[newSelect] 
					&& !selectedV1[newGroup][vert1] 
					&& !selectedV2[newGroup][vert2]
					&&	groupOccupancy[newGroup] + 1 <= m_robotNum[newGroup] )
				{
					found = true;
					selectedV1[newGroup][vert1] = true;
					selectedV2[newGroup][vert2] = true;
					groupOccupancy[newGroup]++;
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

		if (selected.size() != totalRobots)
		{
			if (intEdges.size() == totalRobots)
				break;
			
			continue;
		}
	
		edgeSets.push_back(selected);
		if (m_basic)
			break;
	}

	// transform edges to confEdges
	foreach(vector<int> edgeSet, edgeSets)
	{
		vector<vector<int>> ss1(groups),ss2(groups);
		
		foreach(int e, edgeSet)
		{
			int v1 = intEdges[e].first;
			int v2 = intEdges[e].second;
			int g = edgeGroupMap[e];

			ss1[g].push_back(v1);
			ss2[g].push_back(v2);
		}

		CompositeConfEdge ece;
		ece.ex1 = m_ec1;
		ece.ex2 = m_ec2;
		ece.vert1 = ss1;
		ece.vert2 = ss2;

		m_edges->push_back(ece);
	}
}

void	CompositeISConnectionFactory::print_ip(int num_edges, vector<vector<int>> interferences)
{
	FILE* fOutput = fopen("interferences.csv", "w+");
	if (!fOutput)
	{
		return;
	}

	for (int i = 0; i < num_edges-1; i++)
	{
		fprintf(fOutput, "%d,", 1);
	}
	fprintf(fOutput, "%d\n", 1);

	for (int i = 0; i < num_edges-1; i++)
	{
		fprintf(fOutput, "%d,", -1);
	}
	fprintf(fOutput, "%d\n", -1);

	for (int i = 0; i < num_edges; i++)
	{
		for (int j = 0; j < num_edges-1; j++)
		{
			fprintf(fOutput, "%d,", interferences[i][j]);
		}
		fprintf(fOutput, "%d\n", interferences[i][num_edges-1]);
	}
	fclose(fOutput);
}