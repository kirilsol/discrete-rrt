#include "Sampler.h"
#include <math.h>
#include <stdlib.h>

////////////////////////
// CTORS
////////////////////////
Sampler::~Sampler(void)
{
}

////////////////////////
// Queries
////////////////////////
	
/*	Returns one valid configuration	*/
Conf Sampler::generateConf()
{
	while(true)
	{
		double randx = xmin + lenx * ((double)rand())/((double)RAND_MAX);
		double randy = ymin + leny * ((double)rand())/((double)RAND_MAX);

		Point_2 p(randx,randy);

		if (m_col->testSingleRobot(p))
			return p;
	}
}

/*	Returns a configuration set	*/
ConfSet Sampler::generateConfSet()
{
	ConfSet set; 
	int size;

	if (m_basic)
		size = m_robots;
	else
		size = m_stats->complex_MAX_CONFSET_SIZE;
	for (int i = 0; i < m_stats->complex_CONFSET_ATTEMPTS_TRESHOLD 
		&& set.size() < size; i++)
	{
		Conf conf = generateConf();
		if (m_col->testCandidateConf(&set, conf))
			set.push_back(conf);
	}

	if (m_basic && set.size()<size)
	{
		set.clear();
	}

	return set;
}

/*  Returns a Graph for some sampled ConfSet	*/
ConfGraph Sampler::generateGraph()
{
	ConfSet set = generateConfSet();
	Edges edges;
	if (!set.empty())
	{
		for (int i = 0; i < set.size() - 1; i++)
		{
			for (int j = i+1; j < set.size(); j++)
			{
				if (m_col->testEdge(&set, i, j))
					edges.push_back(Edge(i,j));
			}
		}
	}
	
	return ConfGraph(set,edges);
}

/*  Returns a Graph for some sampled ConfSet	*/
vector<ConfGraph> Sampler::generateGroupGraphs()
{
	vector<ConfGraph> result;
	ConfSet set = generateConfSet();
	vector<ConfSet> groupSets;

	// split set into several sets

	// check if we have enough configurations
	if (m_robots > set.size())
		return result;

	std::random_shuffle(set.begin(), set.end());
	
	vector<int>	weights;
	if (m_basic)
	{
		weights = m_robotGroups;
	}
	else
	{
		int confsForRobot = floor((double)set.size() / (double)m_robots);
		for (int g = 0; g < m_groups; g++)
		{
			weights.push_back(m_robotGroups[g] * confsForRobot);
		}
	}
	
	int elementCount = 0;
	vector<vector<int>> groupConfToConfMap(m_groups);
	for (int g = 0; g < m_groups; g++)
	{
		ConfSet newSet;
		for (int i = 0; i < weights[g]; i++)
		{
			Conf c = set[elementCount];
			newSet.push_back(c);
			groupConfToConfMap[g].push_back(elementCount);
			elementCount++;
		}
		groupSets.push_back(newSet);
	}

	vector<Edges> edges(m_groups);
	for (int g = 0; g < m_groups; g++)
	{
		for (int i = 0; i < groupSets[g].size() - 1; i++)
		{
			for (int j = i+1; j < groupSets[g].size(); j++)
			{
				int iold = groupConfToConfMap[g][i];
				int jold = groupConfToConfMap[g][j];
				if (m_col->testEdge(&set, iold, jold))
					edges[g].push_back(Edge(i,j));
			}
		}
	}

	for (int g = 0; g < m_groups; g++)
		result.push_back(ConfGraph(groupSets[g],edges[g]));
	
	return result;
}