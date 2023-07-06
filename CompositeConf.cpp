#include "CompositeConf.h"

///////////////////////
// CTORS
////////////////////////

CompositeConf::CompositeConf(vector<ConfGraph> cgraph, 
											   IDFactory* vertexCounter, 
											   bool basic, int groups) : 
	m_manager(vertexCounter), m_isBasic(basic), m_groups(groups)
{
	//m_extConfCounter = new IDFactory();
	for (int g = 0; g < m_groups; g++)
	{
		m_extConfs.push_back(ExtendedConf(cgraph[g], vertexCounter, m_isBasic));
	}
}



CompositeConf::~CompositeConf(void)
{
	if (m_isBasic)
		return; 
}

////////////////////////
// Queries
////////////////////////

//Path		CompositeConf::getPath(vector<int>* startConf, vector<int>* targetConf, 
//	vector<int>* robotPosition)
vector<Path>		CompositeConf
	::getPath(vector<vector<int>>* startConf, 
		vector<vector<int>>* targetConf, vector<vector<int>>* robotsOrder)
{
	// get individual paths
	vector<Path>	individualPaths;
	for (int g = 0; g < m_groups; g++)
	{
		individualPaths.push_back(m_extConfs[g].getPath(&(*startConf)[g], 
			&(*targetConf)[g], &(*robotsOrder)[g]));
	}

	return individualPaths;
}

vector<vector<int>> CompositeConf::generateRandomSubset(vector<int> robotNum)
{
	vector<vector<int>> result;

	for (int g = 0; g < m_groups; g++)
	{
		result.push_back(m_extConfs[g].generateRandomSubset(robotNum[g]));
	}
	
	return result;
}

int			CompositeConf::getInstanceID(vector<vector<int>> vertices)
{
	vector<int> sig;
	if (!m_isBasic)
	{
		for (int g = 0; g < m_groups; g++)
		{
			sig.push_back(m_extConfs[g].getInstanceID(vertices[g]));
		}
	}

	return m_manager.getInstanceID(sig);
}

ConfSet	CompositeConf::getConfSubset(vector<vector<int>> ind)
{
	ConfSet result;
	for (int g = 0; g < m_groups; g++)
	{
		ConfSet cs = m_extConfs[g].getConfSubset(&(ind[g]));
		result.insert(result.end(), cs.begin(), cs.end());
	}

	return result;
}

vector<pair<vector<pair<Conf,Conf>>,ConfSet>> CompositeConf::getGeometricGraph()
{
	vector<pair<vector<pair<Conf,Conf>>,ConfSet>> result;
	for (int g = 0; g < m_groups; g++)
	{
		result.push_back(m_extConfs[g].getGeometricGraph());
	}
	return result;
}
