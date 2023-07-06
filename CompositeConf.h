#pragma once

#include "ExtendedConf.h"

class CompositeConf
{
public: 
	///////////////////////
	// CTORS
	////////////////////////
	CompositeConf(vector<ConfGraph> cgraphs, IDFactory* vertexCounter, 
		bool basic, int groups);

	~CompositeConf(void);

	////////////////////////
	// Queries
	////////////////////////

	vector<ConfSet*>	getConfSet()
	{
		vector<ConfSet*> result;
		for (int g = 0; g < m_groups; g++)
			result.push_back(m_extConfs[g].getConfSet());
		return result;

	}
	vector<int>			getNumOfConf()
	{
		vector<int> result;
		for (int g = 0; g < m_groups; g++)
			result.push_back(m_extConfs[g].getNumOfConf());
		return result;
	}
	ConfSet				getConfSubset(vector<vector<int>> ind);
	
	vector<Path>		getPath(vector<vector<int>>* startConf, 
		vector<vector<int>>* targetConf, vector<vector<int>>* robotsOrder);

	vector<vector<int>> generateRandomSubset(vector<int> robotNum);

	bool				isBasic(){ return m_isBasic; }

	int					getInstanceID(vector<vector<int>> vertices);

	vector<pair<vector<pair<Conf,Conf>>,ConfSet>> getGeometricGraph();

private:

	vector<Conf>			m_robotPos;
	InstancesManager		m_manager;
	//IDFactory*				m_vertexCounter;
	bool					m_isBasic;
	vector<ExtendedConf>	m_extConfs;
	int						m_groups;

};

struct CompositeConfEdge
{
	CompositeConfEdge reverseDirection()
	{
		CompositeConfEdge rev;
		rev.ex1 = ex2;
		rev.vert1 = vert2;
		rev.ex1 = ex2;
		rev.vert1 = vert2;

		return rev;
	}
	
	CompositeConf* ex1;
	CompositeConf* ex2;
	vector<vector<int>>	vert1, vert2;
};