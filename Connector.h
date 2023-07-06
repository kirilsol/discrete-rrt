#pragma once
#include "basic_typedef.h"
#include "CompositeConf.h"
#include "SimpleCDwObstacles.h"

class Connector
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	Connector(vector<CompositeConf>* ecl, vector<CompositeConfEdge>* ecel, 
		CompositeConf* start, CompositeConf* target, vector<int> robotNum, CollisionDetector* col, Stats* stats) 
		: m_ec(ecl), m_edges(ecel), m_start(start), m_target(target), 
		m_robotNum(robotNum), m_col(col), m_stats(stats)
	{ 
		m_basic = m_stats->IS_BASIC;

		if (m_basic)
		{
			m_pair_connections = m_stats->basic_PAIR_EXT_CONNECTIONS;
			m_st_connections = m_stats->basic_START_TARGET_CONNECTIONS;
		}
		else
		{
			m_pair_connections = m_stats->complex_PAIR_EXT_CONNECTIONS;
			m_st_connections = m_stats->complex_START_TARGET_CONNECTIONS;
		}
	}

	~Connector();

	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateExtConfEdges() = 0;
	virtual		void	connectStartTarget() = 0;

	////////////////////////
	// Data Members
	////////////////////////
	
	vector<CompositeConf>* m_ec;
	vector<CompositeConfEdge>* m_edges;
	CompositeConf* m_start;
	CompositeConf* m_target;
	vector<int>			m_robotNum;
	CollisionDetector* m_col;
	Stats*		m_stats;
	bool		m_basic;
	int			m_pair_connections;
	int			m_st_connections;
};
