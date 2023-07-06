#pragma once

#include "basic_typedef.h"
#include "CompositeConf.h"
#include "SimpleCDwObstacles.h"

class CompositeConnectionFactory
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	CompositeConnectionFactory(CompositeConf* ec1, CompositeConf* ec2, vector<CompositeConfEdge>* ecel, 
		vector<int> robotNum, CollisionDetector* col, Stats* stats) 
			: m_ec1(ec1), m_ec2(ec2), m_edges(ecel)
			, m_robotNum(robotNum), m_col(col), m_stats(stats)
	{ 
		m_basic = m_stats->IS_BASIC;
		m_cs1 = ec1->getConfSet();
		m_cs2 = ec2->getConfSet();

		if (m_basic)
			m_connNum = m_stats->basic_PAIR_EXT_CONNECTIONS;
		else
			m_connNum = m_stats->complex_PAIR_EXT_CONNECTIONS;
	}

	~CompositeConnectionFactory(){};

	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateEdges() = 0;

	////////////////////////
	// Data Members
	////////////////////////
	
	CompositeConf*				m_ec1;
	CompositeConf*				m_ec2;

	vector<ConfSet*>			m_cs1;
	vector<ConfSet*>			m_cs2;

	vector<CompositeConfEdge>*	m_edges;
	vector<int>					m_robotNum;
	CollisionDetector*			m_col;
	Stats*						m_stats;
	bool						m_basic;

	int							m_connNum;

};
