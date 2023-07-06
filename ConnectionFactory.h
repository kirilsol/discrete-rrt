#pragma once

#include "basic_typedef.h"
#include "ExtendedConf.h"
#include "SimpleCDwObstacles.h"

class ConnectionFactory
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	ConnectionFactory(ExtendedConf* ec1, ExtendedConf* ec2, vector<ExtendedConfEdge>* ecel, 
		int robotNum, CollisionDetector* col, Stats* stats) 
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

	~ConnectionFactory(){};

	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateEdges() = 0;

	////////////////////////
	// Data Members
	////////////////////////
	
	ExtendedConf*				m_ec1;
	ExtendedConf*				m_ec2;

	ConfSet*					m_cs1;
	ConfSet*					m_cs2;

	vector<ExtendedConfEdge>*	m_edges;
	int							m_robotNum;
	CollisionDetector*			m_col;
	Stats*						m_stats;
	bool						m_basic;

	int							m_connNum;

};
