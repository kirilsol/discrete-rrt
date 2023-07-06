#pragma once

#include "basic_typedef.h"
#include "SimpleCollisionDetector.h"

class Sampler
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	Sampler(double radius, Obstacles *obs, Room room, CollisionDetector *col, vector<int> robots, Stats* stats) 
		: m_radius(radius), m_robotGroups(robots), m_stats(stats)
	{
		m_basic = stats->IS_BASIC;
		m_obs = obs;
		m_room = room;
		m_col = col;
		
		tl = m_room.first;
		br = m_room.second;
		xmin = tl.x()  + m_radius;
		xmax = br.x()  - m_radius;
		ymax = br.y()  - m_radius;
		ymin = tl.y()  + m_radius;
		lenx = xmax - xmin;
		leny = ymax - ymin;

		m_groups = m_robotGroups.size();

		m_robots = 0;
		foreach(int rn, m_robotGroups)
			m_robots = m_robots + rn;
	}
	~Sampler(void);

	////////////////////////
	// Queries
	////////////////////////
	
	/*	Returns one valid configuration	*/
	Conf generateConf(); 

	/*	Returns a configuration set	*/
	ConfSet generateConfSet();

	/*  Returns a Graph for some sampled ConfSet	*/
	ConfGraph generateGraph();

	/*	Returns graphs for each group of robots	*/
	vector<ConfGraph> generateGroupGraphs();

	////////////////////////
	// Data Members
	////////////////////////
	double		m_radius; // radius of robots
	Obstacles	*m_obs;
	Room		m_room;
	CollisionDetector	*m_col;
	bool			m_basic;
	vector<int>		m_robotGroups;
	int				m_robots;
	Stats*			m_stats;
	int				m_groups;

	// room variables
	Point_2 tl;
	Point_2 br;
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double lenx;
	double leny;
};
