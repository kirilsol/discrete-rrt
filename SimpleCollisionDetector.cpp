#include "SimpleCollisionDetector.h"
#include <CGAL/squared_distance_2.h>
#include <CGAL/enum.h>

/*	checks if position pos of robot with radius size is 
	collision free with obstacles */
bool	SimpleCollisionDetector::testSingleRobot(Conf pos)
{
	return true;
}

/*  checks if the given new configuration interferes with
	existing configurations */
bool	SimpleCollisionDetector::testCandidateConf(ConfSet *confs, Conf pos)
{
	static const double MINIMAL_SQUARED_DISTANCE = 4 * m_radius.to_double() * m_radius.to_double() ;

	for (ConfSet::iterator iter = confs->begin(); iter != confs->end(); iter++)
	{
		if (!isSquaredDistanceLarge(pos, *iter, MINIMAL_SQUARED_DISTANCE))
			return false;
	}
	return true;
}

/*	checks if the edge between conf1,conf2 is valid,
	i.e. a robot doesn't intersect with any other  configurations
	while moving from one configuration to another.	*/
bool	SimpleCollisionDetector::testEdge(ConfSet *confs, int confind1, int confind2)
{
	for (int i = 0; i < confs->size(); i++)
	{
		if ((i != confind1 && i != confind2)
			&& (!isTwoRobotsClear((*confs)[i],(*confs)[i],(*confs)[confind1],(*confs)[confind2])))
			return false;
	}
	return true;
}

/*	given two sets of configurations decides whether the local-planner
	movement between these subsets of con figurations is possible	*/
bool	SimpleCollisionDetector::testLocalPlanner(ConfSet *confs1, std::vector<int> *subset1, 
	ConfSet *confs2, std::vector<int> *subset2)
{
	for (int i = 0; i < subset1->size() - 1; i++)
	{
		for (int j = i+1; j < subset1->size(); j++)
		{
			if (i != j)
			{
				Conf s1 = (*confs1)[(*subset1)[i]];
				Conf t1 = (*confs2)[(*subset2)[i]];
				Conf s2 = (*confs1)[(*subset1)[j]];
				Conf t2 = (*confs2)[(*subset2)[j]];
				if (!isTwoRobotsClear(s1, t1, s2, t2))
					return false;
			}		
		}
	}
	return true;
}

/*	given two sets of configurations decides whether the local-planner
	movement between these subsets of con figurations is possible	*/
bool	SimpleCollisionDetector::testLocalPlannerNoObstacles(ConfSet *confs1, std::vector<int> *subset1, 
	ConfSet *confs2, std::vector<int> *subset2)
{
	return testLocalPlanner(confs1, subset1, confs2, subset2);
}

/*	given start and goal positions for robot decides whether it 
		doesn't collide with obstacles	*/
bool	SimpleCollisionDetector::testSingleLocalPlanner(Conf start, Conf target)
{	
	return true;
}