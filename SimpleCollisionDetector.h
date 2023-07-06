#pragma once

#include "CollisionDetector.h"

class SimpleCollisionDetector : public CollisionDetector
{
public:
	SimpleCollisionDetector(double radius, Obstacles *obs, Stats* stats)
		: CollisionDetector(radius,obs, stats){}
	~SimpleCollisionDetector(){}

	/*	checks if position pos of robot with radius size is 
		collision free with obstacles */
	virtual		bool	testSingleRobot(Conf pos);

	/*  checks if the given new configuration interferes with
		existing configurations */
	virtual		bool	testCandidateConf(ConfSet *confs, Conf pos);

	/*	checks if the edge between conf1,conf2 is valid,
		i.e. a robot doesn't intersect with any other  configurations
		while moving from one configuration to another	*/
	virtual		bool	testEdge(ConfSet *confs, int confind1, int confind2);

	/*	given two sets of configurations decides whether the local-planner
		movement between these subsets of configurations is possible	*/
	virtual		bool	testLocalPlanner(ConfSet *confs1, std::vector<int> *subset1, 
		ConfSet *confs2, std::vector<int> *subset2);

	/*	given two sets of configurations decides whether the local-planner
	movement between these subsets of configurations is possible	*/
	virtual		bool	testLocalPlannerNoObstacles(ConfSet *confs1, std::vector<int> *subset1, 
		ConfSet *confs2, std::vector<int> *subset2);

	/*	given start and goal positions for robot decides whether it 
		doesn't collide with obstacles	*/
	virtual		bool	testSingleLocalPlanner(Conf start, Conf target);
};
