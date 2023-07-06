#include "basic_typedef.h"
#include <CGAL/squared_distance_2.h>
#include "Stats.h"

class CollisionDetector
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	CollisionDetector(double radius, Obstacles *obs, Stats* stats);
	~CollisionDetector(void);

	////////////////////////
	// Queries
	////////////////////////
	
	/*	checks if position pos of robot with radius size is 
		collision free with obstacles */
	virtual		bool	testSingleRobot(Conf pos) = 0;

	/*  checks if the given new configuration interferes with
		existing configurations */
	virtual		bool	testCandidateConf(ConfSet *confs, Conf pos) = 0;

	/*  checks if none of the configurations interfere	*/
	bool	testConfSet(ConfSet *confs)
	{
		ConfSet cs;
		cs.push_back((*confs)[0]);
		for (int i = 1; i < confs->size(); i++)
		{
			Conf c = (*confs)[i];
			if (!testCandidateConf(&cs, c))
				return false;
		}
		return true;
	}

	/*	checks if the edge between conf1,conf2 is valid,
		i.e. a robot doesn't intersect with any other  configurations
		while moving from one configuration to another	*/
	virtual		bool	testEdge(ConfSet *confs, int confind1, int confind2) = 0;

	/*	given two sets of configurations decides whether the local-planner
		movement between these subsets of configurations is possible	*/
	virtual		bool	testLocalPlanner(ConfSet *confs1, std::vector<int> *subset1, 
		ConfSet *confs2, std::vector<int> *subset2) = 0;

	/*	given two sets of configurations decides whether the local-planner
		movement between these subsets of configurations is possible	*/
	virtual		bool	testLocalPlannerNoObstacles(ConfSet *confs1, std::vector<int> *subset1, 
		ConfSet *confs2, std::vector<int> *subset2) = 0;

	/*	given two sets of configurations decides whether the local-planner
	movement between these subsets of configurations is possible	*/
	bool	testLocalPlannerNoObstacles(vector<ConfSet*> confs1, vector<vector<int>> *subset1, 
		vector<ConfSet*> confs2, vector<vector<int>> *subset2);

	/*	given two sets of configurations decides whether the local-planner
	movement between these subsets of configurations is possible	*/
	bool	testLocalPlanner(vector<ConfSet*> confs1, vector<vector<int>> *subset1, 
		vector<ConfSet*> confs2, vector<vector<int>> *subset2);

	/*	given start and goal positions for robot decides whether it 
		doesn't collide with obstacles	*/
	virtual		bool	testSingleLocalPlanner(Conf start, Conf target) = 0;

	////////////////////////
	// Help Functions
	////////////////////////

	/*	Given two points returns true if the squared distance between them
		is larger than the given value	*/
	bool isSquaredDistanceLarge(Conf p, Conf q, double num);

	/*	Returns true if the two robots do not collide while moving between two given
		positions each	*/
	bool isTwoRobotsClear(Conf ps, Conf pt, Conf qs, Conf qt);
	
	/*	Returns true if the two robots do not collide while moving between two given
		positions each	*/
	bool isTwoRobotsClearInexact(Conf ps, Conf pt, Conf qs, Conf qt);

	/*	Returns true if the two robots do not collide while moving between two given
		positions each	*/
	bool isTwoRobotsClearExact(Conf ps, Conf pt, Conf qs, Conf qt);

	////////////////////////
	// Data Members
	////////////////////////
	Number_type	m_radius; // radius of robots
	double		m_radius_double;
	Obstacles	*m_obs;	
	Stats*		m_stats;
};
