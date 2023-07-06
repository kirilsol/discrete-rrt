#pragma once

#include "circular_typedef.h"
#include "SimpleCollisionDetector.h"
#include <math.h>

class SimpleCDwObstacles : public SimpleCollisionDetector
{
public:
	SimpleCDwObstacles(double radius, Obstacles *obs, Stats* stats);
	
	~SimpleCDwObstacles(){}

	/*	checks if position pos of robot with radius size is 
		collision free with obstacles */
	virtual		bool	testSingleRobot(Conf pos);

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
	bool	testLocalPlannerNoObstacles(ConfSet *confs1, std::vector<int> *subset1, 
		ConfSet *confs2, std::vector<int> *subset2);

	/*	given start and goal positions for robot decides whether it 
		doesn't collide with obstacles	*/
	virtual		bool	testSingleLocalPlanner(Conf start, Conf target);


private:
	template <typename PointLocation>
	bool pointValid(PointLocation* pl, 
		const typename PointLocation::Arrangement_2::Point_2& p)
	{
		CGAL::Object	obj = pl->locate(p);

		typename PointLocation::Arrangement_2::Vertex_const_handle		v;
		typename PointLocation::Arrangement_2::Halfedge_const_handle	e;
		typename PointLocation::Arrangement_2::Face_const_handle		f;

		if (CGAL::assign(e, obj))
			return false;
		else if (CGAL::assign(v, obj))
			return false;
		else if (CGAL::assign(f, obj) && 
			(f->is_unbounded() || !(f->contained())))
		{
			return true;
		}
		return false;
	}

	Offset_arrangement	m_offArr;
	Arrangement_2		m_arr;
	//Offset_ric_pl		m_off_pl;
	Ric_pl				m_pl;
	static const bool	use_exact_check = false;

};
