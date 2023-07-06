#include "SimpleCDwObstacles.h"
#include <CGAL/approximated_offset_2.h>
#include <CGAL/minkowski_sum_2.h>

SimpleCDwObstacles::SimpleCDwObstacles(double radius, Obstacles *obs, Stats* stats)
		: SimpleCollisionDetector(radius,obs, stats)
{
	// Construct exact point location
	//Offset_polygon_set_2 offset_set;
	//double error_bound = radius / 3.0;

	//for (Obstacles::iterator iter = m_obs->begin(); iter != m_obs->end(); iter++)
	//{
	//	Offset_polygon_with_holes_2	poly_wh = CGAL::approximated_offset_2(*iter, m_radius, error_bound); 
	//	Offset_polygon_2			poly = poly_wh.outer_boundary();
	//	offset_set.join(poly);
	//}

	//m_offArr = offset_set.arrangement();

	//m_off_pl = Offset_ric_pl(m_offArr);

	// Construct approximate point location with robot as bounding box	
	Polygon_set_2 set;
	
	Point_2		p1(-radius, -radius);
	Point_2		p2(-radius, radius);
	Point_2		p3(radius, radius);
	Point_2		p4(radius, -radius);

	Polygon_2   bounding_robot;
	bounding_robot.push_back(p1);
	bounding_robot.push_back(p2);
	bounding_robot.push_back(p3);
	bounding_robot.push_back(p4);

	for (Obstacles::iterator iter = m_obs->begin(); iter != m_obs->end(); iter++)
	{
		Polygon_with_holes_2	poly_wh = minkowski_sum_2(*iter, bounding_robot);
		Polygon_2				poly = poly_wh.outer_boundary();
		set.join(poly);
	}

	m_arr = set.arrangement();

	m_pl.attach(m_arr);
	int j = 1;
}

/*	checks if position pos of robot with radius size is 
	collision free with obstacles */
bool	SimpleCDwObstacles::testSingleRobot(Conf pos)
{
	if (pointValid<Ric_pl>(&m_pl, pos))
		return true;
	if (!use_exact_check)
		return false;
	/*Offset_arrangement::Point_2 off_p(pos.x(),pos.y());
	return pointValid<Offset_ric_pl>(&m_off_pl, off_p);*/
} 

/*	checks if the edge between conf1,conf2 is valid,
	i.e. a robot doesn't intersect with any other  configurations
	while moving from one configuration to another	*/
bool	SimpleCDwObstacles::testEdge(ConfSet *confs, int confind1, int confind2)
{
	if (!SimpleCollisionDetector::testEdge(confs, confind1, confind2))
		return false;

	return testSingleLocalPlanner((*confs)[confind1], (*confs)[confind2]);
}
	
/*	given start and goal positions for robot decides whether it 
doesn't collide with obstacles	*/
bool	SimpleCDwObstacles::testSingleLocalPlanner(Conf start, Conf target)
{
	double x1 = start.x();
	double y1 = start.y() ;
	double x2 = target.x() ;
	double y2 = target.y() ;

	double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2,2));

	if (distance <= m_radius_double)
		return true;

	double step_size = m_radius_double;
	int step_num = floor((distance - step_size) / step_size);
	double vx = x2 - x1;
	double vy = y2 - y1;

	for (int i = 1; i <= step_num; i++)
	{
		double offset =  (i * step_size) / (distance - step_size);
		double currx = x1 + vx * offset;
		double curry = y1 + vy * offset;

		Conf currentPos(currx, curry);

		if (!testSingleRobot(currentPos))
			return false;
	}

	return true;
}


/*	given two sets of configurations decides whether the local-planner
movement between these subsets of configurations is possible	*/
bool	SimpleCDwObstacles::testLocalPlanner(ConfSet *confs1, std::vector<int> *subset1, 
	ConfSet *confs2, std::vector<int> *subset2)
{
	if (!SimpleCollisionDetector::testLocalPlanner(confs1, subset1, confs2, subset2))
		return false;

	Conf s,t;
	for (int i = 0; i < subset1->size(); i++)
	{
		s = (*confs1)[(*subset1)[i]];
		t = (*confs2)[(*subset2)[i]];

		if (!testSingleLocalPlanner(s,t))
			return false;
	}

	return true;
}

/*	given two sets of configurations decides whether the local-planner
movement between these subsets of configurations is possible	*/
bool	SimpleCDwObstacles::testLocalPlannerNoObstacles(ConfSet *confs1, std::vector<int> *subset1, 
	ConfSet *confs2, std::vector<int> *subset2)
{
	bool res = SimpleCollisionDetector::testLocalPlanner(confs1, subset1, confs2, subset2);
	return res;
}
