#include "CollisionDetector.h"

////////////////////////
// CTORS
////////////////////////

CollisionDetector::CollisionDetector(double radius, Obstacles *obs, Stats* stats) 
	: m_radius(radius), m_radius_double(radius), m_obs(obs), m_stats(stats){}

CollisionDetector::~CollisionDetector(void)
{

}

/*	given two sets of configurations decides whether the local-planner
	movement between these subsets of configurations is possible	*/
bool	CollisionDetector::testLocalPlanner(vector<ConfSet*> confs1, vector<vector<int>> *subset1, 
		vector<ConfSet*> confs2, vector<vector<int>> *subset2)
{
	/*	TODO: this method doesn't work correctly as it doesn't 
		check interferences between robots from different groups */ 

	for (int g = 0; g < confs1.size(); g++)
	{
		bool res = testLocalPlanner(confs1[g], &((*subset1)[g]), 
			confs2[g], &((*subset2)[g]));
		if (!res)
			return false;
	}
	return true;
}

/*	given two sets of configurations decides whether the local-planner
	movement between these subsets of configurations is possible	*/
bool	CollisionDetector::testLocalPlannerNoObstacles(vector<ConfSet*> confs1, vector<vector<int>> *subset1, 
		vector<ConfSet*> confs2, vector<vector<int>> *subset2)
{
	for (int g = 0; g < confs1.size(); g++)
	{
		bool res = testLocalPlannerNoObstacles(confs1[g], &((*subset1)[g]), 
			confs2[g], &((*subset2)[g]));
		if (!res)
			return false;
	}
	return true;
}

////////////////////////
// Help Functions
////////////////////////

/*	Given two points returns true if the squared distance between them
	is larger than the given value	*/
bool CollisionDetector::isSquaredDistanceLarge(Conf p, Conf q, double num)
{
	return CGAL::Comparison_result::LARGER == 
			CGAL::compare_squared_distance(p, q, num);
}

/*	Returns true if the two robots do not collide while moving between two given
	positions each	*/
bool CollisionDetector::isTwoRobotsClearInexact(Conf ps, Conf pt, Conf qs, Conf qt)
{
	// Assuming the first robot moves at constant speed from u_pos1 to v_pos1
    // while the second one moves at constant speed from u_pos2 to v_pos2
    // from time t = 0 to time t = 1, find the time when the distance between
    // them is minimal.
    // The squared distance D between the robots at time t is given by:
    //   D(t) = alpha*t^2 + 2*beta*t + gamma.
	const double ux1 = ps.x() ;
    const double uy1 = ps.y() ;
    const double vx1 = pt.x() ;
    const double vy1 = pt.y() ;
    const double ux2 = qs.x() ;
    const double uy2 = qs.y() ;
    const double vx2 = qt.x() ;
    const double vy2 = qt.y() ;
    const double dx = ux2 - ux1, dy = uy2 - uy1;
    const double rx1 = vx1 - ux1, rx2 = vx2 - ux2;
    const double ry1 = vy1 - uy1, ry2 = vy2 - uy2;
    const double dlx = rx2 - rx1, dly = ry2 - ry1;
    const double alpha = dlx*dlx + dly*dly;
    const double beta = dx*dlx + dy*dly;
    const double gamma = dx*dx + dy*dy;
    const double t_min = - beta / alpha;

    if ((t_min > 0) && (t_min < 1))
    {
		// We found a valid value 0 < t_min < 1, so the minimal squared distance
		// is given by D(t_min).
		const Number_type   min_sqr_dist = (alpha*t_min + 2*beta)*t_min + gamma;
		bool rc = (CGAL::compare(CGAL::square(m_radius + m_radius),
								min_sqr_dist) == CGAL::SMALLER);
		return rc;
    }

    return true;
}

/*	Returns true if the two robots do not collide while moving between two given
	positions each	*/
bool CollisionDetector::isTwoRobotsClearExact(Conf ps, Conf pt, Conf qs, Conf qt)
{
	// Assuming the first robot moves at constant speed from u_pos1 to v_pos1
    // while the second one moves at constant speed from u_pos2 to v_pos2
    // from time t = 0 to time t = 1, find the time when the distance between
    // them is minimal.
    // The squared distance D between the robots at time t is given by:
    //   D(t) = alpha*t^2 + 2*beta*t + gamma.
    const Number_type ux1 = ps.x();
    const Number_type uy1 = ps.y();
    const Number_type vx1 = pt.x();
    const Number_type vy1 = pt.y();
    const Number_type ux2 = qs.x();
    const Number_type uy2 = qs.y();
    const Number_type vx2 = qt.x();
    const Number_type vy2 = qt.y();
    const Number_type dx = ux2 - ux1, dy = uy2 - uy1;
    const Number_type rx1 = vx1 - ux1, rx2 = vx2 - ux2;
    const Number_type ry1 = vy1 - uy1, ry2 = vy2 - uy2;
    const Number_type dlx = rx2 - rx1, dly = ry2 - ry1;
    const Number_type alpha = dlx*dlx + dly*dly;
    const Number_type beta = dx*dlx + dy*dly;
    const Number_type gamma = dx*dx + dy*dy;
    const Number_type t_min = - beta / alpha;

    if ((CGAL::sign(t_min) == CGAL::POSITIVE) &&
        (CGAL::compare(t_min, 1) == CGAL::SMALLER))
    {
		// We found a valid value 0 < t_min < 1, so the minimal squared distance
		// is given by D(t_min).
		const Number_type   min_sqr_dist = (alpha*t_min + 2*beta)*t_min + gamma;
		bool rc = (CGAL::compare(CGAL::square(m_radius + m_radius),
								min_sqr_dist) == CGAL::SMALLER);
		return rc;
    }

    return true;
}

/*	Returns true if the two robots do not collide while moving between two given
	positions each	*/
bool CollisionDetector::isTwoRobotsClear(Conf ps, Conf pt, Conf qs, Conf qt)
{
	return isTwoRobotsClearInexact(ps,pt,qs,qt);
}
