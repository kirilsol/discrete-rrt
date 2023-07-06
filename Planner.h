/*******************************************************************
*	File name: 		Planner.h
*	Description:	Declaration of the Planner class
*					This class implements the motion planning algorithm
*					and communicates with a Scene object to get the input
*					from the gui and return the result.
*
*	Authors:		Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "Scene.h"
#include <QPointF>
#include <QVector>
#include <QPolygonF>
#include "basic_typedef.h"
#include "CompositeConf.h"
#include "basic_typedef.h"
#include "Sampler.h"
#include "SimpleCDwObstacles.h"
//#include "Stats.h"

struct ExtPath
{
	ExtendedConf* ec;
	vector<int>*  sub;
};

struct CompPath
{
	CompositeConf* cc;
	vector<vector<int>>*  sub;
};

class Planner
{
public:
	
	////////////////////////
	// C'tors & D'tors
	////////////////////////

	Planner(Scene* scene);

	~Planner(void);

	////////////////////////
	// modifiers
	////////////////////////

	void run();

	////////////////////////
	// helpers
	////////////////////////

	list<CompositeConf*>*		sampleCompositeConfigurations();

private:
	Point_2		transformToCGAL(QPointF qp)
	{
		// x and y coordinates of the point
		double x = qp.x();
		double y = qp.y();

		Point_2 cpoint(x,y);
		return cpoint;
	}

	QPointF		transformToQT(Conf cp)
	{
		return QPointF(cp.x() , cp.y() );
	}

	QVector<QPointF> transformToQT(ConfSet cs)
	{
		QVector<QPointF> result;

		foreach(Conf c, cs)
		{
			result.push_back(transformToQT(c));
		}
		return result;
	}

	QVector<QVector<QPointF>> transformToQT(Path path)
	{
		QVector<QVector<QPointF>> result;

		foreach(ConfSet cs, path)
		{
			result.push_back(transformToQT(cs));
		}
		return result;
	}

	void		generateExtConfs();

	void		connectExtConfs();

	void		generateStartTargetExt();

	void		constructRoadmapPath(vector<CompPath>* result);

	void		test();


	///////////////////////
	// Data members
	///////////////////////

	Scene*				m_scene;
	double				m_radius;
	Obstacles			m_obstacles;
	vector<int>			m_robotNum;
	vector<ConfSet>		m_startConfSet, m_targetConfSet;
	CompositeConf*		m_startExt;
	CompositeConf*		m_targetExt;
	vector<CompositeConf>		m_extConfSet;
	vector<CompositeConfEdge>	m_extEdgeSet;
	Room						m_room;

	SimpleCDwObstacles*	m_collision;
	Sampler*					m_sampler;

	IDFactory					m_extConfCounter;

	bool						m_isBasic;

	int							m_startID, m_targetID;

	Stats						m_stats;

	int							m_groups;

};


