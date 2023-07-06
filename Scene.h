/*******************************************************************
*	File name: 		Scene.h
*	Description:	Declaration of the Scene class
*					The class wraps all the geometric data structures
*					needed for the motion planner for its processing
*					and for the GUI layer to display.
*					
*	Authors:		Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#ifndef MOTION_PLAN_SCENE_H
#define MOTION_PLAN_SCENE_H

#pragma once

#include <math.h>
#include <vector>
#include <QVector>
#include <QPointF>
#include <QPolygonF>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QKeyEvent>
#include <time.h>
#include "GraphicsEllipse.h"
#include <QPropertyAnimation>
#include "basic_typedef.h"
#include <QPen>
#include "colors.h"


const double POINT_RADIUS = 1.5;
const double DEFAULT_ROBOT_RADIUS = 20.0;
const double PRECISION = 25;
const int ANIMATION_STAGE_DURATION = 500;

class Scene : public QGraphicsScene
{
	Q_OBJECT

public:
	
	////////////////////////
	// C'tors & D'tors
	////////////////////////

	Scene(QObject * parent = 0);

	~Scene(void);


	////////////////////////
	// Access methods
	////////////////////////

	int Scene::getNumberRobots()
	{
		return m_vecRobots.size();
	}

	QVector<QVector<QPointF>>	getStartPositions()
	{
		return m_startPositions;
	}
	
	QVector<QVector<QPointF>>	getTargetPositions()
	{
		return m_targetPositions;
	}

	double				Scene::getRadius()
	{
		return m_robotRadius;
	}

	QVector<QPolygonF>	Scene::getObstacles()
	{
		QVector<QPolygonF> res;
		for (int i=0; i<m_vecObstacles.size(); i++)
		{
			QPolygonF poly = m_vecObstacles[i]->polygon();
			res.push_back(poly);
		}
		return res;
	}

	QPointF Scene::getRoomTopLeft()
	{
		return m_roomTL;
	}
	
	QPointF Scene::getRoomBottomRight()
	{
		return m_roomBR;
	}

	int getGroupCount()
	{
		return m_groupCount;
	}

	////////////////////////
	// Modifiers
	////////////////////////
	/*  Adds the point qp to the obstacle currently drawn */
	bool Scene::addPointToObstacle(QPointF qp);

	/*	If the current obstacle has at least one point, it turns it into polygon, 
		otherwise it deletes it	*/
	void Scene::createObstacle();

	/*  Adds the robot with the starting position to the scene */
	void Scene::addRobot(QPointF qp);

	/*  Adds the target position to the scene */
	void Scene::addTarget(QPointF qp);

	/* Changes the radius of the robots and updates the GUI accordingly */
	void Scene::setRadius(double radiusFactor);

	/*	Moves robots to positions conf */
	void Scene::updateRobotPositions(QVector<QPointF> conf);

	/*	Sets the room	*/
	void Scene::setRoom(QPointF topleft, QPointF bottomright);

	void Scene::setAnimationMultiplier(double multi){m_dAnimationMultiplier=multi;}

	void Scene::setPath(vector<vector<pair<QTPath,GeometricGraph>>> path)
	{
		m_vecPaths = path;
		if (!m_vecPaths.empty())
			m_bPathExists = true;
		else
			m_bPathExists = false;
	}

	void setGroup(int group) 
	{ 
		m_currentGroup = group; 
		if (m_currentGroup > m_groupCount - 1)
			m_groupCount = group + 1;

		// add necessary structures
	}


	////////////////////////
	// Other methods
	////////////////////////
	
	void animate();

	/* Update the positions of robots to conf */
	//void moveRobots(QVector<QPointF> conf);


	///////////////////////
	// Helper methods
	///////////////////////

	/* Returns a rectangle with center point qp and height/width = radius*2 */
	QRectF rectForPoint(QPointF qp, double radius);

	/* Removes all objects */
	void clearScene();

	/* Remove information of the current obstacle */
	void clearLeftovers();

	/* Remove information of the result */
	void clearResults();

	/* Remove information related to the animation */
	void clearAnimationObjects();

	void	saveToFile(const char* pi_szFileName);
	void	loadFromFile(const char* pi_szFileName);

	void	initiallize()
	{
		m_startPositions = QVector<QVector<QPointF>>(1);
		m_targetPositions = QVector<QVector<QPointF>>(1);
		m_vecRobots = QVector<QVector<QGraphicsEllipseItem*>>(1);
		m_vecTargets = QVector<QVector<QGraphicsEllipseItem*>>(1);
	}

public slots:
	virtual void mousePressEvent( QMouseEvent *e ){};

	void	displayGraph()
	{
		if (m_graphCount >= m_vecPaths.size())
			return;

		for (int g = 0; g < m_groupCount; g++)
		{
			// draw edge
			
			pair<Conf,Conf> p;

			QPen pen(GROUP_COLORS[g % NUM_COLORS], 1, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);

			foreach(p, ((m_vecPaths[m_graphCount + 1][g].second).first))
			{
				QLineF line(p.first.x() , p.first.y() , 
					p.second.x() , p.second.y() );
				QGraphicsLineItem* graphLine = new QGraphicsLineItem(line,0,this);
				graphLine->setPen(pen);
				addItem(graphLine);
				m_displayGraph.push_back(graphLine);
			}

			// draw vertex1
			Conf c;
			foreach(c, ((m_vecPaths[m_graphCount + 1][g].second).second))
			{
				QPointF qp(c.x() , c.y());
				QRectF rectangle = rectForPoint(qp,m_robotRadius / 3);
				QGraphicsEllipseItem* ellipse = new QGraphicsEllipseItem( 0, this);;
				ellipse->setRect(rectangle);
				m_displayGraphVertices.push_back(ellipse);
				QBrush brush(GROUP_COLORS[g % NUM_COLORS], ROBOT_STYLE);
				ellipse->setBrush(brush);
				addItem(ellipse);
			}
		}
		
		emit displayChanged();

		m_graphCount++;
	}

	void	hideGraph()
	{
		foreach(QGraphicsLineItem* line, m_displayGraph)
		{
			removeItem(line);
			delete line;
		}
		m_displayGraph.clear();

		foreach(QGraphicsEllipseItem* ell, m_displayGraphVertices)
		{
			removeItem(ell);
			delete ell;
		}
		m_displayGraphVertices.clear();

		emit displayChanged();
	}
	
	void	animationComplete()
	{
		clearAnimationObjects();
	}

signals: 
	void displayChanged();

	void animationDone();

protected:
	
	virtual void keyPressEvent (QKeyEvent* evt);

	////////////////////////
	// Data Members
	////////////////////////
public:

	//	obstacles information
	QVector<QPointF>				m_currentObstaclePoints;
	QVector<QGraphicsEllipseItem*>	m_currentObstacleEllipses;
	QVector<QGraphicsLineItem*>		m_currentObstacleLines;
	QVector<QGraphicsPolygonItem*>	m_vecObstacles;

	//	robots information
	QVector<QVector<QPointF>>				m_startPositions;
	QVector<QVector<QPointF>>				m_targetPositions;
	QVector<QVector<QGraphicsEllipseItem*>>	m_vecRobots;
	QVector<QVector<QGraphicsEllipseItem*>>	m_vecTargets;
	double									m_robotRadius;

	//	paths information
	vector<vector<pair<QTPath,GeometricGraph>>>		m_vecPaths;

	//	animation inforamtion	
	QVector<QVector<GraphicsEllipse*>>				m_vecAnimatedRobots;

	bool							m_bPathExists;
	double							m_dAnimationMultiplier;

	QPointF							m_roomTL;
	QPointF							m_roomBR;

	vector<QAbstractAnimation*>		m_animation;
	
	vector<QGraphicsLineItem*>		m_displayGraph;
	vector<QGraphicsEllipseItem*>	m_displayGraphVertices;
	int								m_graphCount;

	int								m_currentGroup;
	int								m_groupCount;
};

#endif