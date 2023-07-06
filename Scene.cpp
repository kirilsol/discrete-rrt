/*******************************************************************
*	File name: 		Scene.h
*	Description:	Implementation of the Scene class
*					The class wraps all the geometric data structures
*					needed for the motion planner for its processing
*					and for the GUI layer to display.
*					
*	Authors:		Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "Scene.h"
#include <QPropertyAnimation>
#include <iostream>
#include <fstream>
#include <QDataStream>
#include <qcolor.h>
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>

////////////////////////
// C'tors & D'tors
////////////////////////

Scene::Scene(QObject * parent ) : QGraphicsScene(parent)
{
	m_robotRadius = DEFAULT_ROBOT_RADIUS;
	m_bPathExists = false;
	m_dAnimationMultiplier = 1.0;
	m_currentGroup = 0;
	initiallize();
	m_groupCount = 1;
}

Scene::~Scene(void)
{
	clearScene();
}

	////////////////////////
	// Access methods
	////////////////////////

	////////////////////////
	// Modifiers
	////////////////////////

/*	If the current obstacle has at least one point, it turns it into polygon, 
	otherwise it deletes it	*/
void Scene::createObstacle()
{
	if (m_currentObstaclePoints.size() > 1)
	{
		QPolygonF polygon(m_currentObstaclePoints);
		QGraphicsPolygonItem* item = new QGraphicsPolygonItem(0, this);
		m_vecObstacles.push_back(item);
		item->setBrush(QBrush(OBSTACLE_COLOR));
		item->setPolygon(polygon);
		item->setActive(true);
		item->setZValue(1);
		item->setFlag( QGraphicsItem::ItemIsMovable );
	}
	
	clearLeftovers();
}
	
/* Adds the point qp to the obstacle currently drawn */
bool Scene::addPointToObstacle(QPointF qp)
{	
	QPointF qplast,qpfirst;
	bool noPoints = m_currentObstaclePoints.empty();

	//extract last/first points if needed
	if (!noPoints){
		qplast = m_currentObstaclePoints.last();
		qpfirst = m_currentObstaclePoints.first();
	}

	// check if the new point is the first point, i.e the obstacle is closed
	qreal distance = (qp.rx()-qpfirst.rx())*(qp.rx()-qpfirst.rx()) 
		+ (qp.ry()-qpfirst.ry())*(qp.ry()-qpfirst.ry());

	if ((distance < PRECISION) && !noPoints)
	{
		createObstacle();
		return true;
	}
	else
	{
		// Save current point
		m_currentObstaclePoints.push_back(QPointF(qp));
		
		// Add ellipse to the output
		QRectF rectangle = rectForPoint(qp,POINT_RADIUS);
		QGraphicsEllipseItem* ellipse = new QGraphicsEllipseItem( 0, this);;
		ellipse->setRect(rectangle);
		m_currentObstacleEllipses.push_back(ellipse);
		addItem(ellipse);

		if (!noPoints)
		{
			QLineF line(qplast,qp);
			QGraphicsLineItem*	lineItem = new QGraphicsLineItem(line,0,this);
			m_currentObstacleLines.push_back(lineItem);
			addItem(lineItem);
		}
		return false;
	}
}

/*  Adds the robot with the starting position to the scene */
void Scene::addRobot(QPointF qp)
{
	if (m_startPositions.size() < m_groupCount)
	{	
		QVector<QPointF> dummy;
		m_startPositions.push_back(dummy);
		m_targetPositions.push_back(dummy);

		QVector<QGraphicsEllipseItem*> dummy2;
		m_vecRobots.push_back(dummy2);
		m_vecTargets.push_back(dummy2);
	}
	
	m_startPositions[m_currentGroup].push_back(qp);
	
	// Add big ellipse to the output
	QRectF rectangle = rectForPoint(qp,m_robotRadius);
	QGraphicsEllipseItem* ellipse = new QGraphicsEllipseItem( 0, this);;
	ellipse->setRect(rectangle);
	ellipse->setBrush(QBrush(GROUP_COLORS[m_currentGroup % NUM_COLORS], ROBOT_STYLE));
	m_vecRobots[m_currentGroup].push_back(ellipse);
	addItem(ellipse);
}

/*  Adds the target position to the scene */
void Scene::addTarget(QPointF qp)
{
	m_targetPositions[m_currentGroup].push_back(qp);
	
	// Add ellipse to the output
	qreal  xCord = qp.rx();
	qreal  yCord = qp.ry();
	QRectF rectangle = rectForPoint(qp,m_robotRadius);
	QGraphicsEllipseItem* ellipse = new QGraphicsEllipseItem( 0, this);;
	ellipse->setRect(rectangle);
	QBrush brush(GROUP_COLORS[m_currentGroup % NUM_COLORS], TARGET_STYLE);
	ellipse->setBrush(brush);
	m_vecTargets[m_currentGroup].push_back(ellipse);
	addItem(ellipse);
}

/* Changes the radius of the robots and updates the GUI accordingly */
void Scene::setRadius(double radiusFactor){
	m_robotRadius = DEFAULT_ROBOT_RADIUS * radiusFactor;

	for (int i=0; i< m_vecRobots.size(); i++)
	{
		for (int j = 0; j < m_vecRobots[i].size(); j++)
		{
			QRectF rectangles = rectForPoint((m_startPositions[i])[j],m_robotRadius);
			QRectF rectanglet = rectForPoint((m_targetPositions[i])[j],m_robotRadius);
			(m_vecRobots[i])[j]->setRect(rectangles);
			(m_vecTargets[i])[j]->setRect(rectanglet);
		}
	}

	emit displayChanged();
}

/*	Sets the room	*/
void Scene::setRoom(QPointF topleft, QPointF bottomright)
{
	m_roomTL = topleft;
	m_roomBR = bottomright;
}

///////////////////////
// Helper methods
///////////////////////

/* Returns a rectangle with center point qp and height/width = radius*2 */
QRectF Scene::rectForPoint(QPointF qp, double radius){
	qreal  xCord = qp.rx() - radius;
	qreal  yCord = qp.ry() - radius;
	QRectF rectangle( xCord,  yCord,  radius*2, radius*2);
	return rectangle;
}

/* Removes all objects */
void Scene::clearScene()
{
	clearResults();

	// remove obstacles
	while (!m_vecObstacles.empty())
	{
		QGraphicsPolygonItem* poly = m_vecObstacles.last();
		removeItem(poly);
		delete poly;
		m_vecObstacles.pop_back();
	}

	// remove robots
	m_startPositions.clear();
	m_targetPositions.clear();

	for (int i = 0; i < m_vecRobots.size(); i++)
	{
		for (int j = 0; j < m_vecRobots[i].size(); j++)
		{
			QGraphicsEllipseItem* ellipse1 = (m_vecRobots[i])[j];
			removeItem(ellipse1);
			//delete(ellipse1);

			QGraphicsEllipseItem* ellipse2 = (m_vecTargets[i])[j];
			removeItem(ellipse2);
			//delete(ellipse2);
		}
	}

	m_vecRobots.clear();
	m_vecTargets.clear();
}

void Scene::clearLeftovers()
{
	// clear leftovers
	m_currentObstaclePoints.clear();
	
	while (!m_currentObstacleEllipses.empty())
	{
		QGraphicsEllipseItem* ellipse = m_currentObstacleEllipses.last();
		removeItem(ellipse);
		delete ellipse;
		m_currentObstacleEllipses.pop_back();
	}

	while (!m_currentObstacleLines.empty())
	{
		QGraphicsLineItem* line = m_currentObstacleLines.last();
		removeItem(line);
		delete line;
		m_currentObstacleLines.pop_back();
	}
}

void Scene::clearResults()
{
	//	remove current path
	m_vecPaths.clear();
	m_bPathExists = false;
}

/* Remove information related to the animation */
void Scene::clearAnimationObjects()
{
	for (int i=0; i<m_vecAnimatedRobots.size();i++)
	{
		for (int j=0; j<m_vecAnimatedRobots[i].size();j++)
		{
			removeItem((m_vecAnimatedRobots[i])[j]);
			delete (m_vecAnimatedRobots[i])[j];
		}
	}

	m_vecAnimatedRobots.clear();

	while (!m_animation.empty())
	{
		QAbstractAnimation* anim = m_animation[m_animation.size() - 1];
		m_animation.pop_back();
		//delete anim;
	}

	m_animation.clear();

	// hide start position
	for (int g = 0; g < m_groupCount; g++)
	{
		for (int r = 0; r < m_vecRobots[g].size(); r++)
		{
			addItem(m_vecRobots[g][r]);
		}
	}
}

	////////////////////////
	// Other methods
	////////////////////////

void Scene::animate()
{
	QPointF* currTarget;

	int totalTime;

	m_graphCount = 0;
	
	ConfSet	dummyCS(0);
	vector<pair<Conf,Conf>> dummyEdges(0);
	GeometricGraph dummygeo(dummyEdges, dummyCS);
	// m_vecPaths = vector<vector<pair<QTPath,GeometricGraph>>>(1);
	if (!m_bPathExists)
	{
		m_vecPaths = vector<vector<pair<QTPath,GeometricGraph>>>(1);
		for (int i = 0; i < m_groupCount; i++)
		{
			QTPath dummyqtpath(0);
			dummyqtpath.push_back(m_startPositions[i]);
			dummyqtpath.push_back(m_targetPositions[i]);
			dummyqtpath.push_back(m_startPositions[i]);
			m_vecPaths[0].push_back(pair<QTPath,GeometricGraph>(dummyqtpath, dummygeo));
		}
	}
	
	// create animated robots for display

	m_vecAnimatedRobots = QVector<QVector<GraphicsEllipse*>>(m_groupCount);

	vector<vector<GraphicsEllipse*>> ellipses(m_groupCount);
	for (int g = 0; g < m_groupCount; g++)
	{
		for (int r = 0; r < m_vecRobots[g].size(); r++)
		{
			//Create graphics poly for animation
			GraphicsEllipse* currEllipse = new GraphicsEllipse();
			QPointF initialPos = ((m_vecPaths[0][g].first)[0])[r];
			QRectF currRect = rectForPoint(initialPos, m_robotRadius);
			currEllipse->setRect(currRect);
			currEllipse->setBrush(QBrush(GROUP_COLORS[g % NUM_COLORS], ROBOT_STYLE));

			//Store it locally for later clean up.
			m_vecAnimatedRobots[g].push_back(currEllipse);
			//Add To Scene the Graphic Polygon
			addItem(currEllipse);
			ellipses[g].push_back(currEllipse);
		}
	}

	// hide start position
	int totalRobots = 0;
	for (int g = 0; g < m_groupCount; g++)
	{
		for (int r = 0; r < m_vecRobots[g].size(); r++)
		{
			removeItem(m_vecRobots[g][r]);
			totalRobots ++;
		}
	}

	emit displayChanged();

	QSequentialAnimationGroup* finalAnimation = new QSequentialAnimationGroup;
	QObject::connect(finalAnimation,SIGNAL( finished() ),this,SLOT( animationComplete() ));

	for (int pathCount = 1; pathCount < m_vecPaths.size(); pathCount ++)
	{	
		//Animate each group

		QSequentialAnimationGroup* pathSectionAnimation = new QSequentialAnimationGroup;
		QParallelAnimationGroup* pathSectionLocalPlannerAnimation = new QParallelAnimationGroup;
		QSequentialAnimationGroup* pathSectionRoadmapAnimation = new QSequentialAnimationGroup;
		QObject::connect(pathSectionLocalPlannerAnimation,SIGNAL( finished() ),this,SLOT( displayGraph() ));

		int currentRobot = 0;
		int degenerateRobots = 0;

		for (int g = 0; g < m_groupCount; g++)
		{
			QSequentialAnimationGroup* groupAnimation = new QSequentialAnimationGroup;
			//Animate each robot

			QVector<QVector<QPointF>> currPathSection = m_vecPaths[pathCount][g].first;

			if (pathCount > 0)
			{	
				// add last section from previous path
				QVector<QVector<QPointF>> prev = m_vecPaths[pathCount - 1][g].first;
				QVector<QPointF> lastSection = prev[prev.size() - 1];
				currPathSection.insert(currPathSection.begin(), lastSection);
			}


			QParallelAnimationGroup* roadmapAnimation = new QParallelAnimationGroup;

			if (g == m_groupCount -1 )
				QObject::connect(roadmapAnimation,SIGNAL( finished() ),this,SLOT( hideGraph() ));
			
			for(int r=0; r < m_vecRobots[g].size(); r++)
			{
				currentRobot++;
				QVector<QPointF> currMilestones;

				//Map to Scene
				for(int i=0; i < currPathSection.size(); i++)
				{
					QPointF p = (currPathSection[i])[r];
					currMilestones.push_back(p);
				}

				//Create graphics poly for animation
				GraphicsEllipse* currEllipse = ellipses[g][r];
				QRectF currRect = rectForPoint(currMilestones[0], m_robotRadius);
				currEllipse->setRect(currRect);

				//	Create property animation for this robot moving from previous roadmap 
				//	to current
				QPropertyAnimation* propAnim1 = new QPropertyAnimation(currEllipse, "Rect");

				if (pathCount != m_vecPaths.size() - 1)
				{
					propAnim1->setStartValue(rectForPoint(currMilestones[0], m_robotRadius));
					propAnim1->setEndValue(rectForPoint(currMilestones[1], m_robotRadius));
					propAnim1->setDirection(QAbstractAnimation::Forward);
					propAnim1->setDuration(1.0 * ANIMATION_STAGE_DURATION * 2);
				}
				else
				{
					propAnim1->setDuration(1.0 * ANIMATION_STAGE_DURATION * 6);
					
					QVariant msVar;
					msVar.setValue(rectForPoint(currMilestones[1], m_robotRadius));
					QVariant msVar0;
					msVar0.setValue(rectForPoint(currMilestones[0], m_robotRadius));
					propAnim1->setKeyValueAt(0,msVar0);
					propAnim1->setKeyValueAt(0.33,msVar);
					propAnim1->setKeyValueAt(1,msVar);
					
				}
				

				//Create property animation for this robot inside the roadmap movement
				QPropertyAnimation* propAnim2 = new QPropertyAnimation(currEllipse, "Rect");
				propAnim2->setStartValue(rectForPoint(currMilestones[1], m_robotRadius));

				QRectF currTarget;

				double msSize = currMilestones.size();
				if (msSize == 2)
				{
					degenerateRobots++;
					if (degenerateRobots == totalRobots)
					{
						currMilestones.push_back(currMilestones[1]);
						msSize++;
					}
				}

				totalTime = (msSize - 2) * m_dAnimationMultiplier * 2 * ANIMATION_STAGE_DURATION;

				propAnim2->setDuration(totalTime);

				//Create animation going lineary between each two milestones.
				for(int k=1; k < msSize; k++){
					//currSource = rectForPoint(currMilestones[k-1],m_robotRadius);
					currTarget = rectForPoint(currMilestones[k], m_robotRadius);
					QVariant msVar;

					msVar.setValue(currTarget);
					propAnim2->setKeyValueAt(((k+1)/msSize),msVar);

				}
				propAnim2->setDirection(QAbstractAnimation::Forward);

				pathSectionLocalPlannerAnimation->addAnimation(propAnim1);
				roadmapAnimation->addAnimation(propAnim2);

				m_animation.push_back(propAnim1);
				m_animation.push_back(propAnim2);
			}

			pathSectionRoadmapAnimation->addAnimation(roadmapAnimation);
			m_animation.push_back(roadmapAnimation);
		}
		pathSectionAnimation->addAnimation(pathSectionLocalPlannerAnimation);
		m_animation.push_back(pathSectionLocalPlannerAnimation);
		pathSectionAnimation->addAnimation(pathSectionRoadmapAnimation);
		m_animation.push_back(pathSectionRoadmapAnimation);
		finalAnimation->addAnimation(pathSectionAnimation);
		m_animation.push_back(pathSectionAnimation);
	}

	finalAnimation->start();
	m_animation.push_back(finalAnimation);
	
	if (!m_bPathExists)
		m_vecPaths.clear();
}

/* Update the positions of robots to conf */
//void Scene::moveRobots(QVector<QVector<QPointF>> confs)
//{
//	for (int i=0; i < m_vecRobots.size(); i++)
//	{
//		QRectF rect = rectForPoint(conf[i],m_robotRadius);
//		m_vecRobots[i]->setRect(rect);
//	}
//}


void Scene::keyPressEvent (QKeyEvent* evt){
	
	bool isDeleteKey (evt->key() == Qt::Key_Delete);

}

// Save scene to file
void	Scene::saveToFile(const char* pi_szFileName)
{
	FILE* fOutput = fopen(pi_szFileName, "w+");
	if (!fOutput)
	{
		return;
	}
	
	// scene's bounding rectangle
	fprintf(fOutput, "%.2f %.2f %.2f %.2f\n",
					m_roomTL.x(),m_roomTL.y(),
					m_roomBR.x(), m_roomBR.y());

	// obstacles
	int nNumOfObstacles = m_vecObstacles.size();
	fprintf(fOutput, "%d\n", nNumOfObstacles);

	for (int nIndex = 0; nIndex < nNumOfObstacles; nIndex++)
	{
		
		QGraphicsPolygonItem* obs = m_vecObstacles[nIndex];
		QPolygonF poly = obs->polygon();
		int size = poly.size();

		if (size > 0)
		{
			fprintf(fOutput, "%d\n", size);
		}

		for (int mIndex = 0; mIndex < size; mIndex++)
		{
			double dx = poly[mIndex].rx();
			double dy = poly[mIndex].ry();

			fprintf(fOutput, "%.2f %.2f ", dx, dy);
		}

		if (size > 0)
		{
			fprintf(fOutput, "\n");
		}
	}

	// write every group and its path

	fprintf(fOutput, "\n %d %.2f \n", m_groupCount, m_robotRadius);

	for (int g = 0; g < m_groupCount; g++)
	{
		// robots
		int nNumOfRobots = m_startPositions[g].size();
		fprintf(fOutput, "\n %d ", nNumOfRobots);

		//write source and targets
		for (int nIndex = 0; nIndex < nNumOfRobots; nIndex++){
			QPointF source = m_startPositions[g][nIndex];
			// source
			fprintf(fOutput," %.2f %.2f ", source.x(), source.y());

			// target
			QPointF target = m_targetPositions[g][nIndex];

			fprintf(fOutput, " %.2f %.2f ", target.x(), target.y());
		}

		// path
		int length = m_vecPaths.size();
		fprintf(fOutput, "\n\n%d", length);
		if (m_bPathExists)
		{
			for (int i=0; i<length; i++)
			{
				QTPath subpath = m_vecPaths[i][g].first;
				GeometricGraph graph = m_vecPaths[i][g].second;

				// write subpath
				fprintf(fOutput, "\n\n%d\n", subpath.size());

				for (int j = 0; j < subpath.size(); j++)
				{
					QVector<QPointF> currConf = subpath[j];

					for (int r = 0; r < nNumOfRobots; r++)
					{
						fprintf(fOutput, " %.2f %.2f ", currConf[r].x(), currConf[r].y());
					}
				}

				// write graph

				// write edgeds

				int edgesNum = graph.first.size();
				int vertNum = graph.second.size();

				fprintf(fOutput, "\n%d\n", edgesNum);

				for (int j = 0; j < edgesNum; j++)
				{
					fprintf(fOutput, " %.2f %.2f %.2f %.2f ", 
						graph.first[j].first.x() , graph.first[j].first.y() ,
						graph.first[j].second.x() , graph.first[j].second.y() );
				}

				// write vertices

				fprintf(fOutput, "\n%d\n", vertNum);

				for (int j = 0; j < vertNum; j++)
				{
					fprintf(fOutput, " %.2f %.2f ", 
						graph.second[j].x() , graph.second[j].y() );
				}
			}
		} 
	}

	fclose(fOutput);
}

void	Scene::loadFromFile(const char* pi_szFileName)
{
	std::ifstream ifile(pi_szFileName);
	std::istream *in = &ifile; 
	
    qreal x1,y1,x2,y2;
	*in >> x1>> y1>> x2>> y2;
	m_roomTL = QPointF(x1,y1);
	m_roomBR = QPointF(x2,y2);
	
	// 2) Obstacles.
	int num_obstacles;
	*in >> num_obstacles;

	for (int i = 0; i < num_obstacles; i++)
	{
		// Create the current polygon.
		int num_points;
		*in >> num_points;

		double obsx,obsy;

		for (int j=0; j<num_points; j++)
		{
			*in >> obsx >> obsy;
			m_currentObstaclePoints.push_back(QPointF(obsx,obsy));
		}

		createObstacle();
	}

	*in >> m_groupCount;
	*in >> m_robotRadius;

	initiallize();

	m_currentGroup = -1;

	for (int g = 0; g < m_groupCount; g++)
	{
		// 3) Robots
		int num_robots;
		*in >> num_robots;
		m_currentGroup++; 

		for (int i = 0; i< num_robots; i++)
		{
			double sx,sy,tx,ty;
			*in >> sx >> sy >> tx >> ty;

			QPointF s(sx,sy);
			addRobot(s);

			QPointF t(tx,ty);
			addTarget(t);
		}

		// 4) Paths
		int length;
		*in >> length;

		if (length > 0)
		{
			if (g == 0)
			{
				m_vecPaths = vector<vector<pair<QTPath,GeometricGraph>>>(length);
				m_bPathExists = true;
			}

			for (int i = 0; i < length; i++)
			{

				// read subpath
				QVector<QVector<QPointF>> subpath; 

				int subpathLength;
				*in >> subpathLength;

				for (int j = 0; j < subpathLength; j++)
				{
					QVector<QPointF> subPos;

					for (int r = 0; r < num_robots; r++)
					{
						double x,y;

						*in >> x >> y;

						QPointF p(x,y);

						subPos.push_back(p);
					}

					subpath.push_back(subPos);
				}

				//read edges

				int edgesNum;
				*in >> edgesNum;

				vector<pair<Conf,Conf>> edges;

				for (int j = 0; j < edgesNum; j++)
				{
					double x1,y1,x2,y2;

					*in >> x1 >> y1 >> x2 >> y2;
					Conf p1(x1,y1);
					Conf p2(x2,y2);

					edges.push_back(pair<Conf,Conf>(p1,p2));
				}

				//read vertices
				vector<Conf> vertices;

				int vertNum;
				*in >> vertNum;

				for (int j = 0; j < vertNum; j++)
				{
					double x,y;
					*in >> x >> y;

					Conf c(x,y);
					vertices.push_back(c);
				}

				GeometricGraph graph(edges,vertices);

				m_vecPaths[i].push_back(pair<QTPath, GeometricGraph>(subpath, graph));
			}
		}
	}

	ifile.close();
}


