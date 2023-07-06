/*******************************************************************
*	File name: 		MRMPDrawingBox.h
*	Description:	Declaration of the MRMPDrawingBox class.
*					This class is a component of the GUI main window.
*					It inherits QGraphicsView and it is used as the drawing area of the application.
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#ifndef OURDRAWINGBOX_H
#define OURDRAWINGBOX_H

#include <QWidget>
#include <QPointF>
#include <QPushButton>
#include <QVector>
#include <QString>
#include <QGraphicsEllipseItem>
#include <QDropEvent>
#include <QGraphicsView>
#include "ui_MRMPDrawingBox.h"
#include "Scene.h"
#include "Planner.h"

using namespace std;

class MRMPDrawingBox : public QGraphicsView
{
	Q_OBJECT

public:

	enum State {
		IDLE,
		PRESENTATION,
		DRAW_OBSTACLE,
		DRAW_ROBOT,
		DRAW_TARGET,
		ANIMATION
	};

	
	////////////////////////
	// C'tors & D'tors
	////////////////////////
	MRMPDrawingBox(QWidget *parent = 0);
	~MRMPDrawingBox();
	
	//Setter
	void setFileLocation(string fileLocation);
	//Getter
	int  getAnimationMultiplier() { /*return m_AnimationSpeedMultiplier; */}
	////////////////////////
	// Modifiers
	////////////////////////

public slots:
	/* slots, connected to signals from other classes */
	
	//	indicates that the value of radius slider changed 
	void radiusSliderChanged(int val){
		m_mpScene->setRadius(((double)val)/50.0);
		this->show();
	}
	
	//	orders the animation of paths
	void animateButtonPressed();

	//	calls refresh
	void refresh()
	{
		this->show();
	}

	//  Gets a signal from the animation objects and deletes proper content
	void animationComplete();

	//	tracks mouse press evens of user.
	virtual void mousePressEvent( QMouseEvent *e );
	//	indicates that the obstacles button is pushed.
	void drawObstaclesButtonPushed(); 
	//	indicates that the robots button is pushed.
	void drawRobotsButtonPushed();
	//	clear scene.
	void clear();
	//	clear scene results.
	void clearResults();
	//	loads scene
	void loadScene(QString qfile); 
	//	saves scene
	void saveScene(QString qfile);
	//	setter of the animation multiplier
	void setAnimationMultiplier(int animationFactor) 
	{
		//m_curAnimationSpeed = DEFAULT_ANIMATION_SPEED *(double)(100 - animationFactor)/50; 
		m_mpScene->setAnimationMultiplier((double)(100 - animationFactor)/50);
	}
	
	// Invokes the motion planning algorithm
	void execute();

	void groupChanged(QString group);
	
signals:
	/* signals to emit to other classes about our changes */
	//void plannerFinished();
	
private:	

	// helper method
	QPolygonF converToSceneCords(QPolygonF& poly);

	// sets the scene configuration room
	void setSceneRoom();
	
	///////////////////
	//  DATA MEMBERS
	///////////////////

	int								m_curAnimationSpeed;
	
	Scene*							m_mpScene;

	Planner*						m_planner;

	Ui::MRMPDrawingBoxClass			ui;

	enum State						m_currentState;
};

#endif // OURDRAWINGBOX_H
