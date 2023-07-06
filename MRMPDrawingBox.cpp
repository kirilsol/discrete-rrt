/*******************************************************************
*	File name: 		MRMPDrawingBox.cpp
*	Description:	Implementation of the MRMPDrawingBox class.
*					This class is a component of the GUI main window.
*					It inherits QGraphicsView and it is used as the drawing area of the application.
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "MRMPDrawingBox.h"
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QRect>
#include <QRectF>
#include <QGraphicsView>
#include <QPolygonF>
#include <QPropertyAnimation>
#include <QGraphicsPolygonItem>
#include <string>
#include <QPolygonF>
#include <QMessageBox>
#include <QMetaType>

Q_DECLARE_METATYPE(QPolygonF)

#pragma warning(disable: 4291 4503)
using namespace std;

const int DEFAULT_ANIMATION_SPEED = 500;

MRMPDrawingBox::MRMPDrawingBox(QWidget *parent)
	: QGraphicsView(parent)
{
	ui.setupUi(this);
	m_currentState = IDLE;
	m_mpScene=new Scene(this);
	this->setScene( m_mpScene );
	m_mpScene->setSceneRect(this->viewport()->geometry());
	QObject::connect(m_mpScene,SIGNAL(displayChanged()),this,SLOT(refresh()));
	QObject::connect(m_mpScene,SIGNAL(animationDone()),this,SLOT(animationComplete()));
	m_curAnimationSpeed = DEFAULT_ANIMATION_SPEED;
	m_planner = NULL;

	// force execute if the program is ran by a script
	std::ifstream ifile("nogui.txt");
	std::istream *in = &ifile; 
	bool flag_no_gui;
	*in >> flag_no_gui;

	if (flag_no_gui)
	{
		loadScene("input.txt");
		//execute();
	}

	ifile.close();
}

MRMPDrawingBox::~MRMPDrawingBox()
{
	clear();
	delete m_mpScene;
	delete m_planner;
}

void MRMPDrawingBox::animationComplete()
{

	m_currentState = IDLE;

}

//when our widget is pressed, and we are in a drawing modes we record point.
//and draw it by updating the widget.
void MRMPDrawingBox::mousePressEvent( QMouseEvent *e )
{
	QPointF sPos = mapToScene(e->posF().toPoint());
	setRenderHints(QPainter::Antialiasing);
	if (m_currentState == DRAW_OBSTACLE)
	{
		bool bres = m_mpScene->addPointToObstacle(sPos);
		if (bres)
			m_currentState = IDLE;	
	} 
	else if (m_currentState == DRAW_ROBOT)
	{
		m_mpScene->addRobot(sPos);
		m_currentState = DRAW_TARGET;
	} else if (m_currentState == DRAW_TARGET)
	{
		m_mpScene->addTarget(sPos);
		m_currentState = IDLE;
	}

	this->show();	
}

//	orders the animation of paths
void MRMPDrawingBox::animateButtonPressed()
{
	m_currentState = ANIMATION;
	
	m_mpScene->animate();
	
	this->show();
}

void MRMPDrawingBox::groupChanged(QString group)
{
	int intGroup = group.toInt();
	m_mpScene->setGroup(intGroup);
}

QPolygonF MRMPDrawingBox::converToSceneCords(QPolygonF& poly)
{
	QPolygonF f;
	return f;
}

//start recording points for a new obstacle.
void MRMPDrawingBox::drawObstaclesButtonPushed(){
	if (m_currentState == IDLE)
		m_currentState = DRAW_OBSTACLE;
	else if (m_currentState == DRAW_OBSTACLE)
	{
		m_currentState = IDLE;
		m_mpScene->createObstacle();
	}
}

//start recording points for a new robot.
void MRMPDrawingBox::drawRobotsButtonPushed(){
	m_currentState = DRAW_ROBOT;
}

void MRMPDrawingBox::clear(){
	m_mpScene->clearScene();
	this->update();
	this->show();
	if (!m_planner)
	{
		delete m_planner;
		m_planner = NULL;
	}
}
void MRMPDrawingBox::clearResults(){
	m_mpScene->clearResults();
	this->update();
	this->show();
	if (!m_planner)
	{
		delete m_planner;
		m_planner = NULL;
	}
}

//this will load scene from currFileLocation.
void MRMPDrawingBox::loadScene(QString qfile){
	m_currentState = IDLE;
	setRenderHints(QPainter::Antialiasing);
	m_mpScene->clear();
	m_mpScene->loadFromFile(qfile.toStdString().c_str());
	this->update();
}

//this will save scene to currFileLocation
void MRMPDrawingBox::saveScene(QString qfile){
	setSceneRoom();
	m_mpScene->saveToFile(qfile.toStdString().c_str());
}

void MRMPDrawingBox::setSceneRoom(){
		QVector<QPointF> pointVector;
		QRect windGeo=this->viewport()->geometry();

		m_mpScene->setRoom(this->mapToScene(windGeo.topLeft()),
			this->mapToScene(windGeo.bottomRight()));
}

// Invokes the motion planning algorithm
void MRMPDrawingBox::execute()
{
	// force execute if the program is ran by a script
	/*std::ifstream ifile("nogui.txt");
	std::istream *in = &ifile; 
	bool flag_no_gui;
	*in >> flag_no_gui;

	if (flag_no_gui)
	{
		m_mpScene->loadFromFile("input.txt");
	}*/
	setSceneRoom();
	m_planner = new Planner(m_mpScene);
	m_planner->run();
	m_mpScene->saveToFile("output.txt");
	update();
	QMessageBox msgBox;

	FILE* fOutput = fopen("status.txt", "w+");

	if (m_mpScene->m_vecPaths.empty())
	{
		msgBox.setText("#######################\n\nPlanner finished:\n Failure\n\n#######################");
		fprintf(fOutput, "%d", 0);
	}
	else
	{
		msgBox.setText("#######################\n\nPlanner finished:\n Success\n\n#######################");
		fprintf(fOutput,"%d", 1);
	}
	fclose(fOutput);

	//msgBox.exec();	

	std::ifstream ifile("nogui.txt");
	std::istream *in = &ifile; 
	bool flag_no_gui;
	*in >> flag_no_gui;

	if (flag_no_gui)
	{
		//QObject::connect(this,SIGNAL( plannerFinished() ),this->parent(),SLOT( executeComplete() ));
		//emit plannerFinished();
	}
	
}