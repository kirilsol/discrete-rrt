/*******************************************************************
*	File name: 		MRMPApplication.h
*	Description:	Declaration of the MRMPApplication class.
*					This class is the Main Window of the GUI application.
*					It handles the GUI display and its subcomponents.
*
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#ifndef QTTESTAPPL_H
#define QTTESTAPPL_H

#include <QtGui/QMainWindow>
#include "ui_MRMPApplication.h"

class MRMPApplication : public QMainWindow
{
	Q_OBJECT

////////////////////////
// C'tors & D'tors
////////////////////////

public:
	MRMPApplication(QWidget *parent = 0, Qt::WFlags flags = 0);
	~MRMPApplication();

	virtual bool notify(QObject *rec, QEvent *ev)
    {
        //qDebug() << "MRMPApplication::nofity";
        try {
            return MRMPApplication::notify(rec, ev);
        }
        catch( ... ) {
            //qDebug() << "Unknown Exception: Terminating!";
            exit(0);
        }
        return false;
    }
	
////////////////////////
// Modifiers
////////////////////////
	public slots:
	void	SaveFileAs();
	void    LoadFile();

	/////////////////////////////
// Other methods -- signals.
/////////////////////////////
	signals:
	
////////////////////////
// Data Members
////////////////////////

public slots:
	void executeComplete()
	{
		
		qApp->setQuitOnLastWindowClosed(true);
		qApp->quit();
		//this->close();
		exit(0);

	}

public:
	Ui::MRMPApplicationClass ui;

private :
	enum buttonState{drawState,doneState};

	buttonState drawObstaclesButton;
	buttonState drawRobotsButton;
	buttonState enterTargetButton;
	const double constOneMillion;
};

#endif // QTTESTAPPL_H
