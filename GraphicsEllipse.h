/*******************************************************************
*	File name: 		QGraphicsPolygon.h
*	Description:	Declaration of the GraphicsEllipse class.
*					This class is used to animate an ellipse of the scene.
*					It does so by declaring a private property of the inherited QGraphicsEllipseItem.
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include <QObject>
#include <QGraphicsEllipseItem>
#include <QRectF>
#include <QDragMoveEvent>
#include <QDropEvent>

class GraphicsEllipse : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT
    Q_PROPERTY(QRectF Rect WRITE setEllipseProperty READ ellipseProperty)

public:
    explicit GraphicsEllipse(QObject *parent = 0);
	
    void setEllipseProperty(const QRectF &rect) {setRect(rect);}
    QRectF ellipseProperty(){return rect();}
signals:

protected slots:
	virtual void dragMoveEvent ( QDragMoveEvent * event ){
		event->accept();
	}
	virtual void dropEvent ( QDropEvent*  event ){
		
	}

};
