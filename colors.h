#include <qcolor.h>
#include <qbrush.h>

const int NUM_COLORS = 10;

const Qt::GlobalColor GROUP_COLORS[] = {Qt::GlobalColor::blue,
	Qt::GlobalColor::red, Qt::GlobalColor::magenta, Qt::GlobalColor::green, 
	Qt::GlobalColor::yellow, Qt::GlobalColor::cyan, Qt::GlobalColor::darkBlue, 
	Qt::GlobalColor::darkGreen, Qt::GlobalColor::darkRed,  Qt::GlobalColor::darkYellow};

const Qt::BrushStyle	ROBOT_STYLE		= Qt::SolidPattern;

const Qt::BrushStyle	TARGET_STYLE	= Qt::DiagCrossPattern;

const Qt::GlobalColor	OBSTACLE_COLOR	= Qt::GlobalColor::lightGray;

const Qt::BrushStyle	OBSTACLE_STYLE	= Qt::SolidPattern;