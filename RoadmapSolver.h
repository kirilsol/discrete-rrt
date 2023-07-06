#include "basic_typedef.h"

class RoadmapSolver
{
public:
	Edges* m_edges;
	MoveRecorder m_recorder;
	bool		m_degenerate;
	
	///////////////////////
	// CTORS
	////////////////////////

	RoadmapSolver(Edges* edges)
	{
		m_edges = edges;
		m_degenerate = m_edges->empty();
	}

	~RoadmapSolver(){}

	////////////////////////
	// Queries
	////////////////////////

	/*	Returns a roadmap path from start to target	*/
	virtual		MoveRecorder	calculatePath(list<int> startPos, list<int> targetPos) = 0;

	bool		degenerate()
	{
		return m_degenerate;
	}
};