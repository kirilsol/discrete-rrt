#include "RoadmapSolver.h"
#include "Tree.h"

class RoadmapSolverTree : public RoadmapSolver
{
public:
	map<int,Tree*> m_treeMap;
	Tree* m_root;
	
	////////////////////////
	// CTORS
	////////////////////////

	RoadmapSolverTree(Edges* edges);
	~RoadmapSolverTree(void);

	////////////////////////
	// Queries
	////////////////////////

	/*	Returns a roadmap path from start to target	*/
	virtual		MoveRecorder	calculatePath(list<int> startPos, list<int> targetPos);
};