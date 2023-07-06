#include "Connector.h"

class BrutForceConnector : public Connector
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	BrutForceConnector(vector<CompositeConf>* ecl, vector<CompositeConfEdge>* ecel, 
		CompositeConf* start, CompositeConf* target, vector<int> robotNum, CollisionDetector* col, Stats* stats)  
		: Connector(ecl, ecel, start, target, robotNum, col, stats){};
	~BrutForceConnector(){}

	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateExtConfEdges();

	virtual		void	connectStartTarget();

	////////////////////////
	// Data Members
	////////////////////////
	
};
