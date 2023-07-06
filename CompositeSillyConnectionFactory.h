#include "CompositeConnectionFactory.h"

class CompositeSillyConnectionFactory : public CompositeConnectionFactory
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	CompositeSillyConnectionFactory(CompositeConf* ec1, CompositeConf* ec2, vector<CompositeConfEdge>* ecel, 
		vector<int> robotNum, CollisionDetector* col, Stats* stats) 
			: CompositeConnectionFactory(ec1, ec2, ecel, robotNum, col, stats){}

	~CompositeSillyConnectionFactory(){};

	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateEdges();

	////////////////////////
	// Data Members
	////////////////////////
};
