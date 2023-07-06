#include "ConnectionFactory.h"

class SillyConnectionFactory : public ConnectionFactory
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	SillyConnectionFactory(ExtendedConf* ec1, ExtendedConf* ec2, vector<ExtendedConfEdge>* ecel, 
		int robotNum, CollisionDetector* col, Stats* stats) 
			: ConnectionFactory(ec1, ec2, ecel, robotNum, col, stats){}

	~SillyConnectionFactory(){};

	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateEdges();

	////////////////////////
	// Data Members
	////////////////////////
};
