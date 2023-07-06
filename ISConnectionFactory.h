#include "ConnectionFactory.h"

class ISConnectionFactory : public ConnectionFactory
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	ISConnectionFactory(ExtendedConf* ec1, ExtendedConf* ec2, vector<ExtendedConfEdge>* ecel, 
		int robotNum, CollisionDetector* col, Stats* stats) 
			: ConnectionFactory(ec1, ec2, ecel, robotNum, col, stats){}

	~ISConnectionFactory(){};

	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateEdges();

	////////////////////////
	// Data Members
	////////////////////////
};
