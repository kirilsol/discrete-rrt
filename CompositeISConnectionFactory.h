#include "CompositeConnectionFactory.h"

class CompositeISConnectionFactory : public CompositeConnectionFactory
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	CompositeISConnectionFactory(CompositeConf* ec1, CompositeConf* ec2, vector<CompositeConfEdge>* ecel, 
		vector<int> robotNum, CollisionDetector* col, Stats* stats) 
			: CompositeConnectionFactory(ec1, ec2, ecel, robotNum, col, stats){}

	~CompositeISConnectionFactory(){};
	
	void	print_ip(int num_edges, vector<vector<int>> interferences);


	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateEdges();

	////////////////////////
	// Data Members
	////////////////////////
};
