#include "SillyConnectionFactory.h"

void	SillyConnectionFactory::generateEdges()
{
	for (int k = 0; k < m_connNum; k++)
	{
		vector<int> ss1 = m_ec1->generateRandomSubset(m_robotNum); 
		vector<int> ss2 = m_ec2->generateRandomSubset(m_robotNum);


		bool canConnect = m_col->testLocalPlanner(m_cs1,&ss1,m_cs2,&ss2);
		if (!canConnect)
			continue;

		ExtendedConfEdge ece;
		ece.ex1 = m_ec1;
		ece.ex2 = m_ec2;
		ece.vert1 = ss1;
		ece.vert2 = ss2;

		m_edges->push_back(ece);

		if (m_basic)
			break;
	}
}