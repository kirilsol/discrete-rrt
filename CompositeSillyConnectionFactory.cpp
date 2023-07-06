#include "CompositeSillyConnectionFactory.h"

void	CompositeSillyConnectionFactory::generateEdges()
{
	for (int k = 0; k < m_connNum; k++)
	{
		vector<vector<int>> ss1 = m_ec1->generateRandomSubset(m_robotNum); 
		vector<vector<int>> ss2 = m_ec2->generateRandomSubset(m_robotNum);

		ConfSet flatCS1 = m_ec1->getConfSubset(ss1);
		ConfSet flatCS2 = m_ec2->getConfSubset(ss2);
		vector<int>		flatIndices;
		
		for (int i = 0; i < flatCS1.size(); i++)
			flatIndices.push_back(i);

		bool canConnect = m_col->testLocalPlanner(&flatCS1,&flatIndices,&flatCS2,&flatIndices);
		if (!canConnect)
			continue;

		CompositeConfEdge ece;
		ece.ex1 = m_ec1;
		ece.ex2 = m_ec2;
		ece.vert1 = ss1;
		ece.vert2 = ss2;

		m_edges->push_back(ece);

		if (m_basic)
			break;
	}
}