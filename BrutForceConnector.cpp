#include "BrutForceConnector.h"
#include "CompositeISConnectionFactory.h"

void	BrutForceConnector::generateExtConfEdges()
{ 
	for (int i = 0; i < m_ec->size() -1; i++)
	{
		for (int j = i + 1; j < m_ec->size(); j++)
		{
			CompositeConnectionFactory* cf 
				= new CompositeISConnectionFactory(&((*m_ec)[i]), &((*m_ec)[j]), m_edges,
					m_robotNum, m_col, m_stats);
			
			cf->generateEdges();
			delete cf;
		}
	}
}

void	BrutForceConnector::connectStartTarget()
{ 
	vector<CompositeConf*> stEConfs;
	stEConfs.push_back(m_start);
	stEConfs.push_back(m_target);

	for (int i = 0; i < m_ec->size(); i++)
	{
		for (int j = 0; j < 2; j++)
		{
			CompositeConnectionFactory* cf 
				= new CompositeISConnectionFactory(&((*m_ec)[i]), stEConfs[j], m_edges,
					m_robotNum, m_col, m_stats);
			
			cf->generateEdges();
			delete cf;
		}	
	}
}