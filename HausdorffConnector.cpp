#include "HausdorffConnector.h"
#include "CompositeISConnectionFactory.h"
#include "CompositeSillyConnectionFactory.h"
#include <time.h>
#include <cstdlib>
#include <ctime>

void	HausdorffConnector::generateExtConfEdges()
{ 
	double total_time = 0;
	for (int i = 0; i < m_ec->size() -1; i++)
	{
		priority_queue<PriorityType, vector<PriorityType>, ComparePairs> pq;
		clock_t time_start=clock();
		vector<ConfSet*> csi = (*m_ec)[i].getConfSet();
		
		for (int j = 0; j < m_ec->size(); j++)
		{
			if (j == i)
				continue;

			vector<ConfSet*> csj = (*m_ec)[j].getConfSet();
			double dist = distance(csi,csj);
			PriorityType pt(dist, j);

			pq.push(pt);
		}
		clock_t time_end = clock();
		total_time = total_time +  difftime(time_end,time_start)/1000;

		int num = m_stats->HAUSDORFF_CLOSEST;
		while (!pq.empty() && num != 0)
		{
			PriorityType pt = pq.top();
			int j = pt.second;
			pq.pop();
			num--;
			CompositeConnectionFactory* cf;

			// KIRIL
			//if (m_basic)
			if (false)
				cf = new CompositeSillyConnectionFactory(&((*m_ec)[i]), &((*m_ec)[j]), m_edges,
						m_robotNum, m_col, m_stats);

			else
				cf = new CompositeISConnectionFactory(&((*m_ec)[i]), &((*m_ec)[j]), m_edges,
						m_robotNum, m_col, m_stats);
			
			cf->generateEdges();
			delete cf;

			/*if (m_basic)
			{
				CompositeConnectionFactory* cf = new CompositeSillyConnectionFactory(&((*m_ec)[i]), &((*m_ec)[j]), m_edges,
					m_robotNum, m_col, m_stats);
				cf->generateEdges();
				delete cf;
			}*/
		}
	}

	FILE* fOutput = fopen("hausdorff.txt", "w+");
	fprintf(fOutput, "%f", total_time);
	fclose(fOutput);
}

void	HausdorffConnector::connectStartTarget()
{ 
	vector<CompositeConf*> stEConfs;
	stEConfs.push_back(m_start);
	stEConfs.push_back(m_target);

	for (int i = 0; i < m_ec->size() -1; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			CompositeConnectionFactory* cf;
			//if (m_basic)
			if (false)
				cf = new CompositeSillyConnectionFactory(&((*m_ec)[i]), stEConfs[j], m_edges,
					m_robotNum, m_col, m_stats);
			else
				cf = new CompositeISConnectionFactory(&((*m_ec)[i]), stEConfs[j], m_edges,
					m_robotNum, m_col, m_stats);
			
			cf->generateEdges();
			delete cf;

		}	
	}
}