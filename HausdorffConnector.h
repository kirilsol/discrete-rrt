#include "Connector.h"

typedef pair<double, int> PriorityType;

class ComparePairs {
public:
    bool operator()(PriorityType& p1, PriorityType&  p2)
    {
		return p1.first < p2.first;
    }
};

class HausdorffConnector : public Connector
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	HausdorffConnector(vector<CompositeConf>* ecl, vector<CompositeConfEdge>* ecel, 
		CompositeConf* start, CompositeConf* target, vector<int> robotNum, CollisionDetector* col, Stats* stats) 
		: Connector(ecl, ecel, start, target, robotNum, col, stats){};
	~HausdorffConnector(){}

	////////////////////////
	// Queries
	////////////////////////
	
	virtual		void	generateExtConfEdges();

	virtual		void	connectStartTarget();

private:
	double		confDistance(Conf c1, Conf c2)
	{
		double x1 = c1.x() ;
		double y1 = c1.y() ;
		double x2 = c2.x() ;
		double y2 = c2.y() ;

		return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
	}

	double		oneSidedDistance(ConfSet* cs1, ConfSet* cs2)
	{
		double max_dist = 0;
		for (int i = 0; i < cs1->size(); i++)
		{
			double dist = confDistance((*cs1)[i], (*cs2)[0]);
			for (int j = 1; j < cs2->size(); j++)
			{
				double dist_cand =  confDistance((*cs1)[i], (*cs2)[j]);
				if (dist > dist_cand)
					dist = dist_cand;
			}
			if (dist > max_dist)
				max_dist = dist;
		}
		return max_dist;
	}

	double		distance(ConfSet* cs1, ConfSet* cs2)
	{
		double dist1 = oneSidedDistance(cs1, cs2);
		double dist2 = oneSidedDistance(cs2, cs1);

		if (dist1 > dist2) 
			return dist1;
		return dist2;
	}

	double		distance(vector<ConfSet*> cs1, vector<ConfSet*> cs2)
	{
		double maxDist = 0;

		for (int g = 0; g < cs1.size(); g++)
		{
			double currDist = distance(cs1[g], cs2[g]);
			if (currDist > maxDist)
				maxDist = currDist;
		}
		
		return maxDist;
	}

	////////////////////////
	// Data Members
	////////////////////////
	
};
