#include "basic_typedef.h"
#include "IDFactory.h"

class InstancesManager
{
private:
	list<pair<vector<int>,int>> m_signatureIDMap;
	IDFactory*	m_counter;

public:	
	InstancesManager(IDFactory* vertexCounter)
		: m_counter(vertexCounter) {}

	~InstancesManager(){}

	int		getInstanceID(vector<int> signature)
	{
		bool found = false;
		int ID;

		for(list<pair<vector<int>,int>>::iterator iter = m_signatureIDMap.begin();
			iter != m_signatureIDMap.end(); iter++)
		{
			vector<int> old_sig = iter->first;
			ID = iter->second;

			CGAL_precondition(old_sig.size() ==  signature.size());

			bool match = true;
			for (int i = 0; i < signature.size(); i++)
			{
				if (signature[i] != old_sig[i])
				{
					match = false;
					break;
				}
			}

			if (match)
			{
				found = true;
				break;
			}
		}

		if (!found)
		{
			ID = m_counter->getCount();
			m_signatureIDMap.push_back(pair<vector<int>,int>(signature,ID));
		}

		return ID;
	}
};

