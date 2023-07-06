class IDFactory
{
public:
	IDFactory() : m_count(0){}
	~IDFactory(){}

	int getCount(){return m_count++; }
	int count(){return m_count;}

private:
	int m_count;
};