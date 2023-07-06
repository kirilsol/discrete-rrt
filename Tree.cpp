
#include "Tree.h"

////////////////////////
// CTORS
////////////////////////
Tree::Tree(int ID, MoveRecorder* recorder)
{
	m_ID = ID;
	m_father = NULL;
	m_occupied = false;
	m_recorder = recorder;
	m_targetNode = false;
}

Tree::~Tree()
{
	if (m_children.empty())
		return;

	foreach (Tree* t, m_children)
	{
		delete t;
	}

	m_children.clear();
}

////////////////////////
// queries
////////////////////////
bool	Tree::isFull()
{
	if (!m_occupied)
		return false;

	foreach(Tree* currentChild, m_children)
	{
		if (!currentChild->isFull())
			return false;
	}

	return true;
}

bool	Tree::isEmpty()
{
	if (m_occupied)
		return false;

	foreach(Tree* currentChild, m_children)
	{
		if (!currentChild->isEmpty())
			return false;
	}

	return true;
}

bool	Tree::isOccupied()
{
	return m_occupied;
}

int		Tree::getCurrentOccupancy()
{
	return m_currentOccupancy;
}

int		Tree::getTargetOccupancy()
{
	return m_targetOccupancy;
}

int	Tree::getID()
{
	return m_ID;
}

Tree*	Tree::getRoot()
{
	if (m_father == NULL)
		return this;
	return m_father->getRoot();
}

////////////////////////
// modifiers
////////////////////////

void Tree::setFather(Tree* father)
{
	m_father = father;
}

void	Tree::addChild(Tree* child)
{
	m_children.push_back(child);
}


void	Tree::putToken(/*int occupier*/)
{
	m_occupied = true;
	//m_occupier = occupier;
}

void	Tree::incOccupancy()
{
	m_currentOccupancy++;
}

void	Tree::decOccupancy()
{
	m_currentOccupancy--;
}

void	Tree::makeTarget()
{
	m_targetNode = true;
}

///////////////////////
// algorithms
///////////////////////
/*	moves all the tokens to the bottom of the tree	*/
void	Tree::lowerAllTokens()
{
	if (m_children.empty()) //leaf
	{
		if (m_occupied)
			//m_full = true;
		return; 
	}
	
	bool allChildrenFull = true;
	for (vector<Tree*>::iterator iter = m_children.begin(); 
			iter != m_children.end(); iter ++)
	{
		Tree* currentChild = *iter;
		currentChild->lowerAllTokens();

		if (!currentChild->isFull() && m_occupied)
		{
			currentChild->putToken(/*m_occupier*/);
			
			//Move move(Edge(m_ID, currentChild->getID()), m_occupier);
			int childID = currentChild->getID();
			Edge edge(m_ID, childID);
			m_recorder->push_back(edge);
			currentChild->lowerCurrentToken();

			m_occupied = false;
		} 
		else
			allChildrenFull = allChildrenFull && currentChild->isFull();
	}

	//m_full = m_occupied && allChildrenFull;
}

/*	moves current token to one of the children
	and applies the algorithm recursively	*/
void	Tree::lowerCurrentToken()
{
	for (vector<Tree*>::iterator iter = m_children.begin(); 
			iter != m_children.end(); iter ++)
	{
		Tree* currentChild = *iter;

		if (!currentChild->isFull())
		{
			currentChild->putToken(/*m_occupier*/);
			currentChild->incOccupancy();
			//Move move(Edge(m_ID, currentChild->getID()), m_occupier);
			Edge edge(m_ID, currentChild->getID());
			m_recorder->push_back(edge);
			currentChild->lowerCurrentToken();

			m_occupied = false;

			break;
		} 
	}

	//m_full = m_occupied;
}

/*	clear the tree from the algorithm data	*/
void	Tree::clear()
{
	m_occupied = false;
	m_targetNode = false;
	//m_full =false;

	for (vector<Tree*>::iterator iter = m_children.begin(); 
			iter != m_children.end(); iter ++)
	{
		Tree* currentChild = *iter;

		currentChild->clear();
	}
}

/*	balance the tokens in the subtrees and this node	*/
void	Tree::balance()
{
	prepare();
	if (m_children.empty())
		return;

	std::list<Tree*> under, over;

	for (int i = 0; i < m_children.size(); i++)
	{
		int state = m_children[i]->getCurrentOccupancy() - m_children[i]->getTargetOccupancy();
		
		if (state > 0) 
		{
			for (int j = 0; j < state; j++)
				over.push_back(m_children[i]);
		}
		else if (state < 0) 
		{
			for (int j = 0; j < (-state); j++)
				under.push_back(m_children[i]);
		}
	}

	while (!under.empty())
	{
		Tree* moveTo = under.back();
		under.pop_back();

		Tree* moveFrom = over.back();
		over.pop_back();

		moveFrom->moveTokenUp();

		Edge edge(m_ID, moveTo->getID());
		m_recorder->push_back(edge);

		m_occupied = false;

		moveTo->putToken(/*tokenFromOver*/);
		moveTo->lowerCurrentToken();
	}

	if (!over.empty())
	{
		Tree* moveFrom = over.back();
		over.pop_back();
		moveFrom->moveTokenUp();
		m_occupied = true;
	}

	foreach (Tree* t, m_children)
	{
		t->balance();
	}
}

/*	moves one token from curren node (if occupied) or its subchildren	*/
void	Tree::moveTokenUp()
{
	if (!m_occupied)
	{
		for (vector<Tree*>::iterator iter = m_children.begin(); 
			iter != m_children.end(); iter ++)
		{
			Tree* currentChild = *iter;
			if (!currentChild->isEmpty())
			{
				currentChild->moveTokenUp();
				break;
			}
		}
	}
		
	m_occupied = false;
	decOccupancy();
	
	Edge edge(m_ID, m_father->getID());
	m_recorder->push_back(edge);
	
	//return /*m_occupier*/;
}

/*	updated the occupancy counters	*/
void	Tree::prepare()
{	
	m_currentOccupancy = 0;
	m_targetOccupancy = 0;

	for (vector<Tree*>::iterator iter = m_children.begin(); 
			iter != m_children.end(); iter ++)
	{
		Tree* currentChild = *iter;
		currentChild->prepare();

		m_currentOccupancy = m_currentOccupancy + currentChild->getCurrentOccupancy();
		m_targetOccupancy = m_targetOccupancy + currentChild->getTargetOccupancy();
	}

	if (m_occupied)
		m_currentOccupancy++;

	if (m_targetNode)
		m_targetOccupancy++;
}

void	Tree::run()
{
	lowerAllTokens();
	balance();
	clear();
}