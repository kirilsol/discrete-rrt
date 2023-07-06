#include "basic_typedef.h"

class Tree {

private:
	int				m_ID;

	Tree*			m_father;
	std::vector<Tree*>	m_children;

	bool			m_occupied;
	//int				m_occupier;

	int				m_currentOccupancy;
	int				m_targetOccupancy;

	bool			m_targetNode;

	MoveRecorder*	m_recorder; // records the movement of the tokens within the tree;

public:
	////////////////////////
	// CTORS
	////////////////////////
	Tree(int ID, MoveRecorder* recorder); 
	~Tree();

	////////////////////////
	// queries
	////////////////////////
	bool	isFull();
	bool	isEmpty();
	bool	isOccupied();
	int 	getCurrentOccupancy();
	int		getTargetOccupancy();
	int		getID();
	Tree*	getRoot();
	
	////////////////////////
	// modifiers
	////////////////////////
	void	setFather(Tree* father);
	void	addChild(Tree* child);
	//void	setOccupied(/*int occupier*/);
	void	putToken(/*int occupier*/);
	void	incOccupancy();
	void	decOccupancy();
	void	makeTarget();

	////////////////////////
	// algorithms
	////////////////////////
	/*	moves all the tokens to the bottom of the tree	*/
	void	lowerAllTokens();

	/*	moves current token to one of the children
		and applies the  algorithm recursively	*/
	void	lowerCurrentToken();

	/*	clear the tree from the algorithm data	*/
	void	clear();

	/*	balance the tokens in the subtrees and this node	*/
	void	balance();

	/*	moves one token from currren node (if occupied) or its subchildren	*/
	void	moveTokenUp();

	/*	updated the occupancy counters	*/
	void	prepare();

	void	run();

};