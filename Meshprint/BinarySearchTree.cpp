#include "BinarySearchTree.h"


template<class Type>
BinarySearchTree::BinarySearchTree()
{
}

template<class Type>

BinarySearchTree::~BinarySearchTree()
{
}

template<class Type>
bool BinarySearchTree<Type>::find(const Type & x) const
{

	return find(x,root);
}
template<class Type>

bool BinarySearchTree<Type>::find(const Type & x, BinaryNode * t) const
{
	if (t == NULL)
	{
		return false;
	}
	else if (x < t->data)
	{
		return find(x, t->left);
	}
	else if (t->data < x)
	{
		return find(x, t->right)
	}
	else
		return false;
}

template<class Type>
void BinarySearchTree<Type>::insert(const Type & x)
{
}

template<class Type>
void BinarySearchTree<Type>::remove(const Type & x)
{
}

template<class Type>
void BinarySearchTree<Type>::insert(const Type & x, BinaryNode *& t)
{
}

template<class Type>
void BinarySearchTree<Type>::remove(const Type & x, BinaryNode *& t)
{
}


template<class Type>
void BinarySearchTree<Type>::makeEmpty(BinaryNode *& t)
{
}
