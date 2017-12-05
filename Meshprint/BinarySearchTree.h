#pragma once
template<class Type>
class BinarySearchTree
{
private:
	struct BinaryNode
	{
		Type data;
		BinaryNode *left;
		BinaryNode *right;
		BinaryNode(const Type & thedata, BinaryNode* lt, BinaryNode*rt) :data(thedata), left(lt), right(rt) {}
	};
public:
	BinarySearchTree();
	~BinarySearchTree();
	BinarySearchTree(BinaryNode* t = NULL) { root = t };
	bool find(const Type  &x)const;
	void insert(const Type&x);
	void remove(const Type&x);
private:
	BinaryNode *root;
	void insert(const Type &x, BinaryNode* &t);
	void remove(const Type &x, BinaryNode*&t);
	bool find(const Type &x, BinaryNode* t)const;
	void makeEmpty(BinaryNode*&t);
};

