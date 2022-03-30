/* \author Camilo Duarte */
// Implementation of kd tree

#include "render.h"
#include <iostream>
#include <cmath>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	int depth;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId, int setDepth)
	:	point(arr), id(setId), depth(setDepth), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertRecursive(Node *&node, std::vector<float> point, int id, int depth)
    {
		// Use a helper function to enable recursion 
    	if(node == NULL)
    	{
    		node = new Node(point, id, depth);

    	}
		else{
			// Check the current depth of the tree
			// If 0, then even, we check x points
			// If 1, then odd, we check y points
			int cd = depth % 2;
			if (point[cd] < node->point[cd]){
				insertRecursive(node->left, point, id, depth+1);
			}else{
				insertRecursive(node->right, point, id, depth+1);
			}
		}
    }

	void insert(std::vector<float> point, int id)
	{
		// Initialize the recursion
		insertRecursive(*&root, point, id, 0);

	}


	void searchRecursive(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids){
		if (node !=NULL){
			if( (node->point[0] >=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol)) && (node->point[1] >=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol)))
			{
				float d = sqrt(pow(target[0] - node->point[0], 2.0) + pow(target[1] - node->point[1], 2.0));
				if(d <= distanceTol){
					ids.push_back(node->id);
				}
			}
			// Check the left nodes 
			if (target[depth%2]-distanceTol<node->point[depth%2]){
				searchRecursive(target, node->left, depth+1, distanceTol, ids);
			}
			//Check the right nodes
			if (target[depth%2]+distanceTol>node->point[depth%2]){
				searchRecursive(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol){
		std::vector<int> ids;
		searchRecursive(target, root, 0, distanceTol, ids);

		return ids;
	}

	

};




