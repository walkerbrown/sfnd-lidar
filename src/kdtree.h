// Project template by Aaron Brown at Udacity, Inc.
// Assignment completed by Dylan Walker Brown
// See LICENSE file for details

#ifndef KDTREE_H
#define KDTREE_H
#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id) {
		// Case tree is empty
		if (*node == NULL) {
			*node = new Node(point, id);
		}
		// Otherwise, traverse down the tree
		else {
			// Calculate split type based on current depth
			uint k = 3;  // k-dimensional space to partition
			uint cd = depth % k;
			if (point[cd] < ((*node)->point[cd])) {
				insertHelper(&((*node)->left), depth+1, point, id);
			} else {
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(&root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL) {
			auto x = node->point[0];
			auto y = node->point[1];
			auto z = node->point[2];
			auto tx = target[0];
			auto ty = target[1];
			auto tz = target[2];

			if (x >= tx - distanceTol &&
				x <= tx + distanceTol &&
				y >= ty - distanceTol &&
				y <= ty + distanceTol &&
				z >= tz - distanceTol &&
				z <= tz + distanceTol) {
				float distance = sqrt((x - tx)*(x - tx) +
									  (y - ty)*(y - ty) +
									  (z - tz)*(z - tz));
				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			// Split on xy/yz/zx hyperplane depending on depth and recurse
			uint k = 3;
			auto split_lr = depth % k;
			if (target[split_lr] - distanceTol < node->point[split_lr])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if (target[split_lr] + distanceTol > node->point[split_lr])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	
};

#endif
