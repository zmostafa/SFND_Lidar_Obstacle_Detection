/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <pcl/common/common.h>

// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
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

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// if(this->root == nullptr){
		// 	this->root = new Node(point,id);
		// 	return;
		// }
		_insert(&this->root,point,id,0);

	}

	void _insert(Node** root, pcl::PointXYZI& point,int& id, int depth){
		
		if(*root == nullptr){
			*root = new Node(point,id);
		}else{

			unsigned int current_depth = depth % 3 ; // two planes x , y and z;
			if(current_depth == 0){
				if(point.x < (*root)->point.x){
					_insert(&((*root)->left), point, id, depth + 1);
				}else{
					_insert(&((*root)->right), point, id, depth + 1);
				}
			}else{
				if(point.y < (*root)->point.y){
					_insert(&((*root)->left), point, id, depth + 1);
				}else{
					_insert(&((*root)->right), point, id, depth + 1);
				}
			}
		}
		// return *root;
	}

	void _search(pcl::PointXYZI target, float distanceTol, int depth, std::vector<int>& ids, Node* root){
		if(root != nullptr){
			if(fabs(root->point.x - target.x) <= distanceTol && fabs(root->point.y - target.y) <= distanceTol)
			{
				float distance = sqrt(pow((root->point.x - target.x),2) + pow((root->point.y - target.y),2));
				
				if(distanceTol <= distanceTol)
					ids.push_back(root->id);
			}
			if(depth%3==0)
			{
				if((target.x - distanceTol) < root->point.x)
					_search(target, distanceTol, depth + 1, ids, root->left);
				if((target.x + distanceTol) > root->point.x)
					_search(target, distanceTol, depth + 1, ids, root->right);
			}
			else
			{
				if((target.y - distanceTol) < root->point.y)
					_search(target, distanceTol, depth + 1, ids, root->left);
				if((target.y + distanceTol) > root->point.y)
					_search(target, distanceTol, depth + 1, ids, root->right);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		_search(target,distanceTol,0,ids,this->root);
		return ids;
	}

};




