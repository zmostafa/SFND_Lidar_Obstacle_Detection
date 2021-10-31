/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// if(this->root == nullptr){
		// 	this->root = new Node(point,id);
		// 	return;
		// }
		_insert(&this->root,point,id,0);

	}

	void _insert(Node** root, std::vector<float>& point,int& id, int depth){
		
		if(*root == nullptr){
			*root = new Node(point,id);
		}else{

			unsigned int current_depth = depth % 2 ; // two planes x and y;
			if(point[current_depth] < (*root)->point[current_depth]){
				_insert(&((*root)->left), point, id, depth + 1);
			}else{
				_insert(&((*root)->right), point, id, depth + 1);
			}
		}

		// return *root;
	}

	void _search(std::vector<float> target, float distanceTol, int depth, std::vector<int>& ids, Node* root){
		if(root != nullptr){
			if(root->point[0] >= (target[0] - distanceTol) && root->point[0] <= (target[0] + distanceTol) && root->point[1] >= (target[1] - distanceTol) && root->point[1] <= (target[1] + distanceTol)){
				float distance = sqrt(pow((root->point[0] - target[0]),2) + pow((root->point[1] - target[1]),2)); 
				if(distance <= distanceTol)
					ids.push_back(root->id);
			}

			if((target[depth % 2] - distanceTol) < root->point[depth % 2])
				_search(target, distanceTol, depth + 1, ids, root->left);
			if((target[depth % 2] + distanceTol > root->point[depth % 2]))
				_search(target, distanceTol, depth + 1, ids, root->right);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		_search(target,distanceTol,0,ids,this->root);
		return ids;
	}

};




