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

	void insertRecursor(Node*& currentNode, int depth, std::vector<float> point, int id )
	{
		// if the node is empty
		if(currentNode == NULL)
		{
			currentNode = new Node(point, id);
		}
		else
		{
			//Determine if odd or even based on depth 
			uint idx = depth % 3;

			if(point[idx] < currentNode->point[idx])
			{
				insertRecursor((currentNode->left), depth+1, point, id);
			}
			else
			{
				insertRecursor((currentNode->right), depth+1, point, id);
			}
		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 	
		insertRecursor(root, 0, point, id );		
	}

	void searchRecursor(std::vector<float> target, Node*& currentNode, int depth,float distanceTol,std::vector<int>& ids )
	{
		if (currentNode != NULL)
		{
			float x_diff = abs(target[0] - currentNode->point[0]);
			float y_diff = abs(target[1] - currentNode->point[1]);
			float z_diff = abs(target[2] - currentNode->point[2]);
			uint idx = depth % 3;

			if (x_diff <= distanceTol && y_diff <= distanceTol && z_diff <= distanceTol)  //check if within box
			{
				if (sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff) <= distanceTol) //check if within radius
				{
					ids.push_back(currentNode->id);
				}
			} 
			
			if(target[idx] - distanceTol < currentNode->point[idx])
			{
				searchRecursor(target, (currentNode->left), depth+1, distanceTol, ids);
			}
			if(target[idx] + distanceTol > currentNode->point[idx])
			{
				searchRecursor(target, (currentNode->right), depth+1, distanceTol, ids);
			}	
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRecursor(target, root, 0, distanceTol, ids);
		return ids;
	}	

};




