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
};

static void insert_at_depth(Node* current, int depth, Node* new_node) {
    int idx_to_compare = depth % current->point.size();
    Node*& next = new_node->point.at(idx_to_compare) < current->point.at(idx_to_compare) ? current->left : current->right;
    if (next == nullptr) {
        next = new_node;
    } else {
        insert_at_depth(next, depth + 1, new_node);
    }
}

static bool is_nearby(const std::vector<float>& a, const std::vector<float>& b, float tolerance) {
    float dx = a.at(0) - b.at(0);
    float dy = a.at(1) - b.at(1);
    return std::abs(dx) <= tolerance and std::abs(dy) <= tolerance;
}

static void check_at_depth(const Node& current,
                           int depth,
                           const std::vector<float>& target,
                           float tolerance,
                           std::vector<int>& neighbor) {
    int idx_to_compare = depth % current.point.size();

    if (std::abs(current.point.at(idx_to_compare) - target.at(idx_to_compare)) <= tolerance) {
        // if `current` is close to `target` at least in one dimension,
        // - run strict check and add `current` to `neighbor` if truly in neighbor
        // - check both childen recursively.

        if (is_nearby(current.point, target, tolerance)) {
            neighbor.push_back(current.id);
        }

        if (current.left != nullptr) {
            check_at_depth(*current.left, depth + 1, target, tolerance, neighbor);
        }
        if (current.right != nullptr) {
            check_at_depth(*current.right, depth + 1, target, tolerance, neighbor);
        }
    } else {
        // if `current` is not close enough to `target` in the dimension, check only one child.

        if (target.at(idx_to_compare) < current.point.at(idx_to_compare) and current.left != nullptr) {
            check_at_depth(*current.left, depth + 1, target, tolerance, neighbor);
        } else if (current.right != nullptr) {
            check_at_depth(*current.right, depth + 1, target, tolerance, neighbor);
        }
    }
}

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		Node* new_node = new Node(point, id);

        if (root == nullptr) {
            root = new_node;
            return;
        }

        insert_at_depth(root, 0, new_node);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		if (root != nullptr) {
            check_at_depth(*root, 0, target, distanceTol, ids);
        }

		return ids;
	}
	

};




