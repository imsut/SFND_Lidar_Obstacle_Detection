#pragma once

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
    const PointT& point;
    int id;
    Node* left;
    Node* right;

    Node(const PointT& pt, int setId) : point(pt), id(setId), left(nullptr), right(nullptr) {}
};

template<typename PointT> size_t Dimension(PointT) { return 3; }

template<typename PointT> float Value(const PointT& point, int index);

template <>
inline float Value(const pcl::PointXYZI& point, int index) {
    switch (index) {
        case 0:
            return point.x;
        case 1:
            return point.y;
        case 2:
            return point.z;
        default:
            return 0.0;
    }
}

template <>
inline float Value(const pcl::PointXYZ& point, int index) {
    switch (index) {
        case 0:
            return point.x;
        case 1:
            return point.y;
        case 2:
            return point.z;
        default:
            return 0.0;
    }
}

template<typename PointT>
void insert_at_depth(Node<PointT>* current, int depth, Node<PointT>* new_node) {
    int idx_to_compare = depth % Dimension(current->point);
    Node<PointT>*& next = Value(new_node->point, idx_to_compare) < Value(current->point, idx_to_compare)
                          ? current->left : current->right;
    if (next == nullptr) {
        next = new_node;
    } else {
        insert_at_depth(next, depth + 1, new_node);
    }
}

template<typename PointT>
bool is_nearby(const PointT& a, const PointT& b, float tolerance);

template<>
inline bool is_nearby(const pcl::PointXYZI& a, const pcl::PointXYZI& b, float tolerance) {
    float dx = std::abs(a.x - b.x);
    float dy = std::abs(a.y - b.y);
    float dz = std::abs(a.z - b.z);
    //std::printf("(dx, dy, dz) = (%f, %f, %f)\n", dx, dy, dz);
    return dx <= tolerance and dy <= tolerance and dz <= tolerance;
}

template<>
inline bool is_nearby(const pcl::PointXYZ& a, const pcl::PointXYZ& b, float tolerance) {
    float dx = std::abs(a.x - b.x);
    float dy = std::abs(a.y - b.y);
    float dz = std::abs(a.z - b.z);
    //std::printf("(dx, dy, dz) = (%f, %f, %f)\n", dx, dy, dz);
    return dx <= tolerance and dy <= tolerance and dz <= tolerance;
}

template<typename PointT>
static void check_at_depth(const Node<PointT>& current,
                           int depth,
                           const PointT& target,
                           float tolerance,
                           std::vector<int>& neighbor) {
    int idx_to_compare = depth % Dimension(current.point);

    if (std::abs(Value(current.point, idx_to_compare) - Value(target, idx_to_compare)) <= tolerance) {
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
        if (Value(target, idx_to_compare) < Value(current.point, idx_to_compare) and current.left != nullptr) {
            check_at_depth(*current.left, depth + 1, target, tolerance, neighbor);
        } else if (current.right != nullptr) {
            check_at_depth(*current.right, depth + 1, target, tolerance, neighbor);
        }
    }
}


template<typename PointT>
class KdTree {
  public:
    KdTree() : root(nullptr) {}

    ~KdTree() {
        delete root;
    }

    /// insert a point with id to this tree.
    void insert(const PointT& point, int id) {
        Node<PointT>* new_node = new Node<PointT>(point, id);

        if (root == nullptr) {
            root = new_node;
            return;
        }

        insert_at_depth(root, 0, new_node);
    }

    /// return a list of point ids in the tree that are within distance of target.
    std::vector<int> search(const PointT& target, float distanceTol) const {
        std::vector<int> ids;

        if (root != nullptr) {
            check_at_depth(*root, 0, target, distanceTol, ids);
        }

        return ids;
    }

  private:
    Node<PointT>* root;
};
