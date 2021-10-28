#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <span>

/// <summary>
/// Declare few external boolean that we will need them inside the bounding_volume_hierarchy.cpp 
/// for debug purposes but they were declared in the main.cpp
/// </summary>
extern bool debugIntersectionAABB;
extern bool drawTrianglesInLeaf;

/// <summary>
/// The struct of a single node of our binary tree
/// </summary>
struct Node {
    AxisAlignedBox data; //Here we store the upper and lower point of our axis-aligned-box
    bool isLeaf; //Boolean to indicate whether the node is a leaf or not
    int level; // The level (depth) of our node
    std::vector <int> indices; //The vector in case of leaf contains the indices of the triangles that are inside the corresponding box
                                //in case of an inside node we store the indices of the left and right child inside the binary tree structure
};


// Comparator for the x axis
// Compare the sum of the x axis of each vertex in each tuple with each other
struct BvhComparatorX {
    inline bool operator() (const std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple1, std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple2)
    {
        return (get<0>(tuple1).x + get<1>(tuple1).x + get<2>(tuple1).x < get<0>(tuple2).x + get<1>(tuple2).x + get<2>(tuple2).x);
    }
};

// Comparator for the y axis
// Compare the sum of the y axis of each vertex in each tuple with each other
struct BvhComparatorY {
    inline bool operator() (const std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple1, std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple2)
    {
        return (get<0>(tuple1).y + get<1>(tuple1).y + get<2>(tuple1).y < get<0>(tuple2).y + get<1>(tuple2).y + get<2>(tuple2).y);
    }
};

// Comparator for the z axis
// Compare the sum of the z axis of each vertex in each tuple with each other
struct BvhComparatorZ {
    inline bool operator() (const std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple1, std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple2)
    {
        return (get<0>(tuple1).z + get<1>(tuple1).z + get<2>(tuple1).z < get<0>(tuple2).z + get<1>(tuple2).z + get<2>(tuple2).z);
    }
};

/// <summary>
/// New structure just to be used in the next function when we  will need to use 
/// priority queue in a tuple (see next function)
/// </summary>
struct AABBwithIntersection {
    //t the value for the ray for which it intersects with the box that is stored 
    //the binary tree at [index]
    float t;
    int index;
    //Default constructor
    AABBwithIntersection(float tin, int indx) : t(tin), index(indx) {}
    //Operator we are going to use when adding elements in the pq
    bool operator<(const struct AABBwithIntersection& other) const
    {
        return t > other.t;
    }
};

/// <summary>
/// Decleration of the class and its methods / constructor.
/// More in depth explanation for each of these functions can be found in the bounding_volume_hierarchy.cpp
/// </summary>
class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);
    int numLevels() const;
    void debugDraw(int level);
    bool intersect(Ray& ray, HitInfo& hitInfo) const;
private:
    Scene* m_pScene;
    
};
