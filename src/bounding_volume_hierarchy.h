#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <span>

struct Node {
    AxisAlignedBox data;
    bool isLeaf;
    int level;
    std::vector <int> indices;
};

struct BvhComparatorX {
    inline bool operator() (const std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple1, std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple2)
    {
        return (get<0>(tuple1).x + get<1>(tuple1).x + get<2>(tuple1).x < get<0>(tuple2).x + get<1>(tuple2).x + get<2>(tuple2).x);
    }
};

struct BvhComparatorY {
    inline bool operator() (const std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple1, std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple2)
    {
        return (get<0>(tuple1).y + get<1>(tuple1).y + get<2>(tuple1).y < get<0>(tuple2).y + get<1>(tuple2).y + get<2>(tuple2).y);
    }
};

struct BvhComparatorZ {
    inline bool operator() (const std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple1, std::tuple<glm::vec3, glm::vec3, glm::vec3, int> tuple2)
    {
        return (get<0>(tuple1).z + get<1>(tuple1).z + get<2>(tuple1).z < get<0>(tuple2).z + get<1>(tuple2).z + get<2>(tuple2).z);
    }
};

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Implement these two functions for the Visual Debug.
    // The first function should return how many levels there are in the tree that you have constructed.
    // The second function should draw the bounding boxes of the nodes at the selected level.
    int numLevels() const;
    void debugDraw(int level);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

private:
    Scene* m_pScene;
};
