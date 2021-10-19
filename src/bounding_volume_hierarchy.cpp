#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include<tuple> // for tuple

std::vector<Node> binary_tree;

void recursiveStepBvh(std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3>> triangles, Node& root, int level, int max_level) {

    //Assume the old root has stored all the indices of each vertex of each triangle
    //Also the root node doesnt have the abb values calculated

    //First we want to calculate the AABB values (min x,y,z) and the (max x,y,z)

    glm::vec3 lower{ std::numeric_limits<float>::max() };
    glm::vec3 upper{ -std::numeric_limits<float>::max() };
    //std::cout << "test " << triangles.size() << lower.x << " " << lower.y << " " << lower.z << std::endl;
    for (int index : root.indices) {
        //std::cout << index << std::endl;
        //std::cout << triangles[index].x << " " << triangles[index].y << " " << triangles[index].z << std::endl;
        
        //glm::vec3 temp = get<0>(triangles[index]);

            glm::vec3 v0 = get<0>(triangles[index]);
            glm::vec3 v1 = get<1>(triangles[index]);
            glm::vec3 v2 = get<2>(triangles[index]);
            if (lower.x > v0.x) {
                lower.x = v0.x;
            }
            if (lower.y > v0.y) {
                lower.y = v0.y;
            }
            if (lower.z > v0.z) {
                lower.z = v0.z;
            }

            if (upper.x < v0.x) {
                upper.x = v0.x;
            }
            if (upper.y < v0.y) {
                upper.y = v0.y;
            }
            if (upper.z < v0.z) {
                upper.z = v0.z;
            }
            //
            if (lower.x > v1.x) {
                lower.x = v1.x;
            }
            if (lower.y > v1.y) {
                lower.y = v1.y;
            }
            if (lower.z > v1.z) {
                lower.z = v1.z;
            }

            if (upper.x < v1.x) {
                upper.x = v1.x;
            }
            if (upper.y < v1.y) {
                upper.y = v1.y;
            }
            if (upper.z < v1.z) {
                upper.z = v1.z;
            }

            if (lower.x > v2.x) {
                lower.x = v2.x;
            }
            if (lower.y > v2.y) {
                lower.y = v2.y;
            }
            if (lower.z > v2.z) {
                lower.z = v2.z;
            }

            if (upper.x < v2.x) {
                upper.x = v2.x;
            }
            if (upper.y < v2.y) {
                upper.y = v2.y;
            }
            if (upper.z < v2.z) {
                upper.z = v2.z;
            }
    }
    root.data = AxisAlignedBox{ lower,upper };
    std::cout << "test1 " << triangles.size() << lower.x << " " << lower.y << " " << lower.z << std::endl;
    binary_tree.push_back(root);
   
}

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    binary_tree.clear();
    //Create a vector that contains all the triangles
    std::vector<std::tuple<glm::vec3, glm::vec3 , glm::vec3>> triangles;
    //Use the method given in intersection to iterate and add all of the triangles to our vector
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            std::tuple temp_tri = { v0.position,v1.position,v2.position };
            triangles.push_back(temp_tri);
        }
    }
    
    // Create the first root node
    Node root;
    //We assume at the beginning that is a leaf
    root.isLeaf = true;

    //Create a vector to store the indices of all the triangles at the moment
    std::vector<int> root_indices;

    for (int i = 0; i < triangles.size(); i++) {
        root_indices.push_back(i);
    }
    root.indices = root_indices;

    //binary_tree.push_back(root);
    recursiveStepBvh(triangles, root, 0, 0);

}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display.
int BoundingVolumeHierarchy::numLevels() const
{
    return 1;
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    AxisAlignedBox aabb {binary_tree[0].data};
    std::cout << binary_tree[0].data.lower.x << " " << binary_tree[0].data.lower.y << " " << binary_tree[0].data.lower.z << std::endl;
    //drawAABB(aabb, DrawMode::Wireframe);
    drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
}



// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    bool hit = false;
    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
            float t = ray.t;
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                if (ray.t < t) {
                    hitInfo.material = mesh.material;
                    hit = true;
                    hitInfo.v0 = v0;
                    hitInfo.v1 = v1;
                    hitInfo.v2 = v2;
                }
            }
        }
    }
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}
