#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <tuple> // for tuple

std::vector<Node> binary_tree;

void splitBox(std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3>>& triangles, int index_parent_node, char axisSplit, std::vector<int>& left, std::vector<int>& right) {
    
    // only use the triangles in the parent
    std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3, int>> parentTriangles;
    //std::cout << std::endl;
    // add the indices
    for (int i = 0; i < binary_tree[index_parent_node].indices.size(); i++) {
        //std::cout << i << std::endl;
        std::tuple<glm::vec3, glm::vec3, glm::vec3, int> temp = 
            { get<0>(triangles[binary_tree[index_parent_node].indices[i]]), 
            get<1>(triangles[binary_tree[index_parent_node].indices[i]]), 
            get<2>(triangles[binary_tree[index_parent_node].indices[i]]), binary_tree[index_parent_node].indices[i] };
        parentTriangles.push_back(temp);
    }

    // split based on axis
    if (axisSplit == 'x') {
        std::sort(parentTriangles.begin(), parentTriangles.end(), BvhComparatorX());
    }
    if (axisSplit == 'y') {
        std::sort(parentTriangles.begin(), parentTriangles.end(), BvhComparatorY());
    }
    if (axisSplit == 'z') {
        std::sort(parentTriangles.begin(), parentTriangles.end(), BvhComparatorZ());
    }
   /* std::cout << axisSplit << std::endl;

    for (int i = 0; i < parentTriangles.size(); i++) {
        std::cout << get<0>(parentTriangles[i]).x << " " << get<0>(parentTriangles[i]).y << " " << get<0>(parentTriangles[i]).z << std::endl;
        std::cout << get<1>(parentTriangles[i]).x << " " << get<1>(parentTriangles[i]).y << " " << get<1>(parentTriangles[i]).z << std::endl;
        std::cout << get<2>(parentTriangles[i]).x << " " << get<2>(parentTriangles[i]).y << " " << get<2>(parentTriangles[i]).z << std::endl;
        std::cout << std::endl;
    }*/

    // split indices to left and right
    for (int i = 0; i < parentTriangles.size(); i++) {
        if (i < parentTriangles.size() / 2) {
            left.push_back(get<3>(parentTriangles[i]));
        }
        else {
            right.push_back(get<3>(parentTriangles[i]));
        }
    }/*
    std::sort(left.begin(), left.end());
    std::sort(right.begin(), right.end());
    for (int i = 0; i < parentTriangles.size() / 2; i++) {
        std::cout << left[i] << " " << right[i] << std::endl;
    }*/
}

void recursiveStepBvh(std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3>> triangles, int index_parent_node, int level, int max_level) {

    //Assume the old root has stored all the indices of each vertex of each triangle
    //Also the root node doesnt have the abb values calculated
    //std::cout << index_parent_node << std::endl;
    //First we want to calculate the AABB values (min x,y,z) and the (max x,y,z)
    //std::cout << std::endl;
    //for (int i : binary_tree[index_parent_node].indices) {
        //if (level ==2 )
        //std::cout << i << std::endl;
    //}
    //std::cout << index_parent_node << std::endl;

    glm::vec3 lower{ std::numeric_limits<float>::max() };
    glm::vec3 upper{ -std::numeric_limits<float>::max() };
    float tempsum = 0.f;
    for (int index : binary_tree[index_parent_node].indices) {

            // find the min and max value (go through each vertex of the triangle)
            glm::vec3 v0 = get<0>(triangles[index]);
            glm::vec3 v1 = get<1>(triangles[index]);
            glm::vec3 v2 = get<2>(triangles[index]);
            //std::cout << "LEVEL " << level << std::endl;
            if (level == 3) {
                tempsum+=v0.x+v1.x+v2.x;
            }
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
            //
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
    //std::cout << "LOWER - " << lower.x << " " << lower.y << " " << lower.z << std::endl;
    //std::cout << "UPPER - " << upper.x << " " << upper.y << " " << upper.z << std::endl;
    //if (level == 3) {
        //std::cout << tempsum << std::endl;
       // std::cout << lower.x << " " << lower.y << " " << lower.z << std::endl;
        //std::cout << upper.x << " " << upper.y << " " << upper.z << std::endl;
    //}
    binary_tree[index_parent_node].data = AxisAlignedBox{ lower,upper };
    // if not at max level, you should continue
    if (level < max_level && binary_tree[index_parent_node].indices.size()>1) {
        Node left;
        left.isLeaf = true;
        left.level = level+1;
        Node right;
        right.isLeaf = true;
        right.level = level+1;
        std::vector<int> leftVector;
        std::vector<int> rightVector;
        char splitAxis=' ';
        if (level % 3 == 0) {
            splitAxis = 'x';
        }
        if (level % 3 == 1) {
            splitAxis = 'y';
        }
        if (level % 3 == 2) {
            splitAxis = 'z';
        }
        //std::cout << "split " << index_parent_node << " split axis " << splitAxis << std::endl;
        splitBox(triangles, index_parent_node, splitAxis, leftVector, rightVector);
        left.indices = leftVector;
        right.indices = rightVector;
        binary_tree[index_parent_node].isLeaf = false;

        binary_tree.push_back(left);
        binary_tree.push_back(right);

        int size = binary_tree.size();
        int left_pos = size - 2;
        int right_pos = size - 1;

        //if (left.indices == binary_tree[size - 1].indices) {
            //std::cout << "ok I guess" << std::endl;
        //}

        //std::cout << index_parent_node << " split axis " << splitAxis << " level " << level << " left - " << size - 2 << " "
            //<< binary_tree[size-2].indices.size() << " right - " << size - 1 << " " << binary_tree[size-1].indices.size() << std::endl;
        recursiveStepBvh(triangles, left_pos, level + 1, max_level);
        recursiveStepBvh(triangles, right_pos, level + 1, max_level);
        //root.indices.clear();

        //root.indices.push_back(binary_tree.size());
        //root.indices.push_back(binary_tree.size() + 1);
        // first split using median, call recursiveStepBvh twice (left & right)
    }
   
}

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    std::cout << std::endl << "NEW SCENE - NEW SCENE" << std::endl;
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
    int i = 0;
    root.level = 0;
    binary_tree.push_back(root);
    recursiveStepBvh(triangles, i, 0, 10);

    /*for (int i = 0; i < binary_tree.size(); i++) {
        std::cout << i << " -- " << binary_tree[i].indices.size() << " " << binary_tree[i].isLeaf << " Lx " << binary_tree[i].data.lower.x 
            << " Ly " << binary_tree[i].data.lower.y << " Lz " << binary_tree[i].data.lower.z << std::endl;
        std::cout << " Ux " << binary_tree[i].data.upper.x << " Uy " << binary_tree[i].data.upper.y << " Uz " << binary_tree[i].data.upper.z << std::endl;
    }*/
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display.
int BoundingVolumeHierarchy::numLevels() const
{
    return 11;
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
    //std::cout << binary_tree.size() << std::endl;
    //int cnt = 0; int lcnt = 0;
    for (Node node : binary_tree) {
        AxisAlignedBox aabb{ node.data };
        //drawAABB(aabb, DrawMode::Wireframe);
        if (node.level == level) {
            drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
            //std::cout << lcnt << "---------------" << std::endl;
            //std::cout << aabb.upper.x << " " << aabb.upper.y << " " << aabb.upper.z << std::endl;
            //std::cout << aabb.lower.x << " " << aabb.lower.y << " " << aabb.lower.z << std::endl << std::endl;
            //lcnt++;
        }
        //cnt++;
        //}
    }
    //std::cout << cnt << std::endl;
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
