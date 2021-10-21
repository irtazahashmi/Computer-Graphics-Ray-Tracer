#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <tuple> // for tuple

//Declare the binary tree as a global variable, where we are going to store all the information about the bvh for each node
std::vector<Node> binary_tree;

/// <summary>
/// This function splits the triangles from the parent node, into two vectors 
/// which two vectors are used to create the axis-aligned-boxes for the cildren
/// according to a specific axis. 
/// </summary>
/// <param name="triangles"> A vector which contains all the triangles in the mesh in the form of tuple
///     each glm::vec3 corresponds to one vertex of each triangle</param>
/// <param name="index_parent_node"> The index of the parent node in the binary_tree structure</param>
/// <param name="axisSplit"> A single char , either 'x' , 'y' or 'z' which indicates around which axis we need to split the triangles</param>
/// <param name="left"> A vector which will contain the indices of all the triangles that are at the left child of the parent node</param>
/// <param name="right"> A vector which will contain the indices of all the triangles that are at the right child of the parent node</param>
void splitBox(std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3>>& triangles, int index_parent_node, char axisSplit, std::vector<int>& left, std::vector<int>& right) {

    // Create new structure where we are going to store not only the position vectors of each vertex
    // but also the index of each triangle at the vector which contains all the triangles.
    // So after we sort them we can only push the indices back to child vectors (as specified)
    std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3, int>> parentTriangles;

    //We iterate through all the indices that are stored at the parent node vector<int> indices
    for (int i = 0; i < binary_tree[index_parent_node].indices.size(); i++) {
        //For each triangle in the parent axis-aligned-box we store all the information
        //in the vector that we have created at line-30. In the form of ---
        // --- < vertex vector v0, vertex vector v10, vertex vector v2, the triangle position in the main triangle vector>
        std::tuple<glm::vec3, glm::vec3, glm::vec3, int> temp =
        { get<0>(triangles[binary_tree[index_parent_node].indices[i]]),
        get<1>(triangles[binary_tree[index_parent_node].indices[i]]),
        get<2>(triangles[binary_tree[index_parent_node].indices[i]]), binary_tree[index_parent_node].indices[i] };
        parentTriangles.push_back(temp);
    }

    // We sort all the parent-triangles according to which axis we want to split
    if (axisSplit == 'x') {
        std::sort(parentTriangles.begin(), parentTriangles.end(), BvhComparatorX());
    }
    if (axisSplit == 'y') {
        std::sort(parentTriangles.begin(), parentTriangles.end(), BvhComparatorY());
    }
    if (axisSplit == 'z') {
        std::sort(parentTriangles.begin(), parentTriangles.end(), BvhComparatorZ());
    }

    // Now that all the triangles are sorted correctly, we simply pass the indices of the 
    // first half triangles to the left vector and the rest to the right vector
    for (int i = 0; i < parentTriangles.size(); i++) {
        if (i < parentTriangles.size() / 2) {
            left.push_back(get<3>(parentTriangles[i]));
        }
        else {
            right.push_back(get<3>(parentTriangles[i]));
        }
    }
}

/// <summary>
/// The main function that is used to iterate and create
/// all the neccesary nodes for the binary tree
/// </summary>
/// <param name="triangles"> A vector which contains all the triangles in the mesh in the form of tuple
///     each glm::vec3 corresponds to one vertex of each triangle</param>
/// <param name="index_parent_node"> The index of the (current) parent node in the binary_tree structure</param>
/// <param name="level"> The current level (or depth) of the node in the binary tree</param>
/// <param name="max_level"> The maximum level we want our tree to have</param>
void recursiveStepBvh(std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3>> triangles, int index_parent_node, int level, int max_level) {

    //First we need to calulate the upper and lower point of the current node's axis aligned box
    //Declare two vectors pointing to inf and -inf for the lower and upper vector accordingly
    glm::vec3 lower{ std::numeric_limits<float>::max() };
    glm::vec3 upper{ -std::numeric_limits<float>::max() };

    //Iterate through all the triangles that are in our node in order to find the upper and lower vector
    // of the current axis aligned box
    for (int index : binary_tree[index_parent_node].indices) {

        //Get all the vector positions for each triangle vertex
        glm::vec3 v0 = get<0>(triangles[index]);
        glm::vec3 v1 = get<1>(triangles[index]);
        glm::vec3 v2 = get<2>(triangles[index]);

        //Minimum check for v0
        lower.x = glm::min(lower.x, v0.x);
        lower.y = glm::min(lower.y, v0.y);
        lower.z = glm::min(lower.z, v0.z);
        //Minimum check for v1
        lower.x = glm::min(lower.x, v1.x);
        lower.y = glm::min(lower.y, v1.y);
        lower.z = glm::min(lower.z, v1.z);
        //Minimum check for v2
        lower.x = glm::min(lower.x, v2.x);
        lower.y = glm::min(lower.y, v2.y);
        lower.z = glm::min(lower.z, v2.z);

        //Max check for v0
        upper.x = glm::max(upper.x, v0.x);
        upper.y = glm::max(upper.y, v0.y);
        upper.z = glm::max(upper.z, v0.z);
        //Max check for v1
        upper.x = glm::max(upper.x, v1.x);
        upper.y = glm::max(upper.y, v1.y);
        upper.z = glm::max(upper.z, v1.z);
        //Max check for v2
        upper.x = glm::max(upper.x, v2.x);
        upper.y = glm::max(upper.y, v2.y);
        upper.z = glm::max(upper.z, v2.z);
    }

    //Update the upper and the lower vectors for the parent (current) node's axis aligned box
    binary_tree[index_parent_node].data = AxisAlignedBox{ lower,upper };

    //Now the function checks you havent reached the max level and there are more triangles to split
    if (level < max_level && binary_tree[index_parent_node].indices.size()>1) {

        //Declare the two children nodes the left and the right one
        Node left;
        Node right;

        //We set both nodes as leaves at first
        left.isLeaf = true;
        right.isLeaf = true;

        //We set the level for both left and right node as level+1
        left.level = level + 1;
        right.level = level + 1;

        //Declare the two vectors that we are going to pass as parameters to the split functions
        std::vector<int> leftVector;
        std::vector<int> rightVector;

        //Calculating which axis we need to split this level. Simply 
        //since we have the order x-y-z, we calculate the modulo of the 
        //level by 3 so we can see at which turn we are
        char splitAxis = ' ';
        if (level % 3 == 0) {
            splitAxis = 'x';
        }
        if (level % 3 == 1) {
            splitAxis = 'y';
        }
        if (level % 3 == 2) {
            splitAxis = 'z';
        }

        //Calling the splitBox function, split the triangles indices into two vectors according their position and the split axis
        splitBox(triangles, index_parent_node, splitAxis, leftVector, rightVector);

        //We pass the two vectors we calculated at the splitBox function at the corresponding nodes
        left.indices = leftVector;
        right.indices = rightVector;

        //We update the current (parent) node to be an interior node
        binary_tree[index_parent_node].isLeaf = false;

        //Push back to the vector the two nodes (left and right)
        binary_tree.push_back(left);
        binary_tree.push_back(right);

        //Few variables for easier access that we will need in the next stage
        int size = binary_tree.size();
        int left_pos = size - 2;
        int right_pos = size - 1;

        //Updating the parent node to save its children' indeces in the binary tree
        binary_tree[index_parent_node].indices.clear();
        binary_tree[index_parent_node].indices.push_back(left_pos);
        binary_tree[index_parent_node].indices.push_back(right_pos);

        //Call twice the recursive functions one for the left and once for the right node
        recursiveStepBvh(triangles, left_pos, level + 1, max_level);
        recursiveStepBvh(triangles, right_pos, level + 1, max_level);
    }
}

/// <summary>
/// Bounding Volume Hierarchy constructor
/// </summary>
/// <param name="pScene"></param>
BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    //Whenever we load new scene we need to clear the previous binary tree
    binary_tree.clear();

    //Create a vector that we will use to store all the triangles
    std::vector<std::tuple<glm::vec3, glm::vec3 , glm::vec3>> triangles;

    //Use the method given in intersection to iterate through and add all of the triangles to our vector
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

    //Simply add all numbers from zero to the size of the triangle vectors
    for (int i = 0; i < triangles.size(); i++) {
        root_indices.push_back(i);
    }

    //Assign the calculated vector to the root indices' vector
    root.indices = root_indices;
    int i = 0;

    //Set root level as 0
    root.level = 0;

    //Push the root
    binary_tree.push_back(root);

    //Call the recursive function
    recursiveStepBvh(triangles, i, 0, 10);
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
    for (Node node : binary_tree) {
        AxisAlignedBox aabb{ node.data };
        if (node.level == level) {
            drawAABB(aabb, DrawMode::Wireframe, glm::vec3(1.0f));
        }
    }
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
