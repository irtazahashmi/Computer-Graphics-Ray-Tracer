#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <tuple> 
#include <queue>
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <imgui.h>
#include <nfd.h>
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
DISABLE_WARNINGS_POP()
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <framework/image.h>
#include <framework/imguizmo.h>
#include <framework/trackball.h>
#include <framework/variant_helper.h>
#include <framework/window.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#include <variant>
//Declare the binary tree as a global variable, where we are going to store all the information about the bvh for each node
std::vector<Node> binary_tree;

//In this data structure we will store information for all the triangles in each scene from all the meshes
std::vector<std::tuple<Vertex, Vertex, Vertex, Material>> triangles;

// max bvh level
const int MAX_LEVEL = 10;


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
void splitBox(int index_parent_node, char axisSplit, std::vector<int>& left, std::vector<int>& right) {

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
        { get<0>(triangles[binary_tree[index_parent_node].indices[i]]).position,
        get<1>(triangles[binary_tree[index_parent_node].indices[i]]).position,
        get<2>(triangles[binary_tree[index_parent_node].indices[i]]).position, binary_tree[index_parent_node].indices[i] };
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
void recursiveStepBvh(int index_parent_node, int level, int max_level) {

    //First we need to calulate the upper and lower point of the current node's axis aligned box
    //Declare two vectors pointing to inf and -inf for the lower and upper vector accordingly
    glm::vec3 lower{ std::numeric_limits<float>::max() };
    glm::vec3 upper{ -std::numeric_limits<float>::max() };

    //Iterate through all the triangles that are in our node in order to find the upper and lower vector
    // of the current axis aligned box
    for (int index : binary_tree[index_parent_node].indices) {

        //Get all the vector positions for each triangle vertex
        glm::vec3 v0 = get<0>(triangles[index]).position;
        glm::vec3 v1 = get<1>(triangles[index]).position;
        glm::vec3 v2 = get<2>(triangles[index]).position;

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
    if (level < max_level && binary_tree[index_parent_node].indices.size() > 1) {

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
        splitBox(index_parent_node, splitAxis, leftVector, rightVector);

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
        recursiveStepBvh(left_pos, level + 1, max_level);
        recursiveStepBvh(right_pos, level + 1, max_level);
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

    triangles.clear();

    //Use the method given in intersection to iterate through and add all of the triangles to our vector
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            std::tuple temp_tri = { v0,v1,v2, mesh.material};
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
    recursiveStepBvh(i, 0, MAX_LEVEL);
    int tot = 0; int cnt = 0;
    for (int i = 0; i < binary_tree.size(); i++) {
        if (binary_tree[i].isLeaf) {
            tot += binary_tree[i].indices.size();
            cnt++;
        }
    }
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display.
int BoundingVolumeHierarchy::numLevels() const
{
    return MAX_LEVEL + 1;
}

/// <summary>
/// Function that takes a hitbox as parameter and draws out the triangle.
/// Important note here, is that in order to avoid errors, when we are using this
/// feature of the visual debugger we disable the restorization of the scene so we only paint 
/// the triangles that we need
/// </summary>
/// <param name="hitInfo"></param>
static void drawTriangle(Vertex v0, Vertex v1, Vertex v2, glm::vec3 colour) {
    glBegin(GL_TRIANGLES);
    //Set the colour of the triangle to the random colour that we have calculated before we call this function
    glColor3f(colour.x, colour.y, colour.z);
    //Assign the values of the three vertices
    glVertex3f(v0.position.x, v0.position.y, v0.position.z);
    glVertex3f(v1.position.x, v1.position.y, v1.position.z);
    glVertex3f(v2.position.x, v2.position.y, v2.position.z);
    //Draw the triangle
    glEnd();
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    //For every node in the binary tree (bvh) we check if its level is the level that is given
    for (Node node : binary_tree) {
        AxisAlignedBox aabb{ node.data };
        if (node.level == level) {
            //if so we draw the Axis Aligned box
            drawAABB(aabb, DrawMode::Wireframe, glm::vec3(1.0f));
        }
        //Also here is implemented the feature of the visual debugger that paints all the triangles within each leaf node a rrandom colour
        if (node.isLeaf && drawTrianglesInLeaf) {
            //If the node is a leaf, and the feature is selected we calclulate a random RGB value using the rand() function
            //from the standard main library
            glm::vec3 randomColour{ (std::rand() % 100) / 100.0f,(std::rand() % 100) / 100.0f ,(std::rand() % 100) / 100.0f };
            //Now for all the triangles inside our leaf we paint them that colour
            for (int i = 0; i < node.indices.size(); i++) {
                Vertex v0 = get<0>(triangles[node.indices[i]]);
                Vertex v1 = get<1>(triangles[node.indices[i]]);
                Vertex v2 = get<2>(triangles[node.indices[i]]);
                drawTriangle(v0, v1, v2, randomColour);
            }
        }
    }
}


/// <summary>
/// Return true if something is hit, returns false otherwise. only find hits if they are closer than t stored
/// First using the binary tree we have constructed we can iterate through all the necessary nodes
/// For the leaves that intersect with our ray we check all the triangles inside of them and store the value/info
/// for the closest one. Then we check for all the spheres in the scene using the function that we  have implemented
/// in the ray_tracing.cpp
/// </summary>
/// <param name="ray"> The ray </param>
/// <param name="hitInfo"> Hit Info, conatins information for the 'hit' triangle, vertices, material and normal</param>
/// <returns> true / false as specified in summary </returns>

bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    //Declare a boolean and set as false (assuming there are no intersections)
    bool hit = false;
    
    //The priority queue, where are we going to store all the visited (intersected) nodes (index and intersection point)
    std::priority_queue<AABBwithIntersection> pq;
    std::priority_queue<AABBwithIntersection> pqLeaves;

    //Push the root into the pq
    pq.push(AABBwithIntersection(std::numeric_limits<float>::max(),0));
    
    //Declare a tempT value which we are going to use when declaring new 'tuples' for our pq
    float tempT = pq.top().t;

    //If there is intersection with the very first AABB then we check for its children
    if (intersectRayWithShape(binary_tree[0].data, ray, tempT)) {
        //While we have intersected AABBs that are not yet traversed we do the following
        while (pq.size() > 0) {
            //Store the index and the t value of the first element of the queue
            int current_index = pq.top().index;
            float t = pq.top().t;
            //Remove first element (parent node)
            pq.pop();
            
            //If needed(asked) we draw the boxes that we are visiting
            if (debugIntersectionAABB) {
                drawAABB(binary_tree[current_index].data, DrawMode::Wireframe, glm::vec3(1.0f, 0.0f, 0.0f));
            }

            //If the node we are currently is a leaf then we check all the triangles
            if (binary_tree[current_index].isLeaf) {
                //If the node is a leaf we push the info to the leaf priority queue
                pqLeaves.push(AABBwithIntersection(t, current_index));
            }
            else {
                tempT = std::numeric_limits<float>::max();
                //Check for in intersection with the left child
                if (intersectRayWithShape(binary_tree[binary_tree[current_index].indices[0]].data, ray, tempT)) {
                    //If there is we push the AABB to our pq
                    pq.push(AABBwithIntersection(tempT, binary_tree[current_index].indices[0]));
                }
                tempT = std::numeric_limits<float>::max();
                //Check for in intersection with the left child
                if (intersectRayWithShape(binary_tree[binary_tree[current_index].indices[1]].data, ray, tempT)) {
                    //If there is we push the AABB to our pq
                    pq.push(AABBwithIntersection(tempT, binary_tree[current_index].indices[1]));
                }
            }
        }
    }
    //Now we will check one-by-one starting from the top of the priority queue all the leaf nodes and their intersections
    //The priority queue was created in a way to strore in the front the element wich is the closest to our ray.
    //Important note is that here we observed that sometimes by stopping to the first node that has a hit with our ray
    //sometimes doesnt work due to floating errors points and also the overlap of two or more boxes, so in order to
    //pass that issue we allowed our loop to check the triangles of the node after the first hit so we can avoid errors when 
    //it comes to ray tracing. Its a small price to pay for accuracy and precission
    while (!pqLeaves.empty()) {
        //Store the index and the t value of the first element of the queue
        int current_index = pqLeaves.top().index;
        float t = pqLeaves.top().t;
        //We use this variable to allow us to check the node after the first hit to avoid errors of floating points etc
        bool flag = false;
        //Remove first element
        pqLeaves.pop();
        //Iterate thorugh all the triangles and update the info for the closest one
        for (int i = 0; i < binary_tree[current_index].indices.size(); i++) {
            //Get the info for all the vertices for each triangle
            Vertex v0 = get<0>(triangles[binary_tree[current_index].indices[i]]);
            Vertex v1 = get<1>(triangles[binary_tree[current_index].indices[i]]);
            Vertex v2 = get<2>(triangles[binary_tree[current_index].indices[i]]);
            //If there is a closer intersection of our ray with a triangle we update the hitinfo and the hit boolean
            flag = hit;
            if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                hitInfo.material = get<3>(triangles[binary_tree[current_index].indices[i]]);
                hit = true;
                hitInfo.v0 = v0;
                hitInfo.v1 = v1;
                hitInfo.v2 = v2;
            }
            //If in the previus node there was a hit we exit from the loop
            if (flag) break;
        }
    }
     
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    //return whether there was a hit of our ray with the scene
    return hit;
}
