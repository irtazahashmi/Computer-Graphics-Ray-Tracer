#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>


BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
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
    AxisAlignedBox aabb { glm::vec3(0.0f), glm::vec3(0.0f, 1.05f, 1.05f) };
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

        drawRay({ hitInfo.v0.position, hitInfo.v0.normal, ray.t }, glm::vec3(0.0f, 1.0f, 0.0f));
        drawRay({ hitInfo.v1.position, hitInfo.v1.normal, ray.t }, glm::vec3(0.0f, 1.0f, 0.0f));
        drawRay({ hitInfo.v2.position, hitInfo.v2.normal, ray.t }, glm::vec3(0.0f, 1.0f, 0.0f));

        glm::vec3 p = ray.origin + ray.t * ray.direction;
        float A = glm::length(glm::cross(hitInfo.v1.position - p, hitInfo.v2.position - p)) / 2;
        float B = glm::length(glm::cross(hitInfo.v0.position - p, hitInfo.v2.position - p)) / 2;
        float C = glm::length(glm::cross(hitInfo.v0.position - p, hitInfo.v1.position - p)) / 2;
        float totalArea = glm::length(glm::cross(hitInfo.v2.position - hitInfo.v0.position, hitInfo.v1.position - hitInfo.v0.position)) / 2;
        float alpha = A / totalArea;
        float beta = B / totalArea;
        float gamma = C / totalArea;
        hitInfo.normal = alpha * hitInfo.v0.normal + beta * hitInfo.v1.normal + gamma * hitInfo.v2.normal;

        drawRay({ p, hitInfo.normal, ray.t }, glm::vec3(0.0f, 0.0f, 1.0f));

    }
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}
