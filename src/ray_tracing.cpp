#include "ray_tracing.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

/*
* Checks whether a given point is inside a given triangle.
*
* This is done by finding whether the point p lies inside all three edges of the triangle.
*
* @param 3 vectors, indicating the 3 vartices of the triangle, 1 vector indicating the point we want to check
*           and the normal of the triangle
* @return boolean value, true if the point is inside the triangle, false otherwise
*/
bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
    // The three edges of the triangle
    glm::vec3 triangleEdgeOne = v1 - v0;
    glm::vec3 traingleEdgeTwo = v2 - v1;
    glm::vec3 triangleEdgeThree = v0 - v2;

    // The point relative to triangle edges
    glm::vec3 trianglePointOne = p - v0;
    glm::vec3 trianglePointTwo = p - v1;
    glm::vec3 trianglePointThree = p - v2;

    // checking if the point p is inside all three edges of the triangle
    float insideEdgeOne = glm::dot(n, glm::cross(triangleEdgeOne, trianglePointOne));
    float insideEdgeTwo = glm::dot(n, glm::cross(traingleEdgeTwo, trianglePointTwo));
    float insideEdgeThree = glm::dot(n, glm::cross(triangleEdgeThree, trianglePointThree));

    // point p inside the trianlge conditions
    // small epsilon for precision
    double epsilon = (double)1E-10;
    if (insideEdgeOne > epsilon && insideEdgeTwo > epsilon && insideEdgeThree > epsilon) {
        // point p inside triangle
        return true;
    }

    // point p outside the triangle
    return false;
}

/*
* Checks whether the ray hits the given plane, and change the ray.t value (if needed)
*
* The following was implemented using the equation that was given during the lecture
*
* @param plane, and the ray
* @return boolean value, true if the ray hits the plane , false otherwise
*/
bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    // 1) no intersection is found or t <= 0: return false
    // 2) found intersection point p
    //    a) if p is closer to the ray origin then current intersection point : return true and update ray.t
    //    b) else return false

    // Getting all the variables to calculate the intersection point t
    // t = (D - dot(o, n)) / dot(d, n)
    // plane vars
    float D = plane.D;
    glm::vec3 n = plane.normal;
    //ray vars
    glm::vec3 o = ray.origin;
    glm::vec3 d = ray.direction;

    // calculate t components
    float numerator = D - glm::dot(o, n);
    float denominator = glm::dot(d, n);

    // if abs(denominator) < epsilon, ray is parrallel is to plane -> no intersection
    float epsilon = (float)1E-6;
    if (glm::abs(denominator) < epsilon) {
        return false;
    }

    // calculate intersection
    float t = numerator / denominator;

    // if intersection
    float intersect_epsilon = (float)1E-4;
    if (t > intersect_epsilon) {
        // if the intersection is closer than a previous intersection, update t and return true
        if (t < ray.t) {
            ray.t = t;
            return true;
        }
    }

    // else no intersection, return false
    return false;
}

/*
* Returns a plane from the given 3  points
*
* The following method useses the formulae that were given during the lecture
*
* @param 3 vectors, indicating the 3 points (vertices) of the triangle
* @return plane with all the neccesary information generated
*/
Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;

    // the cross product of difference of the triangle vertices is the normal of the plane:
    // normalize ( cross((v0 - v2), (v1 - v2)) ) = normal of plane
    glm::vec3 edgeAC = v0 - v2;
    glm::vec3 edgeBC = v1 - v2;
    glm::vec3 crossProduct = glm::cross(edgeAC, edgeBC);
    glm::vec3 normalOfPlane = glm::normalize(crossProduct);
    plane.normal = normalOfPlane;

    // D is equal to dot(p, n)
    glm::vec3 p = v0;
    glm::vec3 n = normalOfPlane;
    float D = glm::dot(p, n);
    plane.D = D;

    // return plane
    return plane;
}

/*
* It checks whether the ray intersects with triangle , and if needed updates the ray.t value
*
* First we calculate the intersection between the ray and plane created by the triangle (if any),
* and then we check if that point lies inside the triangle.
*
* All the calculations use formulae that were given during the lecture
*
* @param 3 vectors representing the 3 vertices of the triangle, the ray, and the HitInfo
* @return boolean variable , true if there is an intersection of the ray and triangle false otherwise
*/
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    // TIP: Don’t forget to roll back the ray.t value modified in the intersectRayWithPlane method if 
    // the point is not inside the triangle

    // First, compute the plane containing the triangle.
    Plane planeTriangle = trianglePlane(v0, v1, v2);

    // Second, compute the intersection point of the rayand the plane.
    float previousT = ray.t;
    bool intersectPlaneSuccess = intersectRayWithPlane(planeTriangle, ray);
    bool intersectTriangleSuccess = false;

    // If there is an (plane) intersection, the third step is to check if the point is inside the triangle.
    if (intersectPlaneSuccess) {
        glm::vec3 n = planeTriangle.normal;
        glm::vec3 p = ray.origin + ray.direction * ray.t;

        // check if point inside the triangle
        intersectTriangleSuccess = pointInTriangle(v0, v1, v2, n, p);

        // the point is in the triangle - intersection ray triangle - success
        if (intersectTriangleSuccess) {
            // storing t for the the nearest triangle intersection
            if (previousT < ray.t) {
                ray.t = previousT;
            }

            // update the normal for hitInfo
            if (glm::dot(planeTriangle.normal, ray.direction) < 0) {
                hitInfo.normal = planeTriangle.normal;
            }
            else {
                hitInfo.normal = -planeTriangle.normal;
            }

            // intersection sucessful
            return true;
        }
    }

    // no intersection/the point is not inside the triangle -> roll back 
    ray.t = previousT;
    // intersection of ray triangle failed
    return false;
}

/*
* Checks whether the given ray intersects with a sphere. Also updates if needed the ray.t value
*
* We calculate a, b c, using formulae given in the lecture. Then we reduce the equation in
* the form of (math done by hand):
*
*   a*t^2 + b*t + c =0
*
* Next we find (if any) the value(s) of the t. And then we check if its not behind the ray
* and update if needed the ray.t value.
*
* @param the sphere, the ray , and hit info
*
* @return If any value(s) exist(s) we return true, otherwise false
*
*/
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    // new origin is now dependent on the center of the sphere
    glm::vec3 newOrigin = ray.origin - sphere.center;

    // calculate a
    float xDirection = (ray.direction.x * ray.direction.x);
    float yDirection = (ray.direction.y * ray.direction.y);
    float zDirection = (ray.direction.z * ray.direction.z);
    float a = xDirection + yDirection + zDirection;

    // calculate b
    float xComponent = ray.direction.x * newOrigin.x;
    float yComponent = ray.direction.y * newOrigin.y;
    float zComponent = ray.direction.z * newOrigin.z;
    float b = 2.0f * (xComponent + yComponent + zComponent);

    // calculate c
    float xOrigin = (newOrigin.x * newOrigin.x);
    float yOrigin = (newOrigin.y * newOrigin.y);
    float zOrigin = (newOrigin.z * newOrigin.z);
    float rSquared = (sphere.radius * sphere.radius);
    float c = xOrigin + yOrigin + zOrigin - rSquared;


    // discriminant test: if its < 0 -> no solution
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0) {
        return false;
    }

    // retrieving solutions using the quadaratic equation
    float numeratorOne = -b + glm::sqrt(discriminant);
    float numeratorTwo = -b - glm::sqrt(discriminant);
    float denominaotor = 2.0f * a;
    float solutionOne = numeratorOne / denominaotor;
    float solutionTwo = numeratorTwo / denominaotor;

    // if we have 2 solutions
    if (solutionOne > 0 && solutionTwo > 0) {
        // find the closest intersection point
        float closestT = glm::min(solutionOne, solutionTwo);
        if (closestT < ray.t) {
            // update closest t
            ray.t = closestT;

            // update material
            hitInfo.material = sphere.material;

            //update normal
            glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
            glm::vec3 normalIntersection = glm::normalize(intersectionPoint - sphere.center);
            hitInfo.normal = normalIntersection;
        }
        // intersection sucess
        return true;
    }

    // if we have one solution: case 1
    if (solutionOne > 0) {
        // find closest t
        if (solutionOne < ray.t) {
            // update closest t
            ray.t = solutionOne;

            // update material
            hitInfo.material = sphere.material;

            //update normal
            glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
            glm::vec3 normalIntersection = glm::normalize(intersectionPoint - sphere.center);
            hitInfo.normal = normalIntersection;
        }
        return true;
    }


    // if we have one solution: case 2
    if (solutionTwo > 0) {
        // find closest t
        if (solutionTwo < ray.t) {
            // update closest t
            ray.t = solutionTwo;

            // update material
            hitInfo.material = sphere.material;

            //update normal
            glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
            glm::vec3 normalIntersection = glm::normalize(intersectionPoint - sphere.center);
            hitInfo.normal = normalIntersection;
        }
        return true;
    }

    // no intersection occurred
    return false;
}

/*
* It checks whether the ray intersects with the axis-aligned box.
*
* Using the equations from the lecture, checking if the ray intersects with the axis
* aligned box. Also checks if the intersection happened behind or in front of the origin
* of the ray.
*
* @param axisAlignedBox and the ray
* @return true if there is an intersection, false otherwise
*/
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray, float& t)
{
    // using txmin = (xmin - ox) / dx as an example to find all six sides of the box -> src: lecture slides
    // X components
    float tXMin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float tXMax = (box.upper.x - ray.origin.x) / ray.direction.x;
    // Y components
    float tYMin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tYMax = (box.upper.y - ray.origin.y) / ray.direction.y;
    // Z component
    float tZMin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float tZMax = (box.upper.z - ray.origin.z) / ray.direction.z;

    // tXIn = min(txmin, txmax) ---- tXOut = max(txmin, txmax) -> src: lecture slides
    // X components
    float tXIn = std::min(tXMin, tXMax);
    float tXOut = std::max(tXMin, tXMax);
    // Y
    float tYIn = std::min(tYMin, tYMax);
    float tYOut = std::max(tYMin, tYMax);
    // Z
    float tZIn = std::min(tZMin, tZMax);
    float tZOut = std::max(tZMin, tZMax);

    // finding global in and out -- lecture slides
    float tIn = std::max(tXIn, std::max(tYIn, tZIn));
    float tOut = std::min(tXOut, std::min(tYOut, tZOut));

    // ray misses -- no intersection conditions
    if (tIn > tOut || tOut < 0) {
        return false;
    }
    // there is an intersection of ray and AABB
    if (tIn < ray.t) {
        // if the intersection is behind the origin of the ray, then update ray.t as tOut
        if (tIn < 0) {
            t = tOut;
        }
        else {
            // else the intersection is infront. update ray.t as tIn (where the ray hits the box) and return true
            t = tIn;
        }

        return true;
    }

    // no intersection
    return false;
}
