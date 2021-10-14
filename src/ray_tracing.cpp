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
* Calculates the length of a vector.
* 
* Using the Pythagorean theorem for two points in a 3-dimensional space
* 
* @param the vector
* @return the length of the vector
*/
float lengthV(const glm::vec3& v) {
    return glm::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/*
* Calculates the area of the triangle
* 
* Using the answer given to the following StackExchange question
* https://math.stackexchange.com/questions/128991/how-to-calculate-the-area-of-a-3d-triangle
* 
* @param 3 vectors, indicating the 3 points (vertices of the triangle)
* @return the area of the triangle as a float (we assume that such triangles exists so no need for further checks)
*/
float areaOfTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    //Daclaring the answer variable
    float ans;

    //Use the lengthV() to obtain |AB||AC|
    ans = lengthV(v1 - v0) * lengthV(v2 - v0);

    //calculating the cosine of the angle (a) from the definition of the dot product
    float theta = glm::dot(v1 - v0, v2 - v0) / ans;

    /*
    returning the final result(area)
    Note here, glm::sqrt(1 - theta * theta) calculates the sine of the angle 
    by rewritting sin^2(x) + cos^2(x) = 1 -> sin(x) = sqrt( 1 - cos^2(x) )
    then applying the given formula
    */
    return 0.5f * ans * glm::sqrt(1 - theta * theta);
}

/*
* Checks whether a given point is inside a given triangle
* 
* By using the equation of the barycentric coordinates that uses the areas of the sub-triangles
* which was given during the lecture
* 
* @param 3 vectors, indicating the 3 vartices of the triangle, 1 vector indicating the point we want to check
*           and the normal of the triangle
* @return boolean value, true if the point is inside the triangle, false otherwise
*/
bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
    // a,b,c are the 3 parts of the given equation respectevly
    float a = areaOfTriangle(v1, v2, p) / areaOfTriangle(v0, v1, v2);
    float b = areaOfTriangle(v0, v2, p) / areaOfTriangle(v0, v1, v2);
    float c = areaOfTriangle(v0, v1, p) / areaOfTriangle(v0, v1, v2);

    /*
    * In order for the point to be considered inside the triangle
    * a+b+c must be equal to one (+- error margin) also a,b,c must be non negatives
    */
    if (a >= 0.f && b >= 0.f && c >= 0.f && glm::abs(a + b + c - 1.0f) <= 1e-3) {
        return true;
    }
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
    //Check if the dot product of the normal of the plane, and the direction of the ray is not zero (+- error margin)
    if (glm::abs(glm::dot(glm::normalize(plane.normal), ray.direction)) > 1e-6) {
        //Also we check whether the calculated t value is less than zero, if so we return false (behind the ray)
        if ((plane.D - glm::dot(ray.origin, glm::normalize(plane.normal))) / glm::dot(ray.direction, glm::normalize(plane.normal)) < 0) {
            return false;
        }
        //If the non-zero t.value has smaller value from the current t value we update it
        ray.t = glm::min(ray.t, (plane.D - glm::dot(ray.origin, glm::normalize(plane.normal))) / glm::dot(ray.direction, glm::normalize(plane.normal)));
        return true;
    }
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
    //Calculating the normal
    plane.normal = glm::normalize(glm::cross((v0 - v2), (v1 - v2)));

    //Calculating the d value
    plane.D = glm::dot(plane.normal, v0);
    
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
    //Calculating the plane using the trianglePlane() function
    Plane plane = trianglePlane(v0, v1, v2);

    //First we check if the ray intersects with the plane (described above)
    if (glm::abs(glm::dot(glm::normalize(plane.normal), ray.direction)) > 1e-6) {
        //Calculating the intersection point of the ray and the plane
        glm::vec3 intersectPoint = ray.origin + ray.direction * (glm::dot(v0 - ray.origin, plane.normal) / glm::dot(ray.direction, plane.normal));
        
        //Now we check if the point that we calculated before lies inside the triangle
        if (pointInTriangle(v0, v1, v2, plane.normal, intersectPoint)) {
            // Also we need to check that the t value is positive ray.t>0
            if ((plane.D - glm::dot(ray.origin, glm::normalize(plane.normal))) / glm::dot(ray.direction, glm::normalize(plane.normal)) < 0) {
                //Triangle behind ray origin
                return false;
            }
            //If needed we update the ray.t value to get the smallest positive t value
            ray.t = glm::min(ray.t, (plane.D - glm::dot(ray.origin, glm::normalize(plane.normal))) / glm::dot(ray.direction, glm::normalize(plane.normal)));
            return true;
        }
    }
    //No hit with the triangle plane
    return false;
}

/*
* Same function as before but there is no hitInfo as parameter
*
* @param 3 vectors representing the 3 vertices of the triangle, the ray
* @return boolean variable , true if there is an intersection of the ray and triangle false otherwise
*/
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray)
{
    //Calculating the plane using the trianglePlane() function
    Plane plane = trianglePlane(v0, v1, v2);

    //First we check if the ray intersects with the plane (described above)
    if (glm::abs(glm::dot(glm::normalize(plane.normal), ray.direction)) > 1e-6) {
        //Calculating the intersection point of the ray and the plane
        glm::vec3 intersectPoint = ray.origin + ray.direction * (glm::dot(v0 - ray.origin, plane.normal) / glm::dot(ray.direction, plane.normal));

        //Now we check if the point that we calculated before lies inside the triangle
        if (pointInTriangle(v0, v1, v2, plane.normal, intersectPoint)) {
            // Also we need to check that the t value is positive ray.t>0
            if ((plane.D - glm::dot(ray.origin, glm::normalize(plane.normal))) / glm::dot(ray.direction, glm::normalize(plane.normal)) < 0) {
                //Triangle behind ray origin
                return false;
            }
            //If needed we update the ray.t value to get the smallest positive t value
            ray.t = glm::min(ray.t, (plane.D - glm::dot(ray.origin, glm::normalize(plane.normal))) / glm::dot(ray.direction, glm::normalize(plane.normal)));
            return true;
        }
    }
    //No hit with the triangle plane
    return false;
}

/*
* Checks whether the given ray intersects with a sphere. Also updates if needed the ray.t value
* 
* Equation of the sphere:
* (x - a)^2 + (y - b)^2 + (z - c)^2 = radius^2
* 
* Substituting;
* a = ray.origin.x + ray.direction.x *  ray.t
* b = ray.origin.y + ray.direction.y *  ray.t
* c = ray.origin.z + ray.direction.z *  ray.t
* 
* And then reduce the equation in the form of (math done by hand)
* 
*           a*t^2 + b*t + c =0
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
    //a , b , c of the quadratic formula
    float a = glm::pow(ray.direction.x, 2) + glm::pow(ray.direction.y, 2) + glm::pow(ray.direction.z, 2);
    float b = -2.f * (ray.direction.x * sphere.center.x + ray.direction.y * sphere.center.y + ray.direction.z * sphere.center.z)
        + 2.f * (ray.direction.x * ray.origin.x + ray.direction.y * ray.origin.y + ray.direction.z * ray.origin.z);
    float c = glm::pow(sphere.center.x, 2) + glm::pow(sphere.center.y, 2) + glm::pow(sphere.center.z, 2)
        - glm::pow(sphere.radius, 2)
        + glm::pow(ray.origin.x, 2) + glm::pow(ray.origin.y, 2) + glm::pow(ray.origin.z, 2)
        - 2.f * (sphere.center.x * ray.origin.x + sphere.center.y * ray.origin.y + sphere.center.z * ray.origin.z);
    
    //Check if the discriminant is non-negative that means there are real values for t
    if (b * b - 4 * a * c >= 0) {
        //Calulate the values of t
        float t1 = (-b + glm::sqrt(b * b - 4 * a * c)) / (2 * a);
        float t2 = (-b - glm::sqrt(b * b - 4 * a * c)) / (2 * a);
        
        //Check if both points are behind the ray, if so we return false
        if (t1 < 0 && t2 < 0) {
            return false;
        }

        //Check if one of the points is behind the ray and if needed update the ray.t value
        if (t1 < 0) {
            ray.t = glm::min(ray.t, t2);  
            return true;
        }
        if (t2 < 0) {
            ray.t = glm::min(ray.t, t1);
            return true;
        }
        //otherwise both points hit the sphere, in that case we update (if needed) the ray.t value
        // to take the smallest non-negative t value
        ray.t = glm::min(ray.t, glm::min(t1, t2));
        return true;
    }
    //No intersection return false
    return false;
}

/*
* It checks whether the ray intersects with the axis-aligned box
* 
* Split the box into 12 triangles and check one by one if it intersects
* 
* @param axisAlignedBox and the ray
* @return true if there is an intersection, false otherwise
*/
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    bool hit = false;

    float x0 = box.lower.x, y0 = box.lower.y, z0 = box.lower.z, x1 = box.upper.x, y1 = box.upper.y, z1 = box.upper.z;

    hit |= intersectRayWithTriangle(glm::vec3{ x0,y0,z0 }, glm::vec3{ x0,y1,z0 }, glm::vec3{ x0,y0,z1 }, ray);
    hit |= intersectRayWithTriangle(glm::vec3{ x0,y0,z0 }, glm::vec3{ x0,y0,z1 }, glm::vec3{ x1,y0,z0 }, ray);
    hit |= intersectRayWithTriangle(glm::vec3{ x0,y0,z0 }, glm::vec3{ x0,y1,z0 }, glm::vec3{ x1,y0,z0 }, ray);

    hit |= intersectRayWithTriangle(glm::vec3{ x0,y1,z1 }, glm::vec3{ x0,y1,z0 }, glm::vec3{ x0,y0,z1 }, ray);
    hit |= intersectRayWithTriangle(glm::vec3{ x1,y0,z1 }, glm::vec3{ x0,y0,z1 }, glm::vec3{ x1,y0,z0 }, ray);
    hit |= intersectRayWithTriangle(glm::vec3{ x1,y1,z0 }, glm::vec3{ x0,y1,z0 }, glm::vec3{ x1,y0,z0 }, ray);

    hit |= intersectRayWithTriangle(glm::vec3{ x0,y1,z1 }, glm::vec3{ x0,y1,z0 }, glm::vec3{ x1,y1,z0 }, ray);
    hit |= intersectRayWithTriangle(glm::vec3{ x1,y0,z1 }, glm::vec3{ x1,y1,z0 }, glm::vec3{ x1,y0,z0 }, ray);
    hit |= intersectRayWithTriangle(glm::vec3{ x0,y0,z1 }, glm::vec3{ x0,y1,z1 }, glm::vec3{ x1,y0,z1 }, ray);

    hit |= intersectRayWithTriangle(glm::vec3{ x1,y1,z1 }, glm::vec3{ x0,y1,z1 }, glm::vec3{ x1,y0,z1 }, ray);
    hit |= intersectRayWithTriangle(glm::vec3{ x1,y1,z1 }, glm::vec3{ x1,y0,z1 }, glm::vec3{ x1,y1,z0 }, ray);
    hit |= intersectRayWithTriangle(glm::vec3{ x1,y1,z1 }, glm::vec3{ x0,y1,z1 }, glm::vec3{ x1,y1,z0 }, ray);

    return hit;
}
