#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "ray_tracing.h"
#include "screen.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
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

// This is the main application. The code in here does not need to be modified.
constexpr glm::ivec2 windowResolution { 800, 800 }; // window resolution
const std::filesystem::path dataPath { DATA_DIR };

// Declare of some global boolean variables which will be needed for viual debug
bool debugShadowRay{ false };
bool debugAreaLights = { false };
bool debugIntersectionAABB{ false };
bool debugNormalInterpolation{ false };
bool debugTextures{ false };
bool debugBloomFilter{ false };
bool debugMipmapping{ false };
bool debugTransparency{ false };
bool drawTrianglesInLeaf{ false };

int mipmappingLevel = 0;
int mipmappingReduceResolution = 0;

enum class ViewMode {
    Rasterization = 0,
    RayTracing = 1
};

/*
* Calculates the diffuse term of the phong model using:
*
*           diffuse = kd * dot(N, L) 
* 
* where N is the normal and L is the light direction.
* 
* @param a point light, the hitInfo object and the ray
* @return a vector of the diffuse component of the phong model
*/
static glm::vec3 calculateDiffuse(PointLight pointlight, HitInfo hitInfo, Ray ray) {
    // light attributes
    glm::vec3 lightColor = pointlight.color;
    glm::vec3 lightPos = pointlight.position;

    // intersection point
    glm::vec3 hitPos = ray.origin + ray.t * ray.direction;

    // calculate light direction
    glm::vec3 lightDir = lightPos - hitPos;
    lightDir = glm::normalize(lightDir);

    // normalized normal
    glm::vec3 normalizedNormal = glm::normalize(hitInfo.normal);

    float dotProductNormailLightVector = glm::dot(normalizedNormal, lightDir);

    // if light infront, find diffuse component
    if (dotProductNormailLightVector > 0) {
        glm::vec3 diffuse = hitInfo.material.kd * dotProductNormailLightVector;
        return diffuse;
    }

    // else return black
    return glm::vec3{ 0.0f };
}

/*
* Calculates the specular term of the phong model using:
*
*           specular = ks * dot(N, H)^shininess
*
* where N is the normal and H is the reflection vector
*
* @param a point light, the hitInfo object and the ray
* @return a vector of the specular component of the phong model
*/
static glm::vec3 calculateSpecular(PointLight pointlight, HitInfo hitInfo, Ray ray) {
    // light attributes
    glm::vec3 lightColor = pointlight.color;
    glm::vec3 lightPos = pointlight.position;

    // intersection point
    glm::vec3 hitPos = ray.origin + ray.t * ray.direction;

    // light dir
    glm::vec3 lightDir = lightPos - hitPos;
    lightDir = glm::normalize(lightDir);

    //normal
    glm::vec3 normalizedNormal = glm::normalize(hitInfo.normal);

    // reflection vec
    glm::vec3 H = 2 * glm::dot(lightDir, normalizedNormal) * normalizedNormal - lightDir;
    H = glm::normalize(H);

    //calculate specular
    float shininess = hitInfo.material.shininess;
    glm::vec3 ks = hitInfo.material.ks;
    glm::vec3 specular = ks * glm::pow(glm::max(glm::dot(normalizedNormal, H), 0.0f), shininess);
    return specular;
}

/*
* Checks if the light ray has the same intersection point as the ray. If it does, then
* that point is lit, else it is in shadow.

* @param bvh, the ray and the light position
* @return if the pixel is lit (true) or in shadow (false)
*/
static bool hitLightSuccess(const BoundingVolumeHierarchy& bvh, Ray ray, glm::vec3 lightPos) {
    // create a temporary light ray
    Ray tempLightRay;
    tempLightRay.origin = lightPos;
    glm::vec3 intersectionPointRay = ray.origin + ray.t * ray.direction;
    tempLightRay.direction = glm::normalize(intersectionPointRay - tempLightRay.origin);

    // intersect the light ray with the bvh
    HitInfo hitInfo;
    bvh.intersect(tempLightRay, hitInfo);

    glm::vec3 intersectionPointLight = tempLightRay.origin + tempLightRay.t * tempLightRay.direction;
    intersectionPointRay = ray.origin + ray.t * ray.direction;


    float epsilon = (float) 1E-4;;
    // if light intersection point and ray intersection point the same -> we can see the light -> lit
    glm::vec3 differenceIntersection = intersectionPointLight - intersectionPointRay;
    if (differenceIntersection.x < epsilon && 
        differenceIntersection.y < epsilon && 
        differenceIntersection.z < epsilon) {
        return true;
    }

    // else there is darkness -> we can't see light from the pixel
    return false;
}

/// <summary>
/// Calculate the barycentric weights using the three vertices of the triangle and the intersection point.
/// It is always already checked before calling this method whether the intersection point is actually in the triangle
/// with bvh.intersect.
/// </summary>
/// <param name="v0">First vertex of triangle</param>
/// <param name="v1">Second vertex of triangle</param>
/// <param name="v2">Third vertex of triangle</param>
/// <param name="p">Intersection point</param>
/// <returns>A tuple with the three barycentric weights</returns>
static std::tuple<float, float, float> getBarycentricWeights(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3& p) {
    // Calculate the area of each sub-triangle
    float A = glm::length(glm::cross(v1 - p, v2 - p)) / 2;
    float B = glm::length(glm::cross(v0 - p, v2 - p)) / 2;
    float C = glm::length(glm::cross(v0 - p, v1 - p)) / 2;

    // Calculate the total area
    float totalArea = glm::length(glm::cross(v2 - v0, v1 - v0)) / 2;

    // Calculate the weights
    float alpha = A / totalArea;
    float beta = B / totalArea;
    float gamma = C / totalArea;

    // Check whether p is inside the triangle
    return  { alpha, beta, gamma };
}

/// <summary>
/// Calculate the interpolated normal given the information from the triangle
/// and the ray.
/// </summary>
/// <param name="hitInfo">The info from the triangle where the ray hits</param>
/// <param name="ray">The ray where you want to calculate the interpolated normal</param>
static void drawInterpolatedNormal(HitInfo& hitInfo, Ray& ray) {
    // Draw the normals from the vertices of the triangle
    drawRay({ hitInfo.v0.position, hitInfo.v0.normal, ray.t }, glm::vec3(0.0f, 1.0f, 0.0f));
    drawRay({ hitInfo.v1.position, hitInfo.v1.normal, ray.t }, glm::vec3(0.0f, 1.0f, 0.0f));
    drawRay({ hitInfo.v2.position, hitInfo.v2.normal, ray.t }, glm::vec3(0.0f, 1.0f, 0.0f));

    // Calculate the intersection point
    glm::vec3 p = ray.origin + ray.t * ray.direction;

    // Calculate the barycentric weights
    std::tuple<float, float, float> weights = getBarycentricWeights(hitInfo.v0.position, hitInfo.v1.position, hitInfo.v2.position, p);
    float alpha = get<0>(weights);
    float beta = get<1>(weights);
    float gamma = get<2>(weights);

    // Calculate the interpolated normal by using the normals of the vertices and the barycentric weights
    hitInfo.normal = alpha * hitInfo.v0.normal + beta * hitInfo.v1.normal + gamma * hitInfo.v2.normal;

    // Draw the interpolated normal
    drawRay({ p, hitInfo.normal, ray.t }, glm::vec3(0.0f, 0.0f, 1.0f));
}

/// <summary>
/// Function that takes a hitbox as parameter and draws out the triangle
/// We use this function to draw the final triangle as was asked for the visual debug
/// of the bvh generation
/// </summary>
/// <param name="hitInfo"> Contains info for the triangle (we  may only use the vertices positions and the material)</param>
static void drawDebugTriangle(HitInfo& hitInfo) {
    glBegin(GL_TRIANGLES);

    glColor3f(1, 1, 1);
    glNormal3f(hitInfo.normal.x, hitInfo.normal.y, hitInfo.normal.z);

    glVertex3f(hitInfo.v0.position.x, hitInfo.v0.position.y, hitInfo.v0.position.z);
    glVertex3f(hitInfo.v1.position.x, hitInfo.v1.position.y, hitInfo.v1.position.z);
    glVertex3f(hitInfo.v2.position.x, hitInfo.v2.position.y, hitInfo.v2.position.z);

    glEnd();
}

/// <summary>
/// We check all the light sources of our scene, around the endpoint of the ray. By using the 
/// Phong shading model and applying if needed any external textures we can calculate the final of the endpoint of our ray.
/// If needed (i.e. specular material) we call this funcion recursively which its partial result will contribute to the specular component 
/// of the initial point(pixel)
/// </summary>
/// <param name="scene"> Our scene which contains all the meshes</param>
/// <param name="bvh"> BVH used when we ray-tracing the image to calculate if the ray intersects with the scene</param>
/// <param name="ray"> The ray that points at the point in 3d space which we want to calclulate the color for</param>
/// <param name="level"> The current level of the recursive step (0) if its the first </param>
/// <param name="maxLevel"> The maximum level of recursive steps we want to have per pixel</param>
/// <param name="hitInfo"> Stores info about the triangle that we intersect, such as its vertices, its normal and its material</param>
/// <returns></returns>
static glm::vec3 recursive_ray_tracer(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray, int level, int maxLevel, HitInfo& hitInfo) {
    if (bvh.intersect(ray, hitInfo)) {
        glm::vec3 finalColor{ 0.f };

        if (debugNormalInterpolation) {
            drawInterpolatedNormal(hitInfo, ray);
        }
        
        if (debugTextures) {
            // First get the ray from cameraview to the pixel
            glm::vec3 p = ray.origin + ray.t * ray.direction;

            // Calculate the barycentric weights
            std::tuple<float, float, float> weights = getBarycentricWeights(hitInfo.v0.position, hitInfo.v1.position, hitInfo.v2.position, p);
            float alpha = get<0>(weights);
            float beta = get<1>(weights);
            float gamma = get<2>(weights);

            // Calculate the texture coordinates using the barycentric weights
            glm::vec2 textureCoordinates = alpha * hitInfo.v0.texCoord + beta * hitInfo.v1.texCoord + gamma * hitInfo.v2.texCoord;

            // The image used for the texture, uncomment the bricks one and you'll get the bricks pattern
            //Image image = Image("../../../data/bricks.jpg");
            Image image = Image("../../../data/default.png");

            // Calculate the texel
            return image.getTexel(textureCoordinates);
        }
        else {

            // for all the lights in the scene
            for (const auto& light : scene.lights) {

                // POINT LIGHT
                if (std::holds_alternative<PointLight>(light)) {
                    const PointLight pointlight = std::get<PointLight>(light);

                    // Hard shdow - if the point is in light, calculate color, else in shadow.
                    if (hitLightSuccess(bvh, ray, pointlight.position) || debugTransparency) {
                        finalColor += pointlight.color * calculateDiffuse(pointlight, hitInfo, ray);
                        finalColor += pointlight.color * calculateSpecular(pointlight, hitInfo, ray);
                    }
                    else {
                        if (debugShadowRay) {
                            // debug shadow ray: shadow ray occluded - hits something other than light -> red ray
                            glm::vec3 intersectionPointRay = ray.origin + ray.t * ray.direction;

                            Ray shadowRay;
                            shadowRay.origin = intersectionPointRay;
                            shadowRay.direction = -(intersectionPointRay - pointlight.position);
                            HitInfo shadowRayInfo;
                            bvh.intersect(shadowRay, shadowRayInfo);
                            drawRay(shadowRay, glm::vec3(1.0f, 0.0f, 0.0f));
                        }
                    }

                }
                // SEGMENT LIGHT
                else if (std::holds_alternative<SegmentLight>(light)) {
                    const SegmentLight segmentlight = std::get<SegmentLight>(light);

                    // divide segment lights into 20 samples
                    int sampleSize = 20;
                    float alpha = 0.05f;

                    glm::vec3 lightZeroPos = segmentlight.endpoint0;
                    glm::vec3 lightOnePos = segmentlight.endpoint1;
                    glm::vec3 lightZeroColor = segmentlight.color0;
                    glm::vec3 lightOneColor = segmentlight.color1;

                    //Split the segment to equals steps
                    glm::vec3 x_step = (lightOnePos - lightZeroPos) / (float)sampleSize;

                    for (int i = 0; i < sampleSize; i++) {

                        // random sampling by adding jittering to each step
                        float e = std::rand() % 100 / (float)100;
                        float new_alpha = (float)(i + e) * alpha;

                        //Calculating the final position of each point
                        glm::vec3 currPos = lightZeroPos + ((float)i + e) * x_step;

                        // linear interpolation to calculate its color
                        glm::vec3 currColor = ((1 - new_alpha) * lightZeroColor + (new_alpha)*lightOneColor);

                        //We create a new PointLight var in order to be able to reuse the functions we have already implemented for the PointLight sources eariler in the assignment
                        PointLight currPointLight = { currPos, currColor };

                        // SOFT SHADOWS - treat each step light as point light
                        if (hitLightSuccess(bvh, ray, currPointLight.position)) {
                            finalColor += (currPointLight.color * calculateDiffuse(currPointLight, hitInfo, ray) * alpha);
                            finalColor += (currPointLight.color * calculateSpecular(currPointLight, hitInfo, ray) * alpha);

                            // debug ray: segment lights
                            // drawing all the sampled rays that hit the light with their color
                            if (debugAreaLights) {
                                Ray tempLightRay;
                                tempLightRay.origin = currPos;
                                glm::vec3 intersectionPointRay = ray.origin + ray.t * ray.direction;
                                tempLightRay.direction = glm::normalize(intersectionPointRay - tempLightRay.origin);
                                HitInfo hitInfo;
                                bvh.intersect(tempLightRay, hitInfo);
                                drawRay(tempLightRay, currColor);
                            }
                        }
                        else {
                            if (debugAreaLights) {
                                // if the rays are not hitting the light, we make them red
                                glm::vec3 intersectionPointRay = ray.origin + ray.t * ray.direction;

                                Ray shadowRay;
                                shadowRay.origin = intersectionPointRay;
                                shadowRay.direction = -(intersectionPointRay - currPos);
                                HitInfo shadowRayInfo;
                                bvh.intersect(shadowRay, shadowRayInfo);
                                drawRay(shadowRay, glm::vec3(1.0f, 0.0f, 0.0f));
                            }
                        }
                    }

                }
                // PARALLELOGRAM LIGHT
                else if (std::holds_alternative<ParallelogramLight>(light)) {
                    const ParallelogramLight parallelogramlight = std::get<ParallelogramLight>(light);

                    // divide parallelogram lights into 10 samples for x and y
                    int sampleSize = 10;
                    float alpha = 0.1f;


                    //Calculating the three vertices of the parallelogram
                    glm::vec3 vertexZero = parallelogramlight.v0; // v0
                    glm::vec3 vertexOne = vertexZero + parallelogramlight.edge01; // vo + v1
                    glm::vec3 vertexTwo = vertexZero + parallelogramlight.edge02; // vo + v2

                    //Assigning the values of each endpoint to another variables for easuer implementation
                    glm::vec3 colorZero = parallelogramlight.color0;
                    glm::vec3 colorOne = parallelogramlight.color1;
                    glm::vec3 colorTwo = parallelogramlight.color2;
                    glm::vec3 colorThree = parallelogramlight.color3;


                    //Calculate the steps in the 'x' and 'y' axis of the parallelogram 
                    // we will add the jittering in the next part, right now we assume equal distances
                    glm::vec3 x_step = (vertexOne - vertexZero) / (float)sampleSize;
                    glm::vec3 y_step = (vertexTwo - vertexZero) / (float)sampleSize;

                    // bilinear interpolation
                    // f(0,0)(1-x)(1-y) + f(0,1)(1-x)y + f(1,0) x(1-y) + f(1,1)xy
                    for (int i = 0; i < sampleSize; i++) {
                        for (int j = 0; j < sampleSize; j++) {

                            // random sampling by adding jittering for each sample
                            float e = std::rand() % 100 / (float)100;
                            float new_alpha_x = (float)(i + e) * alpha;
                            float new_alpha_y = (float)(j + e) * alpha;

                            // https://blogs.sas.com/content/iml/2020/05/18/what-is-bilinear-interpolation.html#:~:text=Bilinear%20interpolation%20is%20a%20weighted,four%20corners%20of%20the%20rectangle.&text=Given%20a%20rectangle%20with%20lower,y0)%2F(y1%2Dy0)
                            //using bileaner interpolation and our new data we calclulate the new values for the colour of the sample light
                            glm::vec3 currColor{ 0.f };
                            currColor += (colorZero * (1 - new_alpha_x) * (1 - new_alpha_y));
                            currColor += (colorTwo * (1 - new_alpha_x) * (new_alpha_y));
                            currColor += (colorOne * (new_alpha_x) * (1 - new_alpha_y));
                            currColor += (colorThree * (new_alpha_x) * (new_alpha_y));

                            // before we averrage, we save the color to show in debug ray
                            glm::vec3 debugRayColor = currColor;

                            //We average out all the samples
                            currColor *= (alpha * alpha);

                            //Calculating the new initial position of the sample
                            glm::vec3 currPos = vertexZero + (((float)i + e) * x_step + ((float)j + e) * y_step);
                            //And then create a point light source and we apply once more the same functions we implemented during the point light
                            PointLight currPointLight = { currPos, currColor };

                            //Again using hard shadows for each of the point light source -> soft shadows
                            if (hitLightSuccess(bvh, ray, currPointLight.position)) {
                                finalColor += currPointLight.color * calculateDiffuse(currPointLight, hitInfo, ray);
                                finalColor += currPointLight.color * calculateSpecular(currPointLight, hitInfo, ray);


                                // debug ray: parallelogram lights
                                // drawing all the sampled rays that hit the ligth with their color
                                if (debugAreaLights) {
                                    Ray tempLightRay;
                                    tempLightRay.origin = currPos;
                                    glm::vec3 intersectionPointRay = ray.origin + ray.t * ray.direction;
                                    tempLightRay.direction = glm::normalize(intersectionPointRay - tempLightRay.origin);
                                    HitInfo hitInfo;
                                    bvh.intersect(tempLightRay, hitInfo);
                                    drawRay(tempLightRay, debugRayColor);
                                }
                            }
                            else {
                                // if the rays are not hitting the light, we make them red
                                glm::vec3 intersectionPointRay = ray.origin + ray.t * ray.direction;

                                if (debugAreaLights) {
                                    Ray shadowRay;
                                    shadowRay.origin = intersectionPointRay;
                                    shadowRay.direction = -(intersectionPointRay - currPos);
                                    HitInfo shadowRayInfo;
                                    bvh.intersect(shadowRay, shadowRayInfo);
                                    drawRay(shadowRay, glm::vec3(1.0f, 0.0f, 0.0f));
                                }
                            }
                        }
                    }
                }
            }

            // drawing the camera ray using the final color
            drawRay(ray, finalColor);


            // everytime the ray intersects a specular surface, trace another ray in the mirror-reflection direction
            float epsilon = (float)1E-6;
            if (glm::length(hitInfo.material.ks) > epsilon && level < maxLevel) {

                glm::vec3 hitNormal = glm::normalize(hitInfo.normal);
                glm::vec3 reflectedVector = 2 * glm::dot(-ray.direction, hitNormal) * hitNormal + ray.direction;
                reflectedVector = glm::normalize(reflectedVector);

                glm::vec3 intersectionPoint = ray.origin + ray.direction * ray.t;
                Ray reflectedRay = { intersectionPoint,  reflectedVector };

                HitInfo hitInfo1;
                finalColor += hitInfo.material.ks * (recursive_ray_tracer(scene, bvh, reflectedRay, level + 1, maxLevel, hitInfo1));
            }
            if (debugTransparency) {
                float transparency = hitInfo.material.transparency;
                if (transparency < 1) {
                    float background = 1 - transparency;
                    glm::vec3 intersectionPoint = ray.origin + ray.direction * ray.t;
                    Ray secondRay = { intersectionPoint, ray.direction };

                    HitInfo hitInfo1;
                    glm::vec3 backgroundColor = recursive_ray_tracer(scene, bvh, secondRay, level + 1, maxLevel, hitInfo1);
                    glm::vec3 oldFinalColor = finalColor;
                    finalColor = transparency * oldFinalColor + background * backgroundColor;
                }
            }
        }
        return finalColor;
    }
    else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));

        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}

/// <summary>
/// This funcition is called in order to calculate the color of each pixel in our scene. 
/// By using the function above recursively we can calculate the final colour
/// </summary>
/// <param name="scene"></param>
/// <param name="bvh"></param>
/// <param name="ray"></param>
/// <returns></returns>
static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray)
{
    int startLevel = 0;
    int maxLevel = 6;
    HitInfo hitInfo;
    glm::vec3 finalColour = recursive_ray_tracer(scene, bvh, ray, startLevel, maxLevel, hitInfo);
    if (debugIntersectionAABB) {
        drawDebugTriangle(hitInfo);
    }
    return finalColour;
}

// box Filter which averages the pixel based on its neighbouring pixels
glm::vec3 boxFilter(Screen& source, int i, int j, int filterSize) {
    // filterSize cannot be smaller than 1
    filterSize = glm::max(1, filterSize);
    glm::vec3 sum{ 0.0f };

    // sum and average the surrounding pixels based on the filterSize
    for (int x = -filterSize; x < filterSize + 1; ++x) {
        for (int y = -filterSize; y < filterSize + 1; ++y) {
            sum += source.getPixel(i + x, j + y);
        }
    }
    sum /= (2 * filterSize + 1) * (2 * filterSize + 1);
    
    // return new averaged pixel
    return sum;
}

glm::vec3 mipmapping(Screen& source, int i, int j, int filterSize) {
    // filterSize cannot be smaller than 1
    filterSize = glm::max(1, filterSize);
    glm::vec3 sum{ 0.0f };

    // sum and average the surrounding pixels based on the filterSize
    for (int x = 0; x < filterSize; ++x) {
        for (int y = 0; y < filterSize; ++y) {
            sum += source.getPixel(i + x, j + y);
        }
    }
    sum /= filterSize * filterSize;

    // return new averaged pixel
    return sum;
}

// filter that goes through all the pixels in the screen, based on the filterSize
Screen GeneralFilter(Screen& source, int filterSize) {
    if (debugBloomFilter) {
        Screen result(windowResolution);
        for (int i = filterSize; i < windowResolution.x - filterSize; ++i) {
            for (int j = filterSize; j < windowResolution.y - filterSize; ++j) {
                // call boxFilter for new colour of the pixel
                result.setPixel(i, j, boxFilter(source, i, j, filterSize));
            }
        }
        return result;
    }
    if (debugMipmapping) {
        // the window size will be smaller when using mipmapping
        // this is only possible if the current windowResolution is big enough
        if (source.getResolution().x > filterSize && source.getResolution().y > filterSize) {
            Screen result(source.getResolution() / filterSize);
            for (int i = 0; i < source.getResolution().x - filterSize; i = i + filterSize) {
                for (int j = 0; j < source.getResolution().y - filterSize; j = j + filterSize) {
                    // call mipmapping for new colour of the pixel
                    result.setPixel(i / filterSize, j / filterSize, mipmapping(source, i, j, filterSize));
                }
            }
            return result;
        }
        else {
            return source;
        }        
    }
}


static void setOpenGLMatrices(const Trackball& camera);
static void drawLightsOpenGL(const Scene& scene, const Trackball& camera, int selectedLight);
static void drawSceneOpenGL(const Scene& scene);

// This is the main rendering function. You are free to change this function in any way (including the function signature).
static Screen renderRayTracing(const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen)
{
#ifndef NDEBUG
    // Single threaded in debug mode
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos {
                float(x) / windowResolution.x * 2.0f - 1.0f,
                float(y) / windowResolution.y * 2.0f - 1.0f
            };
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay));
        }
    }
#else
    // Multi-threaded in release mode

    // create new screen to store the thresholded values
    Screen threshold{ windowResolution };

    const tbb::blocked_range2d<int, int> windowRange { 0, windowResolution.y, 0, windowResolution.x };
    tbb::parallel_for(windowRange, [&](tbb::blocked_range2d<int, int> localRange) {
        for (int y = std::begin(localRange.rows()); y != std::end(localRange.rows()); y++) {
            for (int x = std::begin(localRange.cols()); x != std::end(localRange.cols()); x++) {
                // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
                const glm::vec2 normalizedPixelPos {
                    float(x) / windowResolution.x * 2.0f - 1.0f,
                    float(y) / windowResolution.y * 2.0f - 1.0f
                };
                const Ray cameraRay = camera.generateRay(normalizedPixelPos);
                screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay));

                if (debugBloomFilter) {
                    glm::vec3 colour = screen.getPixel(x, y);
                    // add the pixel to the threshold if it has a certain value
                    if (colour.x > 0.5f || colour.y > 0.5f || colour.z > 0.5f) {
                        threshold.setPixel(x, y, colour);
                    }
                    else {
                        threshold.setPixel(x, y, glm::vec3{ 0.0f });
                    }
                }
            }
        }
    });
    if (debugBloomFilter) {
        // box filter
        threshold = GeneralFilter(threshold, 5);

        // add the new pixels to the original
        tbb::parallel_for(windowRange, [&](tbb::blocked_range2d<int, int> localRange) {
            for (int y = std::begin(localRange.rows()); y != std::end(localRange.rows()); y++) {
                for (int x = std::begin(localRange.cols()); x != std::end(localRange.cols()); x++) {
                    // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
                    const glm::vec2 normalizedPixelPos{
                        float(x) / windowResolution.x * 2.0f - 1.0f,
                        float(y) / windowResolution.y * 2.0f - 1.0f
                    };
                    glm::vec3 newColour = screen.getPixel(x, y) + 1.0f * threshold.getPixel(x, y);
                    screen.setPixel(x, y, newColour);
                }
            }
        });
    }
    if (debugMipmapping) {
        Screen mipmap = screen;
        int level = mipmappingLevel;
        int reduceResolution = mipmappingReduceResolution;
        for (int i = 0; i < level; i++) {
            mipmap = GeneralFilter(mipmap, reduceResolution);
        }
        return mipmap;

    }
    // TODO
    // slider for threshold
    // slider for filterSize
    return screen;
#endif
}

int main(int argc, char** argv)
{
    Trackball::printHelp();
    std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
              << std::endl;

    Window window { "Final Project", windowResolution, OpenGLVersion::GL2 };
    Screen screen { windowResolution };
    Trackball camera { &window, glm::radians(50.0f), 3.0f };
    camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

    SceneType sceneType { SceneType::SingleTriangle };
    std::optional<Ray> optDebugRay;
    Scene scene = loadScene(sceneType, dataPath);
    BoundingVolumeHierarchy bvh { &scene };

    // debug buttons gui
    int bvhDebugLevel = 0;
    bool debugBVH { false };

    ViewMode viewMode { ViewMode::Rasterization };

    window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
        if (action == GLFW_PRESS) {
            switch (key) {
            case GLFW_KEY_R: {
                // Shoot a ray. Produce a ray from camera to the far plane.
                const auto tmp = window.getNormalizedCursorPos();
                optDebugRay = camera.generateRay(tmp * 2.0f - 1.0f);
            } break;
            case GLFW_KEY_ESCAPE: {
                window.close();
            } break;
            };
        }
    });

    int selectedLightIdx = scene.lights.empty() ? -1 : 0;
    while (!window.shouldClose()) {
        window.updateInput();

        // === Setup the UI ===
        ImGui::Begin("Final Project");
        {
            constexpr std::array items { "SingleTriangle", "Cube (segment light)", "Cornell Box (with mirror)", 
                "Cornell Box (parallelogram light and mirror)", "Monkey", "Teapot", "Dragon", /* "AABBs",*/ "Spheres", /*"Mixed",*/ "Custom", "Dragon2"};
            if (ImGui::Combo("Scenes", reinterpret_cast<int*>(&sceneType), items.data(), int(items.size()))) {
                optDebugRay.reset();
                scene = loadScene(sceneType, dataPath);
                selectedLightIdx = scene.lights.empty() ? -1 : 0;
                bvh = BoundingVolumeHierarchy(&scene);
                if (optDebugRay) {
                    HitInfo dummy {};
                    bvh.intersect(*optDebugRay, dummy);
                }
            }
        }
        {
            constexpr std::array items { "Rasterization", "Ray Traced" };
            ImGui::Combo("View mode", reinterpret_cast<int*>(&viewMode), items.data(), int(items.size()));
        }
        if (ImGui::Button("Render to file")) {
            // Show a file picker.
            nfdchar_t* pOutPath = nullptr;
            const nfdresult_t result = NFD_SaveDialog("bmp", nullptr, &pOutPath);
            if (result == NFD_OKAY) {
                std::filesystem::path outPath { pOutPath };
                free(pOutPath); // NFD is a C API so we have to manually free the memory it allocated.
                outPath.replace_extension("bmp"); // Make sure that the file extension is *.bmp

                // Perform a new render and measure the time it took to generate the image.
                using clock = std::chrono::high_resolution_clock;
                const auto start = clock::now();
                std::cout << "Rendering in progress..." << std::endl;
                Screen output = renderRayTracing(scene, camera, bvh, screen);
                const auto end = clock::now();
                std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;


                // Store the new image.
                output.writeBitmapToFile(outPath);
            }
        }
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Shadow Debug Ray", &debugShadowRay);
            ImGui::Checkbox("Draw Area Lights", &debugAreaLights);
            ImGui::Checkbox("Draw BVH", &debugBVH);

            if (debugBVH) {
                ImGui::SliderInt("BVH Level", &bvhDebugLevel, 0, bvh.numLevels() - 1);
                ImGui::Checkbox("Draw triangles inside leaf nodes", &drawTrianglesInLeaf);
            }

            ImGui::Checkbox("Draw Intersected But Not Visited Modes", &debugIntersectionAABB);
            ImGui::Checkbox("Draw Interpolated Normals", &debugNormalInterpolation);
            ImGui::Checkbox("Add Texture", &debugTextures);
            ImGui::Checkbox("Mipmapping", &debugMipmapping);
            if (debugMipmapping) {
                ImGui::SliderInt("Level", &mipmappingLevel, 0, 10);
                ImGui::SliderInt("Reduce resolution by X times X", &mipmappingReduceResolution, 2, 10);
            }
            ImGui::Checkbox("Bloom Filter", &debugBloomFilter);
            ImGui::Checkbox("Transparency", &debugTransparency);

        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Lights");
        {
            std::vector<std::string> options;
            options.push_back("None");
            for (size_t i = 0; i < scene.lights.size(); i++) {
                options.push_back("Light " + std::to_string(i));
            }
            std::vector<const char*> optionsPointers;
            std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers), [](const auto& str) { return str.c_str(); });

            // Offset such that selectedLightIdx=-1 becomes item 0 (None).
            ++selectedLightIdx;
            ImGui::Combo("Selected light", &selectedLightIdx, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
            --selectedLightIdx;

            if (selectedLightIdx >= 0) {
                setOpenGLMatrices(camera);
                std::visit(
                    make_visitor(
                        [&](PointLight& light) {
                            showImGuizmoTranslation(window, camera, light.position); // 3D controls to translate light source.
                            ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                            ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                        },
                        [&](SegmentLight& light) {
                            static int selectedEndpoint = 0;
                            // 3D controls to translate light source.
                            if (selectedEndpoint == 0)
                                showImGuizmoTranslation(window, camera, light.endpoint0);
                            else
                                showImGuizmoTranslation(window, camera, light.endpoint1);

                            const std::array<const char*, 2> endpointOptions { "Endpoint 0", "Endpoint 1" };
                            ImGui::Combo("Selected endpoint", &selectedEndpoint, endpointOptions.data(), (int)endpointOptions.size());
                            ImGui::DragFloat3("Endpoint 0", glm::value_ptr(light.endpoint0), 0.01f, -3.0f, 3.0f);
                            ImGui::DragFloat3("Endpoint 1", glm::value_ptr(light.endpoint1), 0.01f, -3.0f, 3.0f);
                            ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                            ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                        },
                        [&](ParallelogramLight& light) {
                            glm::vec3 vertex1 = light.v0 + light.edge01;
                            glm::vec3 vertex2 = light.v0 + light.edge02;

                            static int selectedVertex = 0;
                            // 3D controls to translate light source.
                            if (selectedVertex == 0)
                                showImGuizmoTranslation(window, camera, light.v0);
                            else if (selectedVertex == 1)
                                showImGuizmoTranslation(window, camera, vertex1);
                            else
                                showImGuizmoTranslation(window, camera, vertex2);

                            const std::array<const char*, 3> vertexOptions { "Vertex 0", "Vertex 1", "Vertex 2" };
                            ImGui::Combo("Selected vertex", &selectedVertex, vertexOptions.data(), (int)vertexOptions.size());
                            ImGui::DragFloat3("Vertex 0", glm::value_ptr(light.v0), 0.01f, -3.0f, 3.0f);
                            ImGui::DragFloat3("Vertex 1", glm::value_ptr(vertex1), 0.01f, -3.0f, 3.0f);
                            light.edge01 = vertex1 - light.v0;
                            ImGui::DragFloat3("Vertex 2", glm::value_ptr(vertex2), 0.01f, -3.0f, 3.0f);
                            light.edge02 = vertex2 - light.v0;

                            ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                            ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                            ImGui::ColorEdit3("Color 2", glm::value_ptr(light.color2));
                            ImGui::ColorEdit3("Color 3", glm::value_ptr(light.color3));
                        },
                        [](auto) { /* any other type of light */ }),
                    scene.lights[selectedLightIdx]);
            }
        }

        if (ImGui::Button("Add point light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(PointLight { .position = glm::vec3(0.0f), .color = glm::vec3(1.0f) });
        }
        if (ImGui::Button("Add segment light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(SegmentLight { .endpoint0 = glm::vec3(0.0f), .endpoint1 = glm::vec3(1.0f), .color0 = glm::vec3(1, 0, 0), .color1 = glm::vec3(0, 0, 1) });
        }
        if (ImGui::Button("Add parallelogram light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(ParallelogramLight {
                .v0 = glm::vec3(0.0f),
                .edge01 = glm::vec3(1, 0, 0),
                .edge02 = glm::vec3(0, 1, 0),
                .color0 = glm::vec3(1, 0, 0), // red
                .color1 = glm::vec3(0, 1, 0), // green
                .color2 = glm::vec3(0, 0, 1), // blue
                .color3 = glm::vec3(1, 1, 1) // white
            });
        }
        if (selectedLightIdx >= 0 && ImGui::Button("Remove selected light")) {
            scene.lights.erase(std::begin(scene.lights) + selectedLightIdx);
            selectedLightIdx = -1;
        }

        // Clear screen.
        glViewport(0, 0, window.getFrameBufferSize().x, window.getFrameBufferSize().y);
        glClearDepth(1.0f);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        setOpenGLMatrices(camera);

        // Draw either using OpenGL (rasterization) or the ray tracing function.
        switch (viewMode) {
        case ViewMode::Rasterization: {
            glPushAttrib(GL_ALL_ATTRIB_BITS);

            if (debugIntersectionAABB == false && drawTrianglesInLeaf == false) {
                drawSceneOpenGL(scene);
            }

            if (optDebugRay) {
                // Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
                // draw the rays instead.
                enableDrawRay = true;
                (void)getFinalColor(scene, bvh, *optDebugRay);
                enableDrawRay = false;
            }
            glPopAttrib();
        } break;

        case ViewMode::RayTracing: {
            screen.clear(glm::vec3(0.0f));
            renderRayTracing(scene, camera, bvh, screen);
            screen.setPixel(0, 0, glm::vec3(1.0f));
            screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
        } break;
        default:
            break;
        };

        drawLightsOpenGL(scene, camera, selectedLightIdx);

        if (debugBVH) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            enableDrawRay = true;
            bvh.debugDraw(bvhDebugLevel);
            enableDrawRay = false;
            glPopAttrib();
        }

        ImGui::End();
        window.swapBuffers();
    }

    return 0;
}

static void setOpenGLMatrices(const Trackball& camera)
{
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void drawLightsOpenGL(const Scene& scene, const Trackball& camera, int selectedLight)
{
    // Normals will be normalized in the graphics pipeline.
    glEnable(GL_NORMALIZE);
    // Activate rendering modes.
    glEnable(GL_DEPTH_TEST);
    // Draw front and back facing triangles filled.
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    // Interpolate vertex colors over the triangles.
    glShadeModel(GL_SMOOTH);

    glDisable(GL_LIGHTING);
    // Draw all non-selected lights.
    for (size_t i = 0; i < scene.lights.size(); i++) {
        std::visit(
            make_visitor(
                [](const PointLight& light) { drawSphere(light.position, 0.01f, light.color); },
                [](const SegmentLight& light) {
                    glPushAttrib(GL_ALL_ATTRIB_BITS);
                    glBegin(GL_LINES);
                    glColor3fv(glm::value_ptr(light.color0));
                    glVertex3fv(glm::value_ptr(light.endpoint0));
                    glColor3fv(glm::value_ptr(light.color1));
                    glVertex3fv(glm::value_ptr(light.endpoint1));
                    glEnd();
                    glPopAttrib();
                    drawSphere(light.endpoint0, 0.01f, light.color0);
                    drawSphere(light.endpoint1, 0.01f, light.color1);
                },
                [](const ParallelogramLight& light) {
                    glPushAttrib(GL_ALL_ATTRIB_BITS);
                    glBegin(GL_QUADS);
                    glColor3fv(glm::value_ptr(light.color0));
                    glVertex3fv(glm::value_ptr(light.v0));
                    glColor3fv(glm::value_ptr(light.color1));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge01));
                    glColor3fv(glm::value_ptr(light.color3));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge01 + light.edge02));
                    glColor3fv(glm::value_ptr(light.color2));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge02));
                    glEnd();
                    glPopAttrib();
                },
                [](auto) { /* any other type of light */ }),
            scene.lights[i]);
    }

    // Draw a colored sphere at the location at which the trackball is looking/rotating around.
    glDisable(GL_LIGHTING);
    drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}

void drawSceneOpenGL(const Scene& scene)
{
    // Activate the light in the legacy OpenGL mode.
    glEnable(GL_LIGHTING);

    // Tell OpenGL where the lights are (so it nows how to shade surfaces in the scene).
    // This is only used in the rasterization view. OpenGL only supports point lights so
    // we replace segment/parallelogram lights by point lights.
    int i = 0;
    const auto enableLight = [&](const glm::vec3& position, const glm::vec3 color) {
        glEnable(GL_LIGHT0 + i);
        const glm::vec4 position4 { position, 1 };
        glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
        const glm::vec4 color4 { glm::clamp(color, 0.0f, 1.0f), 1.0f };
        const glm::vec4 zero4 { 0.0f, 0.0f, 0.0f, 1.0f };
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
        // NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
        // The distance is not in world space but in NDC space!
        glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
        i++;
    };
    for (const auto& light : scene.lights) {
        std::visit(
            make_visitor(
                [&](const PointLight& light) {
                    enableLight(light.position, light.color);
                },
                [&](const SegmentLight& light) {
                    // Approximate with two point lights: one at each endpoint.
                    enableLight(light.endpoint0, 0.5f * light.color0);
                    enableLight(light.endpoint1, 0.5f * light.color1);
                },
                [&](const ParallelogramLight& light) {
                    enableLight(light.v0, 0.25f * light.color0);
                    enableLight(light.v0 + light.edge01, 0.25f * light.color1);
                    enableLight(light.v0 + light.edge02, 0.25f * light.color2);
                    enableLight(light.v0 + light.edge01 + light.edge02, 0.25f * light.color3);
                },
                [](auto) { /* any other type of light */ }),
            light);
    }

    // Draw the scene and the ray (if any).
    drawScene(scene);
}
