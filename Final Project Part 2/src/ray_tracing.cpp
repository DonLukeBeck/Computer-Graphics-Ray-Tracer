#include "ray_tracing.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>
    
bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
    if (glm::dot(n, glm::cross(v1 - v0, p - v0)) > 0
        && glm::dot(n, glm::cross(v2 - v1, p - v1)) > 0 
        && glm::dot(n, glm::cross(v0 - v2, p - v2)) > 0)
        return true;
    return false;
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    float d = glm::dot(glm::normalize(ray.direction), plane.normal);
    if (d != 0) {
        float t = ((plane.D - glm::dot(ray.origin, plane.normal)) / glm::dot(ray.direction, plane.normal));
        if (t > 0.0001f && t < ray.t) {
            ray.t = t;
            return true;
        }
    }
    return false;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    plane.normal = glm::normalize(glm::cross((v1 - v0), (v2 - v0)));
    plane.D = glm::dot(plane.normal, v2);
    
    return plane;
}


/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    float tCopy = ray.t;
    Plane plane = trianglePlane(v0, v1, v2);
    if (intersectRayWithPlane(plane, ray) && pointInTriangle(v0, v1, v2, plane.normal, ray.origin + ray.t * ray.direction)) {
        hitInfo.normal = plane.normal;
        hitInfo.intersection = ray.origin + ray.t * ray.direction;
        return true;
    }
    ray.t = tCopy;
    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    glm::vec3 origin = ray.origin - sphere.center;
    float A = ray.direction[0] * ray.direction[0] + ray.direction[1] * ray.direction[1] + ray.direction[2] * ray.direction[2];
    float B = 2 * (ray.direction[0] * origin[0] + ray.direction[1] * origin[1] + ray.direction[2] * origin[2]);
    float C = origin[0] * origin[0] + origin[1] * origin[1] + origin[2] * origin[2] - sphere.radius * sphere.radius;
    float discriminant = B * B - 4 * A * C;
    if (discriminant < 0)
        return false;
    float t0 = (-B - glm::sqrt(discriminant)) / 2 * A;
    float t1 = (-B + glm::sqrt(discriminant)) / 2 * A;
    if (t0 < 0)
        if (t1 < 0 || t1 >= ray.t)
            return false;
        else {
            ray.t = t1;
            return true;
        }
    if (t0 >= ray.t)
        return false;
    ray.t = t0;
    hitInfo.material = sphere.material;
    hitInfo.intersection = ray.origin + ray.t * ray.direction;
    return true;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    glm::vec3 tmin = (box.lower - ray.origin) / ray.direction;
    glm::vec3 tmax = (box.upper - ray.origin) / ray.direction;

    float tinx = glm::min(tmin[0], tmax[0]);
    float toutx = glm::max(tmin[0], tmax[0]);
    float tiny = glm::min(tmin[1], tmax[1]);
    float touty = glm::max(tmin[1], tmax[1]);
    float tinz = glm::min(tmin[2], tmax[2]);
    float toutz = glm::max(tmin[2], tmax[2]);

    float tin = glm::max(glm::max(tinx, tiny), tinz);
    float tout = glm::min(glm::min(toutx, touty), toutz);

    if (tin > tout || tout < 0 || tin >= ray.t)
        return false;
    ray.t = tin;
    return true;

}
