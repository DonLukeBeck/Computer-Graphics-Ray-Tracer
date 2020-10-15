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
    float areaABC = dot(n, cross((v1 - v0), (v2 - v0)));
    float areaPBC = dot(n, cross((v1 - p), (v2 - p)));
    float areaPCA = dot(n, cross((v2 - p), (v0 - p)));
    float a = areaPBC / areaABC;
    float b = areaPCA / areaABC;
    if (a < 0.0f) {
        return false;
    }
    if (b < 0.0f) {
        return false;
    }
    if (a + b > 1.0f) {
        return false;
    }
    return true;
}


bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    float t = 0;
    if (!dot(ray.direction, plane.normal)) {
        t = ((plane.D - dot(ray.origin, plane.normal)) / dot(ray.direction, plane.normal));
    }
    if (dot((ray.origin + ray.direction * t), plane.normal) - plane.D == 0.0f) {
        if (t > ray.t) {
            return false;
        }
        ray.t = t;
        return true;
    }
    return false;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    plane.normal = (cross((v0 - v2), (v1 - v2)) / glm::length(cross((v0 - v2), (v1 - v2))));
    plane.D = dot(plane.normal, v2);
    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    float t = ray.t;
    Plane plane = trianglePlane(v0, v1, v2);
    if (intersectRayWithPlane(plane, ray)) {
        glm::vec3  p = ray.origin + (ray.direction * ray.t);
        if (pointInTriangle(v0, v1, v2, plane.normal, p)) {
            return true;
        }
    }
    ray.t = t;
    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    glm::vec3 o = ray.origin - sphere.center;
    glm::vec3 d = ray.direction;
    float r = sphere.radius;
    float A = pow(d.x, 2) + pow(d.y, 2) + pow(d.z, 2);
    float B = (d.x * o.x + d.y * o.y + d.z * o.z) * 2;
    float C = pow(o.x, 2) + pow(o.y, 2) + pow(o.z, 2) - pow(r, 2);
    float Discriminant = pow(B, 2) - (4 * A * C);
    if (Discriminant < 0.0f) {
        return false;
    }
    float t0 = (-B + sqrt(Discriminant)) / (2 * A);
    float t1 = (-B - sqrt(Discriminant)) / (2 * A);
    ray.t = glm::min(t0, t1);
    if (distance(ray.origin, sphere.center) < r) {
        ray.t = glm::max(t0, t1);
    }
    return true;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    float xmin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float xmax = (box.upper.x - ray.origin.x) / ray.direction.x;
    float xin = glm::min(xmin, xmax);
    float xout = glm::max(xmin, xmax);

    float ymin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float ymax = (box.upper.y - ray.origin.y) / ray.direction.y;
    float yin = glm::min(ymin, ymax);
    float yout = glm::max(ymin, ymax);

    float zmin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float zmax = (box.upper.z - ray.origin.z) / ray.direction.z;
    float zin = glm::min(zmin, zmax);
    float zout = glm::max(zmin, zmax);

    float tin = glm::max(xin, yin, zin);
    float tout = glm::min(xout, yout, zout);
    if (tin > tout || tout < 0) {
        return false;
    }
    ray.t = tin;
    if (tin <= 0.0f) {
        ray.t = tout;
    }
    return true;
}
