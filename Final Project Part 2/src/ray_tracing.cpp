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
    float area0 = glm::length(glm::cross(v1 - v0, v2 - v0)) / 2.0f;
    float a = glm::length(glm::cross(v0 - p, v1 - p)) / 2.0f / area0;
    float b = glm::length(glm::cross(v1 - p, v2 - p)) / 2.0f / area0;
    float g = glm::length(glm::cross(v2 - p, v0 - p)) / 2.0f / area0;
    if (area0 <= 0.0f) {
        return false;
    }

    if (a < 0.0f || b < 0.0f || g < 0.0f || a + b + g > 1.00001f || a + b + g < 0.99999f)
        return false;

    return true;
}


bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    float t = ray.t;

    if (glm::dot(ray.direction, plane.normal) == 0) {
        return false;
    }

    if (fabs(glm::dot(ray.direction, plane.normal)) >= 0.00001f) {
        t = glm::dot(plane.D * plane.normal - ray.origin, plane.normal) / glm::dot(ray.direction, plane.normal);

        if (t < ray.t && t > 0.00001f) {
            ray.t = t;
            return true;
        }
    }

    return false;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    plane.normal = glm::normalize(glm::cross((v0 - v2), (v1 - v2)));
    plane.D = fabs(glm::dot(plane.normal, v0));

    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{

    Plane triangleP = trianglePlane(v0, v1, v2);
    float t = ray.t;

    if (intersectRayWithPlane(triangleP, ray)) {
        glm::vec3 p = ray.origin + ray.t * ray.direction;

        if (pointInTriangle(v0, v1, v2, triangleP.normal, p)) {

            if (ray.t < t) {
                hitInfo.normal = triangleP.normal;
                hitInfo.intersection = ray.origin + ray.t * ray.direction;
                return true;
            }
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

    float t = ray.t;
    glm::vec3 oc = ray.origin - sphere.center;
    float a = glm::dot(ray.direction, ray.direction);
    float b = 2.0 * glm::dot(oc, ray.direction);
    float c = dot(oc, oc) - sphere.radius * sphere.radius;
    float delta = b * b - 4 * a * c;

    if (delta < 0.0) {
        return false;
    }
    else {
        float numerator = -b - sqrt(delta);

        if (numerator > 0.0) {
            ray.t = numerator / (2.0 * a);

            if (t < ray.t) {
                ray.t = t;
                return false;
            }
            hitInfo.material = sphere.material;
            hitInfo.intersection = ray.origin + ray.t * ray.direction;
            return true;
        }

        numerator = -b + sqrt(delta);
        if (numerator > 0.0) {
            ray.t = numerator / (2.0 * a);

            if (t < ray.t) {
                ray.t = t;
                return false;
            }
            hitInfo.material = sphere.material;
            hitInfo.intersection = ray.origin + ray.t * ray.direction;
            return true;
        }
    }

    return false;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    float txmin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float txmax = (box.upper.x - ray.origin.x) / ray.direction.x;

    float tymin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tymax = (box.upper.y - ray.origin.y) / ray.direction.y;

    float tzmin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float tzmax = (box.upper.z - ray.origin.z) / ray.direction.z;

    float tinx = fmin(txmin, txmax);
    float toutx = fmax(txmin, txmax);

    float tiny = fmin(tymin, tymax);
    float touty = fmax(tymin, tymax);

    float tinz = fmin(tzmin, tzmax);
    float toutz = fmax(tzmin, tzmax);

    float tin = fmax(fmax(tinx, tiny), tinz);
    float tout = fmin(fmin(toutx, touty), toutz);

    float t = ray.t;
    if (tin > tout || tout < 0) {
        return false;
    }
    else {

        if ((ray.origin.x > tin) && (ray.origin.y > tin) && (ray.origin.z > tin) && (ray.origin.x < tout || ray.origin.y < tout || ray.origin.z < tout)) {
            ray.t = tout;
        }
        else {
            ray.t = tin;
        }

        if (t < ray.t) {
            ray.t = t;
            return false;
        }

        return true;
    }

}
