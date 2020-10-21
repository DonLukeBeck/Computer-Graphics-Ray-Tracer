#include "bounding_volume_hierarchy.h"
#include "draw.h"

AxisAlignedBox aabb;

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    float xmin = FLT_MAX;
    float ymin = FLT_MAX;
    float zmin = FLT_MAX;
   
    float xmax = FLT_MIN;
    float ymax = FLT_MIN;
    float zmax = FLT_MIN;
    
    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            
            xmin = fmin(xmin, fmin(v0.p.x, fmin(v1.p.x, v2.p.x)));
            ymin = fmin(ymin, fmin(v0.p.y, fmin(v1.p.y, v2.p.y)));
            zmin = fmin(zmin, fmin(v0.p.z, fmin(v1.p.z, v2.p.z)));
            
            xmax = fmax(xmax, fmax(v0.p.x, fmax(v1.p.x, v2.p.x)));
            ymax = fmax(ymax, fmax(v0.p.y, fmax(v1.p.y, v2.p.y)));
            zmax = fmax(zmax, fmax(v0.p.z, fmax(v1.p.z, v2.p.z)));
        }
    }
    aabb = { glm::vec3(xmin,ymin,zmin),
        glm::vec3(xmax, ymax,zmax) };
    // as an example of how to iterate over all meshes in the scene, look at the intersect method below
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
    //AxisAlignedBox aabb { glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    drawAABB(aabb, DrawMode::Wireframe);
    //drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
}

int BoundingVolumeHierarchy::numLevels() const
{
    return 5;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    bool hit = false;
    float t = ray.t;
    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo)) {
                if (t > ray.t) {
                    t = ray.t;
                    hitInfo.material = mesh.material;
                }
                hit = true;
            }
            if (t < ray.t) {
                ray.t = t;
            }
        }
    }
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}
