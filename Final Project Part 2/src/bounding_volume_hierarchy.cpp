#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm\geometric.hpp>
#include<iostream>

AxisAlignedBox aabb;
int levels = 0;

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    float xmin = FLT_MAX;
    float ymin = FLT_MAX;
    float zmin = FLT_MAX;
   
    float xmax = FLT_MIN;
    float ymax = FLT_MIN;
    float zmax = FLT_MIN;
    
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

    Node root, left, right;

    root.leaf = false;
    root.indices = { glm::vec3(xmin,ymin,zmin),
        glm::vec3(xmax, ymax,zmax) };
    root.meshes = m_pScene->meshes;
    int lvl = 0;

    this->nodes.push_back(root);
    int i = 0;
    while (i < 32) {
  
        std::vector<glm::vec3> indices = this->nodes[i].indices;
        Node left, right;
        if (lvl % 2 == 0) {
            left.indices = { indices[0], glm::vec3((indices[0][0] + indices[1][0]) / 2, indices[1][1], indices[1][2]) };
            right.indices = { glm::vec3((indices[0][0] + indices[1][0]) / 2, indices[0][1], indices[0][2]), indices[1] };

            for (const auto& mesh : this->nodes[i].meshes) {
                float xmin = FLT_MAX;
                float ymin = FLT_MAX;
                float zmin = FLT_MAX;

                float xmax = FLT_MIN;
                float ymax = FLT_MIN;
                float zmax = FLT_MIN;
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
                if ((xmin + xmax) / 2 <= (indices[0][0] + indices[1][0]) / 2)
                    left.meshes.push_back(mesh);
                else
                    right.meshes.push_back(mesh);
            }
        }
        else {
            left.indices = { indices[0], glm::vec3(indices[1][0], indices[1][1], (indices[0][2] + indices[1][2]) / 2 )};
            right.indices = { glm::vec3(indices[0][0], indices[1][1], (indices[0][2] + indices[1][2]) / 2), indices[1] };

            for (const auto& mesh : this->nodes[i].meshes) {
                float xmin = FLT_MAX;
                float ymin = FLT_MAX;
                float zmin = FLT_MAX;

                float xmax = FLT_MIN;
                float ymax = FLT_MIN;
                float zmax = FLT_MIN;
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
                if ((zmin + zmax) / 2 <= (indices[0][2] + indices[1][2]) / 2)
                    left.meshes.push_back(mesh);
                else
                    right.meshes.push_back(mesh);
            }
        }
        if (!left.meshes.empty())
            left.leaf = false;
        if (!right.meshes.empty())
            right.leaf = false;
        if(i == glm::pow(2, lvl + 1) - 2)
            lvl++;
        this->nodes.push_back(left);
        this->nodes.push_back(right);
        i++;
    }
    levels = lvl;
    
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
    //drawAABB(aabb, DrawMode::Wireframe);
    if(level<this->numLevels())
        for (int i = glm::pow(2, level)-1 ; i < glm::pow(2, (level+1))-1; i++) {
            if (!this->nodes[i].leaf) {
                AxisAlignedBox box = { this->nodes[i].indices[0],this->nodes[i].indices[1] };
                drawAABB(box, DrawMode::Wireframe);
            }
        }
    //AxisAlignedBox box = { this->nodes[0].indices[0],this->nodes[0].indices[1]};
    //drawAABB(box, DrawMode::Wireframe);
    //drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
}

int BoundingVolumeHierarchy::numLevels() const
{
    return levels;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, int level) const
{
    bool hit = false;
    //float t = ray.t;
    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo)) {
                hitInfo.material = mesh.material;
                hit = true;
            }
        }
    }
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    
   if (hit == true && level < 5) {
        drawRay(ray,glm::vec3(1.0f));
        HitInfo newHitInfo;
        if (hitInfo.material.ks != glm::vec3(0)) {
            Ray newRay;
            newRay.origin = hitInfo.intersection;
            newRay.direction = glm::reflect(ray.direction, hitInfo.normal);
            hit = intersect(newRay, newHitInfo, level);
        }
    }

    return hit;
}

Ray reflection(Ray& ray, HitInfo& hitInfo)
{
    glm::vec3 direction = glm::normalize(glm::reflect(glm::normalize(ray.direction), glm::normalize(hitInfo.normal)));
    glm::vec3 intersection = ray.origin + ray.t * ray.direction;
    return Ray{ intersection, direction };
}

void BoundingVolumeHierarchy::reflect(Ray &ray, HitInfo &hitInfo) const
{
    drawRay(ray, glm::vec3(1.0f));
    glm::vec3 ks = hitInfo.material.ks;
    if(ks.x != 0 || ks.y != 0 || ks.z != 0){
        Ray reflected = reflection(ray, hitInfo);
        HitInfo newHitInfo;
        if (BoundingVolumeHierarchy::intersect(reflected, newHitInfo)) {
            reflect(reflected, newHitInfo);
            drawRay(reflected, glm::vec3(1.0f));
        }
    }
    
}


