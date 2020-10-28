#pragma once
#include "ray_tracing.h"
#include "scene.h"

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    int numLevels() const;

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo, int level = 1) const;

    void reflect(Ray &ray, HitInfo &hitInfo) const;

    struct Node {
        bool leaf;
        Node* left;
        Node* right;
        std::vector<glm::vec3> indices;
    };

private:
    Scene* m_pScene;
};
