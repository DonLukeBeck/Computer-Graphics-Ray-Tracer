#pragma once
#include "ray_tracing.h"
#include "scene.h"


class BoundingVolumeHierarchy {
public:
    AxisAlignedBox createBV(std::vector<std::pair<int, int>> triangles);
    void split(AxisAlignedBox& parentBox, std::vector<std::pair<int, int>>& parentTriangles,
        std::vector<std::pair<int, int>>& leftTriangles, std::vector<std::pair<int, int>>& rightTriangles, int axis);
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
        bool leaf = false;
        int left;
        int right;
        std::vector<glm::vec3> indices;
        std::vector<Mesh> meshes;
        std::vector<std::pair<int, int>> triangles;
        AxisAlignedBox bv;
    };

    //Node root;

    std::vector<Node> nodes = {};


private:
    Scene* m_pScene;
};
