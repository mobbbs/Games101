#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,SplitMethod splitMethod) : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod), primitives(std::move(p)){
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuildNaive(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf("\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuildNaive(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i){
        bounds = Union(bounds, objects[i]->getBounds());
    }
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildNaive(std::vector{objects[0]});
        node->right = recursiveBuildNaive(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildNaive(leftshapes);
        node->right = recursiveBuildNaive(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects){
    // TODO implement SAH Build
    BVHBuildNode *node = new BVHBuildNode();

    constexpr int C_isect = 1;
    constexpr int C_trav = 1;

    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i){
        bounds = Union(bounds, objects[i]->getBounds());
    }
    node->bounds = bounds;
    int n = objects.size();
    float currentSufArea = bounds.SurfaceArea();
    float minCost = n * C_isect;
    int bestSplit = -1;
    int bestAxis = -1;
    for (int axis = 0; axis < 3; ++axis) {
        std::sort(objects.begin(), objects.end(), [axis](auto f1, auto f2) {
            return f1->getBounds().Centroid()[axis] < f2->getBounds().Centroid()[axis];
        });

        for (int i = 0; i < n - 1; ++i){
            Bounds3 leftBounds, rightBounds;
            for (int j = 0; j <= i; ++j){
                leftBounds = Union(leftBounds, objects[j]->getBounds());
            }
            for (int j = i + 1; j < n; ++j){
                rightBounds = Union(rightBounds, objects[j]->getBounds());
            }
            float cost = 2 * C_trav + (leftBounds.SurfaceArea() / currentSufArea) * (i + 1) * C_isect + (rightBounds.SurfaceArea() / currentSufArea) * (n - i - 1) * C_isect;
            if (cost < minCost){
                minCost = cost;
                bestSplit = i;
                bestAxis = axis;
            }
        }
    }
    if (bestSplit == -1){
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else {
        auto beginning = objects.begin();
        auto middling = objects.begin() + bestSplit + 1;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildSAH(leftshapes);
        node->right = recursiveBuildSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::BuildTree(std::vector<Object*> objects, SplitMethod method){
    switch (method){
    case SplitMethod::NAIVE:
        return recursiveBuildNaive(objects);
        break;
    case SplitMethod::SAH:
        return recursiveBuildSAH(objects);
        break;
    default:
        break;
    }
    return nullptr;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    
    Intersection intersec;
    auto x = intersec;
    if (node->bounds.IntersectP(ray, ray.direction_inv, {int(ray.direction_inv.x < 0), int(ray.direction_inv.y < 0), int(ray.direction_inv.z < 0)})) {
        if (node->left == nullptr && node->right == nullptr) {
            intersec = node->object->getIntersection(ray);
        }
        else {
            Intersection leftIntersec = getIntersection(node->left, ray);
            Intersection rightIntersec = getIntersection(node->right, ray);
            if (leftIntersec.distance < rightIntersec.distance) {
                intersec = leftIntersec;
            }
            else {
                intersec = rightIntersec;
            }
        }
    }
    return intersec;
}