#ifndef CG4__BVH_H_
#define CG4__BVH_H_
#include "Ogre.h"

typedef std::pair<Ogre::Vector3, Ogre::Vector3> AABB;
// int counter = 0;

class TriangleTestResult {
 public:
  explicit TriangleTestResult(bool hit, Ogre::Real t, Ogre::Vector3 normal, int source)
      : hit(hit), t(t), normal(normal), source(source) {}

  bool hit;
  Ogre::Real t;
  Ogre::Vector3 normal;
  int source;
};

class Triangle {
 public:
  explicit Triangle(const Ogre::Vector3& corner_1,
                    const Ogre::Vector3& corner_2,
                    const Ogre::Vector3& corner_3,
                    const Ogre::Vector3& normal,
                    int index)
      : v0(corner_1), v1(corner_2), v2(corner_3), normal(normal), index(index) {}

  Triangle(const Triangle& triangle) {
    v0 = triangle.v0;
    v1 = triangle.v1;
    v2 = triangle.v2;
    normal = triangle.normal;
    index = triangle.index;
  }

  Ogre::Vector3 v0, v1, v2, normal;
  int index;

  TriangleTestResult hit(Ogre::Ray ray) const {
    // counter++;
    // Determine whether a ray intersect with a triangle
    const Ogre::Vector3 orig = ray.getOrigin();
    const Ogre::Vector3 dir = ray.getDirection();
    Ogre::Vector3 E1 = v1 - v0;
    Ogre::Vector3 E2 = v2 - v0;
    Ogre::Vector3 P = dir.crossProduct(E2);
    // determinant
    float det = E1.dotProduct(P), u, v, t = 0.0;
    // keep det > 0, modify T accordingly
    Ogre::Vector3 T;
    if (det > 0) {
      T = orig - v0;
    } else {
      T = v0 - orig;
      det = -det;
    }
    // If determinant is near zero, ray lies in plane of triangle
    if (det < 0.0001f)
      return TriangleTestResult(false, t, Ogre::Vector3::ZERO, -1);
    // Calculate u and make sure u <= 1
    u = T.dotProduct(P);
    if (u < 0.0f || u > det)
      return TriangleTestResult(false, t, Ogre::Vector3::ZERO, -1);
    // Q
    Ogre::Vector3 Q = T.crossProduct(E1);
    // Calculate v and make sure u + v <= 1
    v = dir.dotProduct(Q);
    t = E2.dotProduct(Q) / det;
    return TriangleTestResult(!(v < 0.0f || u + v > det || t < 1e-5), t, this->normal, index);
  }

  AABB getAABB() const {
    return std::make_pair(
        Ogre::Vector3(min(v0.x, v1.x, v2.x), min(v0.y, v1.y, v2.y), min(v0.z, v1.z, v2.z)),
        Ogre::Vector3(max(v0.x, v1.x, v2.x), max(v0.y, v1.y, v2.y), max(v0.z, v1.z, v2.z)));
  }

  bool operator<(const Triangle& x) const {
    return this->getAABB() < x.getAABB();
  }

 private:
  Ogre::Real min(Ogre::Real c1, Ogre::Real c2, Ogre::Real c3) const {
    return std::min(std::min(c1, c2), c3);
  }

  Ogre::Real max(Ogre::Real c1, Ogre::Real c2, Ogre::Real c3) const {
    return std::max(std::max(c1, c2), c3);
  }
};

class BVHNode {
 public:
  BVHNode* left{nullptr}, * right{nullptr};
  AABB bound;
  Triangle triangle;

  BVHNode() : triangle(Ogre::Vector3::ZERO,
                       Ogre::Vector3::ZERO,
                       Ogre::Vector3::ZERO,
                       Ogre::Vector3::ZERO,
                       -1) {}

  bool hit(Ogre::Ray ray) const {
    Ogre::RayTestResult res = ray.intersects(Ogre::AxisAlignedBox(bound.first, bound.second));
    return res.first && (res.second >= 0);
  }

  ~BVHNode() {
    delete left;
    delete right;
  }
};

AABB mergeAABB(AABB a, AABB b) {
  Ogre::Vector3 min_pos(std::min(a.first.x, b.first.x),
                        std::min(a.first.y, b.first.y),
                        std::min(a.first.z, b.first.z));
  Ogre::Vector3 max_pos(std::max(a.second.x, b.second.x),
                        std::max(a.second.y, b.second.y),
                        std::max(a.second.z, b.second.z));
  return std::make_pair(min_pos, max_pos);
}

TriangleTestResult mergeTriangleTestResult(TriangleTestResult a, TriangleTestResult b) {
  if (!a.hit && !b.hit) return a;
  if (!a.hit && b.hit) return b;
  if (a.hit && !b.hit) return a;
  if (a.t < b.t) return a;
  else if (a.t > b.t) return b;
  else if (a.t == b.t) {
    if (a.source < b.source) return b;
    else return a;
  }
}

void build(BVHNode*& root, std::vector<Triangle>& triangles, int l, int r) {
  if (l > r) return;
  std::sort(triangles.begin() + l, triangles.begin() + r + 1);
  int mid = (l + r) / 2;
  if (root == nullptr) root = new BVHNode();
  if (l == r) {
    root->triangle = triangles[mid];
    root->bound = root->triangle.getAABB();
  } else {
    build(root->left, triangles, l, mid);
    build(root->right, triangles, mid + 1, r);
    assert(root->left != nullptr && root->right != nullptr);
    root->bound = mergeAABB(root->left->bound, root->right->bound);
  }
}

TriangleTestResult BVHintersect(const BVHNode* root, Ogre::Ray ray) {
  if (root->left == nullptr && root->right == nullptr) return root->triangle.hit(ray);
  assert(root->left != nullptr && root->right != nullptr);
  TriangleTestResult result(false, 0, Ogre::Vector3::ZERO, -1);
  if (root->left->hit(ray)) result = mergeTriangleTestResult(result, BVHintersect(root->left, ray));
  if (root->right->hit(ray)) result = mergeTriangleTestResult(result, BVHintersect(root->right, ray));
  return result;
}

#endif //CG4__BVH_H_
