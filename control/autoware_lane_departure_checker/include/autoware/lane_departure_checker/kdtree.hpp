// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef AUTOWARE__LANE_DEPARTURE_CHECKER__KDTREE_HPP_
#define AUTOWARE__LANE_DEPARTURE_CHECKER__KDTREE_HPP_

#include <autoware/lane_departure_checker/parameters.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace autoware::lane_departure_checker
{
// Node structure for the kd-tree.
struct KDNode
{
  Point2d point;
  KDNode * left;
  KDNode * right;
  KDNode(const Point2d & pt) : point(pt), left(nullptr), right(nullptr) {}
};

// Comparator for the max-heap used in k-nearest search.
// We store pairs (squared_distance, Point2d) and want the largest distance at the top.
struct ComparePair
{
  bool operator()(const std::pair<double, Point2d> & a, const std::pair<double, Point2d> & b) const
  {
    return a.first < b.first;  // larger squared distance has higher priority
  }
};

class KDTree
{
public:
  // Build the kd-tree from a vector of points.
  KDTree() = default;
  explicit KDTree(const std::vector<Point2d> & points)
  {
    std::vector<Point2d> pts = points;
    root = build(pts, 0);
  }

  ~KDTree() { freeTree(root); }

  // Return the single nearest neighbor (for backward compatibility).
  Point2d nearest(const Point2d & target)
  {
    Point2d best;
    double bestDist = std::numeric_limits<double>::max();
    nearestRec(root, target, 0, best, bestDist);
    return best;
  }

  // Return the k nearest neighbors to target.
  std::vector<Point2d> kNearest(const Point2d & target, size_t k)
  {
    // Priority queue (max-heap) storing (squared_distance, point).
    std::priority_queue<
      std::pair<double, Point2d>, std::vector<std::pair<double, Point2d>>, ComparePair>
      best;
    kNearestRec(root, target, 0, k, best);
    std::vector<Point2d> result;
    while (!best.empty()) {
      result.push_back(best.top().second);
      best.pop();
    }
    // The results are in reverse order (largest distance first), so reverse them.
    std::reverse(result.begin(), result.end());
    return result;
  }

private:
  KDNode * root;

  // Recursively build the kd-tree.
  KDNode * build(std::vector<Point2d> & pts, int depth)
  {
    if (pts.empty()) return nullptr;

    int axis = depth % 2;  // 0 for x, 1 for y.
    auto comparator = [axis](const Point2d & a, const Point2d & b) {
      return (axis == 0) ? (a.x() < b.x()) : (a.y() < b.y());
    };
    std::sort(pts.begin(), pts.end(), comparator);

    size_t medianIndex = pts.size() / 2;
    auto * node = new KDNode(pts[medianIndex]);

    // Build left and right subtrees.
    std::vector<Point2d> leftPts(pts.begin(), pts.begin() + medianIndex);
    std::vector<Point2d> rightPts(pts.begin() + medianIndex + 1, pts.end());
    node->left = build(leftPts, depth + 1);
    node->right = build(rightPts, depth + 1);

    return node;
  }

  // Recursive nearest neighbor search (for a single nearest neighbor).
  void nearestRec(
    KDNode * node, const Point2d & target, int depth, Point2d & best, double & bestDist)
  {
    if (!node) return;

    double d = distance(node->point, target);
    if (d < bestDist) {
      bestDist = d;
      best = node->point;
    }

    int axis = depth % 2;
    KDNode * nextNode = nullptr;
    KDNode * otherNode = nullptr;
    if (
      (axis == 0 && target.x() < node->point.x()) || (axis == 1 && target.y() < node->point.y())) {
      nextNode = node->left;
      otherNode = node->right;
    } else {
      nextNode = node->right;
      otherNode = node->left;
    }

    nearestRec(nextNode, target, depth + 1, best, bestDist);

    double diff =
      (axis == 0) ? std::abs(target.x() - node->point.x()) : std::abs(target.y() - node->point.y());
    if (diff < bestDist) {
      nearestRec(otherNode, target, depth + 1, best, bestDist);
    }
  }

  // Recursive k-nearest neighbor search.
  void kNearestRec(
    KDNode * node, const Point2d & target, int depth, size_t k,
    std::priority_queue<
      std::pair<double, Point2d>, std::vector<std::pair<double, Point2d>>, ComparePair> & best)
  {
    if (!node) return;

    double d2 = squaredDistance(node->point, target);
    // If we haven't found k points yet, push the current point.
    if (best.size() < static_cast<size_t>(k)) {
      best.emplace(d2, node->point);
    }
    // Otherwise, if the current point is closer than the worst in the heap, replace it.
    else if (d2 < best.top().first) {
      best.pop();
      best.emplace(d2, node->point);
    }

    int axis = depth % 2;
    KDNode * nextNode = nullptr;
    KDNode * otherNode = nullptr;
    if (
      (axis == 0 && target.x() < node->point.x()) || (axis == 1 && target.y() < node->point.y())) {
      nextNode = node->left;
      otherNode = node->right;
    } else {
      nextNode = node->right;
      otherNode = node->left;
    }

    kNearestRec(nextNode, target, depth + 1, k, best);

    double diff =
      (axis == 0) ? std::abs(target.x() - node->point.x()) : std::abs(target.y() - node->point.y());
    // Check if it's possible that the other side of the split contains a closer point.
    if (best.size() < k || (diff * diff) < best.top().first) {
      kNearestRec(otherNode, target, depth + 1, k, best);
    }
  }

  // Compute Euclidean distance.
  double distance(const Point2d & a, const Point2d & b) { return std::sqrt(squaredDistance(a, b)); }

  // Compute squared Euclidean distance.
  double squaredDistance(const Point2d & a, const Point2d & b)
  {
    double dx = a.x() - b.x();
    double dy = a.y() - b.y();
    return dx * dx + dy * dy;
  }

  // Recursively free the kd-tree.
  void freeTree(KDNode * node)
  {
    if (!node) return;
    freeTree(node->left);
    freeTree(node->right);
    delete node;
  }
};
}  // namespace autoware::lane_departure_checker

#endif  // AUTOWARE__LANE_DEPARTURE_CHECKER__KDTREE_HPP_
