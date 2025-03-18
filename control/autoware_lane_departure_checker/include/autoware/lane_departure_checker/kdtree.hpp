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
struct KDNode {
  Point2d point;
  std::unique_ptr<KDNode> left;
  std::unique_ptr<KDNode> right;
  KDNode(const Point2d & pt) : point(pt), left(nullptr), right(nullptr) {}
};

// Comparator for the max-heap used in k-nearest search.
struct ComparePair {
  bool operator()(const std::pair<double, Point2d> & a, const std::pair<double, Point2d> & b) const {
    return a.first < b.first;  // larger squared distance has higher priority
  }
};

class KDTree {
public:
  KDTree() = default;
  explicit KDTree(const std::vector<Point2d> & points) {
    std::vector<Point2d> pts = points;
    root = buildIterative(pts);
  }

  Point2d nearest(const Point2d & target) {
    Point2d best;
    double bestDist = std::numeric_limits<double>::max();
    nearestRec(root.get(), target, 0, best, bestDist);
    return best;
  }

  std::vector<Point2d> kNearest(const Point2d & target, size_t k) {
    std::priority_queue<
      std::pair<double, Point2d>,
      std::vector<std::pair<double, Point2d>>,
      ComparePair
    > best;

    kNearestRec(root.get(), target, 0, k, best);

    std::vector<Point2d> result;
    while (!best.empty()) {
      result.push_back(best.top().second);
      best.pop();
    }
    std::reverse(result.begin(), result.end());
    return result;
  }

private:
  std::unique_ptr<KDNode> root;

  std::unique_ptr<KDNode> build(std::vector<Point2d> & pts, int depth)
  {
    if (pts.empty()) return nullptr;

    if (pts.size() == 1) {
      return std::make_unique<KDNode>(pts[0]);  // base case
    }

    int axis = depth % 2;
    auto comparator = [axis](const Point2d & a, const Point2d & b) {
      return (axis == 0) ? (a.x() < b.x()) : (a.y() < b.y());
    };
    std::sort(pts.begin(), pts.end(), comparator);

    size_t medianIndex = pts.size() / 2;
    auto node = std::make_unique<KDNode>(pts[medianIndex]);

    // ⚠️ Check that you're not creating empty slices in a loop
    if (medianIndex > 0) {
      std::vector<Point2d> leftPts(pts.begin(), pts.begin() + medianIndex);
      node->left = build(leftPts, depth + 1);
    }

    if (medianIndex + 1 < pts.size()) {
      std::vector<Point2d> rightPts(pts.begin() + medianIndex + 1, pts.end());
      node->right = build(rightPts, depth + 1);
    }

    return node;
  }

  void nearestRec(KDNode * node, const Point2d & target, int depth, Point2d & best, double & bestDist) {
    if (!node) return;

    double d = distance(node->point, target);
    if (d < bestDist) {
      bestDist = d;
      best = node->point;
    }

    int axis = depth % 2;
    KDNode * nextNode = nullptr;
    KDNode * otherNode = nullptr;
    if ((axis == 0 && target.x() < node->point.x()) || (axis == 1 && target.y() < node->point.y())) {
      nextNode = node->left.get();
      otherNode = node->right.get();
    } else {
      nextNode = node->right.get();
      otherNode = node->left.get();
    }

    nearestRec(nextNode, target, depth + 1, best, bestDist);

    double diff = (axis == 0) ? std::abs(target.x() - node->point.x()) : std::abs(target.y() - node->point.y());
    if (diff < bestDist) {
      nearestRec(otherNode, target, depth + 1, best, bestDist);
    }
  }
std::unique_ptr<KDNode> buildIterative(std::vector<Point2d> & pts) {
  if (pts.empty()) return nullptr;

  struct WorkItem {
    std::vector<Point2d> points;
    int depth;
    KDNode * parent;
    bool isLeftChild;
  };

auto root = std::make_unique<KDNode>(Point2d{});  // dummy root

std::queue<WorkItem> queue;
queue.push({pts, 0, root.get(), false});

while (!queue.empty()) {
  auto [points, depth, parent, isLeft] = queue.front();
  queue.pop();

  if (points.empty()) continue;

  int axis = depth % 2;
  auto comparator = [axis](const Point2d & a, const Point2d & b) {
    return (axis == 0) ? (a.x() < b.x()) : (a.y() < b.y());
  };
  std::sort(points.begin(), points.end(), comparator);
  size_t medianIndex = points.size() / 2;

  auto node = std::make_unique<KDNode>(points[medianIndex]);
  KDNode * nodeRawPtr = node.get();

  // ✅ Instead of *parent = *node
  if (parent == root.get()) {
    root->point = node->point;
    root->left = std::move(node->left);
    root->right = std::move(node->right);
    nodeRawPtr = root.get();
  } else {
    if (isLeft) {
      parent->left = std::move(node);
    } else {
      parent->right = std::move(node);
    }
  }

  if (medianIndex > 0) {
    std::vector<Point2d> leftPts(points.begin(), points.begin() + medianIndex);
    queue.push({leftPts, depth + 1, nodeRawPtr, true});
  }

  if (medianIndex + 1 < points.size()) {
    std::vector<Point2d> rightPts(points.begin() + medianIndex + 1, points.end());
    queue.push({rightPts, depth + 1, nodeRawPtr, false});
  }
}

  return root;
}

  void kNearestRec(
    KDNode * node, const Point2d & target, int depth, size_t k,
    std::priority_queue<
      std::pair<double, Point2d>,
      std::vector<std::pair<double, Point2d>>,
      ComparePair> & best)
  {
    if (!node) return;

    const double d2 = squaredDistance(node->point, target);

    if (!std::isfinite(d2) || !std::isfinite(node->point.x()) || !std::isfinite(node->point.y())) {
      return;
    }

    const std::pair<double, Point2d> candidate(d2, node->point);

    if (best.size() < k) {
      best.push(candidate);
    } else if (d2 < best.top().first) {
      best.pop();
      best.push(candidate);
    }

    const int axis = depth % 2;
    KDNode * nextNode = nullptr;
    KDNode * otherNode = nullptr;

    if ((axis == 0 && target.x() < node->point.x()) || (axis == 1 && target.y() < node->point.y())) {
      nextNode = node->left.get();
      otherNode = node->right.get();
    } else {
      nextNode = node->right.get();
      otherNode = node->left.get();
    }

    kNearestRec(nextNode, target, depth + 1, k, best);

    const double diff = (axis == 0)
      ? std::abs(target.x() - node->point.x())
      : std::abs(target.y() - node->point.y());

    if (best.size() < k || (diff * diff) < best.top().first) {
      kNearestRec(otherNode, target, depth + 1, k, best);
    }
  }

  double distance(const Point2d & a, const Point2d & b) {
    return std::sqrt(squaredDistance(a, b));
  }

  double squaredDistance(const Point2d & a, const Point2d & b) {
    double dx = a.x() - b.x();
    double dy = a.y() - b.y();
    return dx * dx + dy * dy;
  }
};
}  // namespace autoware::lane_departure_checker

#endif  // AUTOWARE__LANE_DEPARTURE_CHECKER__KDTREE_HPP_
