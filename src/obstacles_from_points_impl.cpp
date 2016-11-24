#include "obstacles_from_points_impl.h"

#include <limits>

namespace {
const float kLaneWidth = 0.4;
const float kBlobDistanceThreshold = 0.1;
const int kMinBlobElements = 5;
}

std::vector<const lms::math::vertex2f*> ObstaclesFromPointsImpl::cullValidPoints(
    const lms::math::polyLine2f& points,
    const lms::math::polyLine2f& centerLine) {
    std::vector<const lms::math::vertex2f*> validPoints;
    for (const auto& point : points.points()) {
        // check if point is something on the car
        if ((point.x > 0.25 || point.x < -0.1) &&
            (point.y > 0.1 || point.y < -0.1)) {
            float distanceToCenterLine = std::numeric_limits<float>::infinity();
            for (const auto& centerLinePoint : centerLine.points()) {
                float ndistance = centerLinePoint.distance(point);
                if (ndistance < distanceToCenterLine) {
                    distanceToCenterLine = ndistance;
                }
            }
            if (distanceToCenterLine < kLaneWidth) {
                validPoints.push_back(&point);
            }
        }
    }
    return validPoints;
}

void ObstaclesFromPointsImpl::fillObstacles(
    const std::vector<const lms::math::vertex2f*>& points,
    street_environment::BoundedObstacles& obstacles) {
    std::vector<lms::math::vertex2f> blob;
    const lms::math::vertex2f* prevPoint = points[0];
    for (const auto curPoint : points) {
        if (prevPoint->distance(*curPoint) > kBlobDistanceThreshold) {
            if(blob.size() >= kMinBlobElements) {
                obstacles.push_back(street_environment::BoundingBox(blob));
            }
            blob.clear();
        }
        blob.push_back(*curPoint);
        prevPoint = curPoint;
    }
}
