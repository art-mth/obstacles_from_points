#include "obstacles_from_points_impl.h"

#include <limits>

std::vector<const lms::math::vertex2f*> ObstaclesFromPointsImpl::cullValidPoints(
    const lms::math::polyLine2f& points,
    const lms::math::polyLine2f& centerLine) {
    std::vector<const lms::math::vertex2f*> validPoints;
    for (const auto& point : points.points()) {
        // check if point is something on the car
        if (point.x > 0.25 || point.x < -0.1 ||
            point.y > 0.1 || point.y < -0.1) {
            float distanceToCenterLine = std::numeric_limits<float>::infinity();
            for (const auto& centerLinePoint : centerLine.points()) {
                float ndistance = centerLinePoint.distance(point);
                if (ndistance < distanceToCenterLine) {
                    distanceToCenterLine = ndistance;
                }
            }
            if (distanceToCenterLine < m_laneWidthMeter) {
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
        if (prevPoint->distance(*curPoint) > m_obstacleDistanceThreshold) {
            if (blob.size() >= m_obstaclePointThreshold) {
                obstacles.push_back(street_environment::BoundingBox(blob));
            }
            blob.clear();
        }
        blob.push_back(*curPoint);
        prevPoint = curPoint;
    }
    if (blob.size() >= m_obstaclePointThreshold) {
        obstacles.push_back(street_environment::BoundingBox(blob));
    }
}
