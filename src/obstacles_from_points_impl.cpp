#include "obstacles_from_points_impl.h"

#include <limits>

std::vector<lms::math::vertex2f> ObstaclesFromPointsImpl::cullValidPoints(
    const lms::math::PointCloud2f& pointCloud,
    const lms::math::polyLine2f& centerLine) {
    std::vector<lms::math::vertex2f> validPoints;
    for (const auto& point : pointCloud.points()) {
        // check if point is something on the car
        if (point.x > m_obstaclePointMinXOffsetFront ||
            point.x < -m_obstaclePointMinXOffsetBack ||
            point.y > m_obstaclePointMinYOffsetLeft ||
            point.y < -m_obstaclePointMinYOffsetRight) {
            // check if point is beside the road
            if (centerLine.perpendicularDistance(point) < m_laneWidthMeter) {
                validPoints.push_back(point);
            }
        }
    }
    return validPoints;
}

street_environment::BoundingBox2fVector ObstaclesFromPointsImpl::getObstacles(
    const lms::math::PointCloud2f& pointCloud) {
    street_environment::BoundingBox2fVector obstacles;
    lms::math::PointCloud2f obstaclePoints;
    const lms::math::vertex2f* prevPoint = &(pointCloud.points().at(0));
    for (const auto& curPoint : pointCloud.points()) {
        if (prevPoint->distance(curPoint) > m_obstacleDistanceThresholdMeter) {
            if (obstaclePoints.size() >= m_obstaclePointThreshold) {
                obstacles.push_back(
                    street_environment::BoundingBox2f(obstaclePoints));
            }
            obstaclePoints.clear();
        }
        obstaclePoints.points().push_back(curPoint);
        prevPoint = &curPoint;
    }
    if (obstaclePoints.size() >= m_obstaclePointThreshold) {
        obstacles.push_back(street_environment::BoundingBox2f(obstaclePoints));
    }
    return obstacles;
}
