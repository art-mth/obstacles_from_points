#include "obstacles_from_points_impl.h"

#include <limits>

std::vector<lms::math::vertex2f> ObstaclesFromPointsImpl::cullValidPoints(
    const lms::math::PointCloud2f& pointCloud,
    const lms::math::polyLine2f& centerLine) {
    std::vector<lms::math::vertex2f> validPoints;
    for (const auto& point : pointCloud.points()) {
        // check if point is something on the car
        if (point.x > 0.25 || point.x < -0.1 || point.y > 0.1 ||
            point.y < -0.1) {
            float distanceToCenterLine = std::numeric_limits<float>::infinity();
            for (const auto& centerLinePoint : centerLine.points()) {
                float distance = centerLinePoint.distance(point);
                if (distance < distanceToCenterLine) {
                    distanceToCenterLine = distance;
                }
            }
            if (distanceToCenterLine < m_laneWidthMeter) {
                validPoints.push_back(point);
            }
        }
    }
    return validPoints;
}

street_environment::BoundingBoxVector ObstaclesFromPointsImpl::getObstacles(
    const lms::math::PointCloud2f& pointCloud) {
    street_environment::BoundingBoxVector obstacles;
    lms::math::PointCloud2f obstaclePoints;
    const lms::math::vertex2f* prevPoint = &(pointCloud.points().at(0));
    for (const auto& curPoint : pointCloud.points()) {
        if (prevPoint->distance(curPoint) > m_obstacleDistanceThreshold) {
            if (obstaclePoints.size() >= m_obstaclePointThreshold) {
                obstacles.push_back(
                    street_environment::BoundingBox(obstaclePoints));
            }
            obstaclePoints.clear();
        }
        obstaclePoints.points().push_back(curPoint);
        prevPoint = &curPoint;
    }
    if (obstaclePoints.size() >= m_obstaclePointThreshold) {
        obstacles.push_back(street_environment::BoundingBox(obstaclePoints));
    }
    return obstacles;
}
