#include "obstacles_from_points_impl.h"

#include <limits>

void ObstaclesFromPointsImpl::cullValidPoints(
    const lms::math::PointCloud2f& pointCloud,
    const lms::math::polyLine2f& centerLine,
    lms::math::PointCloud2f& culledPointCloud) {
    for (const auto& point : pointCloud.points()) {
        // check if point is something on the car
        if (point.x > 0.25 || point.x < -0.1 || point.y > 0.1 ||
            point.y < -0.1) {
            float distanceToCenterLine = std::numeric_limits<float>::infinity();
            for (const auto& centerLinePoint : centerLine.points()) {
                float ndistance = centerLinePoint.distance(point);
                if (ndistance < distanceToCenterLine) {
                    distanceToCenterLine = ndistance;
                }
            }
            if (distanceToCenterLine < m_laneWidthMeter) {
                culledPointCloud.points().push_back(point);
            }
        }
    }
}

void ObstaclesFromPointsImpl::fillObstacles(
    const lms::math::PointCloud2f& pointCloud,
    street_environment::BoundedObstacles& obstacles) {
    std::vector<lms::math::vertex2f> obstaclePoints;
    const lms::math::vertex2f* prevPoint = &(pointCloud.points()[0]);
    for (const auto& curPoint : pointCloud.points()) {
        if (prevPoint->distance(curPoint) > m_obstacleDistanceThreshold) {
            if (obstaclePoints.size() >= m_obstaclePointThreshold) {
                obstacles.push_back(
                    street_environment::BoundingBox(obstaclePoints));
            }
            obstaclePoints.clear();
        }
        obstaclePoints.push_back(curPoint);
        prevPoint = &curPoint;
    }
    if (obstaclePoints.size() >= m_obstaclePointThreshold) {
        obstacles.push_back(street_environment::BoundingBox(obstaclePoints));
    }
}
