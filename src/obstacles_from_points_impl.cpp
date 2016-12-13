#include "obstacles_from_points_impl.h"

#include <limits>

#include <street_environment/bounding_box.h>

namespace {
street_environment::BasicObstacle culledObstacle(
    float minX, float maxX, const street_environment::BasicObstacle& obstacle) {
    std::vector<lms::math::vertex2f> culledPoints;
    for (const auto& point : obstacle.points()) {
        if(point.x > minX && point.x < maxX) {
            culledPoints.push_back(point);
        }
    }
    return street_environment::BasicObstacle(culledPoints);
}
}

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

street_environment::BasicObstacleVector
ObstaclesFromPointsImpl::cullOldObstacles(
    const street_environment::BasicObstacleVector& obstacles) {
    street_environment::BasicObstacleVector culledObstacles;
    for (const auto& obstacle : obstacles) {
        street_environment::BoundingBox2f boundingBox = obstacle.boundingBox();
        if (boundingBox.corners().at(1).x > -m_maxObstacleTranslate &&
            boundingBox.corners().at(0).x < 0) {
            culledObstacles.push_back(
                culledObstacle(-m_maxObstacleTranslate, 0, obstacle));
        }
    }
    return culledObstacles;
}

street_environment::BasicObstacleVector
ObstaclesFromPointsImpl::getNewObstacles(
    const lms::math::PointCloud2f& pointCloud) {
    street_environment::BasicObstacleVector obstacles;
    lms::math::PointCloud2f obstaclePoints;
    const lms::math::vertex2f* prevPoint = &(pointCloud.points().at(0));
    for (const auto& curPoint : pointCloud.points()) {
        if (prevPoint->distance(curPoint) > m_obstacleDistanceThresholdMeter) {
            if (obstaclePoints.size() >= m_obstaclePointThreshold) {
                obstacles.push_back(
                    street_environment::BasicObstacle(obstaclePoints.points()));
            }
            obstaclePoints.clear();
        }
        obstaclePoints.points().push_back(curPoint);
        prevPoint = &curPoint;
    }
    if (obstaclePoints.size() >= m_obstaclePointThreshold) {
        obstacles.push_back(
            street_environment::BasicObstacle(obstaclePoints.points()));
    }
    return obstacles;
}

void ObstaclesFromPointsImpl::moveObstacles(
    street_environment::BasicObstacleVector& obstacles,
    const lms::math::vertex2f& deltaPosition, float deltaRotation) {
    for (auto& obstacle : obstacles) {
        obstacle.translate(-deltaPosition);
        obstacle.rotate(deltaRotation);
    }
}
