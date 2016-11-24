#include "obstacles_from_points_impl.h"

#include <limits>

namespace {
const float kLaneWidth = 0.4;
const float kBlobDistanceThreshold = 0.1;
const int kMinBlobElements = 5;

std::vector<lms::math::vertex2f> createBoundingBox(
    const std::vector<const lms::math::vertex2f*>& blob) {
    int minX = blob[0]->x;
    int maxX = blob[0]->x;
    int minY = blob[0]->y;
    int maxY = blob[0]->y;

    for (const auto point : blob) {
        if (point->x < minX) {
            minX = point->x;
        }
        if (point->x > maxX) {
            maxX = point->x;
        }
        if (point->y < minY) {
            minY = point->y;
        }
        if (point->y > maxY) {
            maxY = point->y;
        }
    }

    std::vector<lms::math::vertex2f> boundingBoxPoints;
    boundingBoxPoints.push_back(lms::math::vertex2f(minX, minY));
    boundingBoxPoints.push_back(lms::math::vertex2f(maxX, minY));
    boundingBoxPoints.push_back(lms::math::vertex2f(maxX, maxY));
    boundingBoxPoints.push_back(lms::math::vertex2f(minX, maxY));
    return boundingBoxPoints;
}
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
    street_environment::EnvironmentObjects& obstacles) {
    std::vector<const lms::math::vertex2f*> blob;
    const lms::math::vertex2f* prevPoint = points[0];
    for (const auto curPoint : points) {
        if (prevPoint->distance(*curPoint) <= kBlobDistanceThreshold) {
            blob.push_back(curPoint);
        } else {
            if(blob.size() >= kMinBlobElements) {
                obstacles.objects.push_back(createObstacle(blob));
            }
            blob.clear();
            blob.push_back(curPoint);
        }
        prevPoint = curPoint;
    }
}

street_environment::ObstaclePtr ObstaclesFromPointsImpl::createObstacle(const std::vector<const lms::math::vertex2f*>& blob) {
    std::vector<lms::math::vertex2f> boundingBox = createBoundingBox(blob);
    street_environment::ObstaclePtr obstacle(
        new street_environment::Obstacle());
    for (const auto& point : boundingBox) {
        obstacle->addPoint(point);
    }
    return obstacle;
}
