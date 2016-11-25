#include "obstacles_from_points.h"

#include <lms/math/vertex.h>

bool ObstaclesFromPoints::initialize() {
    pointCloud = readChannel<lms::math::PointCloud2f>("POINT_CLOUD");
    centerLine = readChannel<lms::math::polyLine2f>("CENTER_LINE");
    obstacles = writeChannel<street_environment::BoundedObstacles>("OBSTACLES");

    impl =
        std::unique_ptr<ObstaclesFromPointsImpl>(new ObstaclesFromPointsImpl);
    configureImpl();
    return true;
}

bool ObstaclesFromPoints::deinitialize() { return true; }

void ObstaclesFromPoints::configsChanged() { configureImpl(); }

bool ObstaclesFromPoints::cycle() {
    obstacles->clear();
    if (pointCloud->points().size() == 0) {
        return true;
    }
    std::vector<const lms::math::vertex2f*> validPoints =
        impl->cullValidPoints(*pointCloud, *centerLine);
    if (validPoints.size() == 0) {
        return true;
    }
    impl->fillObstacles(validPoints, *obstacles);
    return true;
}

void ObstaclesFromPoints::configureImpl() {
    impl->setLaneWidthMeter(config().get<float>("laneWidthMeter", 0.4));
    impl->setObstacleDistanceThreshold(
        config().get<float>("obstacleDistanceThreshold", 0.05));
    impl->setObstaclePointThreshold(
        config().get<int>("obstaclePointThreshold", 10));
}
