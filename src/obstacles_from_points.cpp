#include "obstacles_from_points.h"

#include <lms/math/vertex.h>

bool ObstaclesFromPoints::initialize() {
    points = readChannel<lms::math::polyLine2f>("POINTS");
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
    if (points->points().size() == 0) {
        logger.debug() << "No points";
        return true;
    }
    std::vector<const lms::math::vertex2f*> validPoints =
        impl->cullValidPoints(*points, *centerLine);
    if (validPoints.size() == 0) {
        logger.debug() << "No valid points";
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
