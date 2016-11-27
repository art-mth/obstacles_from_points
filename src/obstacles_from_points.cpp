#include "obstacles_from_points.h"

#include <lms/math/vertex.h>

bool ObstaclesFromPoints::initialize() {
    newData = readChannel<bool>("NEW_DATA");
    pointCloud = readChannel<lms::math::PointCloud2f>("POINT_CLOUD");
    centerLine = readChannel<lms::math::polyLine2f>("CENTER_LINE");
    culledPointCloud =
        writeChannel<lms::math::PointCloud2f>("CULLED_POINT_CLOUD");
    obstacles =
        writeChannel<street_environment::BoundingBoxVector>("OBSTACLES");

    impl =
        std::unique_ptr<ObstaclesFromPointsImpl>(new ObstaclesFromPointsImpl);
    configureImpl();
    return true;
}

bool ObstaclesFromPoints::deinitialize() { return true; }

void ObstaclesFromPoints::configsChanged() { configureImpl(); }

bool ObstaclesFromPoints::cycle() {
    if (*newData) {
        if (pointCloud->size() == 0) {
            return true;
        }
        culledPointCloud->points(
            impl->cullValidPoints(*pointCloud, *centerLine));
        if (culledPointCloud->size() == 0) {
            return true;
        }
        *obstacles = impl->getObstacles(*culledPointCloud);
    }
    return true;
}

void ObstaclesFromPoints::configureImpl() {
    impl->setLaneWidthMeter(config().get<float>("laneWidthMeter", 0.4));
    impl->setObstacleDistanceThreshold(
        config().get<float>("obstacleDistanceThreshold", 0.05));
    impl->setObstaclePointThreshold(
        config().get<int>("obstaclePointThreshold", 10));
}
