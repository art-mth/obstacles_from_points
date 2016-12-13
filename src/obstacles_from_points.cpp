#include "obstacles_from_points.h"

#include <lms/math/vertex.h>

bool ObstaclesFromPoints::initialize() {
    newData = readChannel<bool>("NEW_DATA");
    pointCloud = readChannel<lms::math::PointCloud2f>("POINT_CLOUD");
    centerLine = readChannel<lms::math::polyLine2f>("CENTER_LINE");
    poseHistory = readChannel<lms::math::Pose2DHistory>("POSE2D_HISTORY");
    culledPointCloud =
        writeChannel<lms::math::PointCloud2f>("CULLED_POINT_CLOUD");
    obstacles =
        writeChannel<street_environment::BasicObstacleVector>("OBSTACLES");

    impl =
        std::unique_ptr<ObstaclesFromPointsImpl>(new ObstaclesFromPointsImpl);
    configureImpl();
    return true;
}

bool ObstaclesFromPoints::deinitialize() { return true; }

void ObstaclesFromPoints::configsChanged() { configureImpl(); }

bool ObstaclesFromPoints::cycle() {
    lms::math::Pose2D deltaPose(getDeltaPose());
    impl->moveObstacles(*obstacles,
                        lms::math::vertex2f(deltaPose.x, deltaPose.y),
                        deltaPose.phi);

    if (*newData) {
        if (pointCloud->size() == 0) {
            return true;
        }
        culledPointCloud->points(
            impl->cullValidPoints(*pointCloud, *centerLine));
        if (culledPointCloud->size() == 0) {
            return true;
        }
        *obstacles = impl->cullOldObstacles(*obstacles);
        street_environment::BasicObstacleVector newObstacles =
            impl->getNewObstacles(*culledPointCloud);
        obstacles->insert(std::end(*obstacles), std::begin(newObstacles),
                          std::end(newObstacles));
    }

    return true;
}

void ObstaclesFromPoints::configureImpl() {
    impl->setLaneWidthMeter(config().get<float>("laneWidthMeter", 0.4));
    impl->setObstacleDistanceThresholdMeter(
        config().get<float>("obstacleDistanceThresholdMeter", 0.05));
    impl->setObstaclePointThreshold(
        config().get<int>("obstaclePointThreshold", 10));
    impl->setObstaclePointMinXOffsetFront(
        config().get<float>("obstaclePointMinXOffsetFront", 0.25));
    impl->setObstaclePointMinXOffsetBack(
        config().get<float>("obstaclePointMinXOffsetBack", 0.1));
    impl->setObstaclePointMinYOffsetLeft(
        config().get<float>("obstaclePointMinYOffsetLeft", 0.2));
    impl->setObstaclePointMinYOffsetRight(
        config().get<float>("obstaclePointMinYOffsetRight", 0.2));
    impl->setMaxObstacleTranslate(
        config().get<float>("maxObstacleTranslate", 0.5));
}

lms::math::Pose2D ObstaclesFromPoints::getDeltaPose() {
    lms::math::Pose2D oldPose, deltaPose;
    if (poseHistory->getPose(lastUpdate.toFloat<std::milli, double>(),
                             oldPose)) {
        lms::math::CoordinateSystem2D coord(oldPose);
        deltaPose = coord.transformTo(poseHistory->currentPose());
    } else {
        logger.warn("cycle") << "no valid pose found: "
                             << lastUpdate.toFloat<std::milli, double>();
    }
    lastUpdate = lms::Time::now();
    return deltaPose;
}
