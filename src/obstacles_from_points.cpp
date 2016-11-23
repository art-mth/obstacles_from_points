#include "obstacles_from_points.h"

bool ObstaclesFromPoints::initialize() {
    points = readChannel<lms::math::polyLine2f>("POINTS");
    centerLine = readChannel<lms::math::polyLine2f>("CENTER_LINE");
    obstacles = writeChannel<street_environment::EnvironmentObjects>("OBSTACLES");
    return true;
}

bool ObstaclesFromPoints::deinitialize() { return true; }

void ObstaclesFromPoints::configsChanged() {}

bool ObstaclesFromPoints::cycle() { return true; }
