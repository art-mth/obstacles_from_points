#ifndef OBSTACLES_FROM_POINTS_IMPL_H
#define OBSTACLES_FROM_POINTS_IMPL_H

#include <vector>

#include <lms/math/polyline.h>
#include <lms/math/vertex.h>
#include <street_environment/obstacle.h>
#include <street_environment/street_environment.h>

class ObstaclesFromPointsImpl {
   public:
    std::vector<const lms::math::vertex2f*> cullValidPoints(
        const lms::math::polyLine2f& points,
        const lms::math::polyLine2f& centerLine);

    void fillObstacles(const std::vector<const lms::math::vertex2f*>& points,
                       street_environment::EnvironmentObjects& obstacles);

    street_environment::ObstaclePtr createObstacle(
        const std::vector<const lms::math::vertex2f*>& blob);
};

#endif  // OBSTACLES_FROM_POINTS_IMPL_H
