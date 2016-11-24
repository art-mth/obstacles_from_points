#ifndef OBSTACLES_FROM_POINTS_IMPL_H
#define OBSTACLES_FROM_POINTS_IMPL_H

#include <vector>

#include <lms/math/polyline.h>
#include <lms/math/vertex.h>
#include <street_environment/bounding_box.h>

class ObstaclesFromPointsImpl {
   public:
    std::vector<const lms::math::vertex2f*> cullValidPoints(
        const lms::math::polyLine2f& points,
        const lms::math::polyLine2f& centerLine);

    void fillObstacles(const std::vector<const lms::math::vertex2f*>& points,
                       street_environment::BoundedObstacles& obstacles);
};

#endif  // OBSTACLES_FROM_POINTS_IMPL_H
