#ifndef OBSTACLES_FROM_POINTS_H
#define OBSTACLES_FROM_POINTS_H

#include <memory>
#include <vector>

#include <lms/math/point_cloud.h>
#include <lms/math/polyline.h>
#include <lms/module.h>
#include <street_environment/bounding_box.h>

#include "obstacles_from_points_impl.h"

class ObstaclesFromPoints : public lms::Module {
   public:
    bool initialize();
    bool deinitialize();
    void configsChanged() override;
    bool cycle();

   private:
    lms::ReadDataChannel<lms::math::PointCloud2f> pointCloud;
    lms::ReadDataChannel<lms::math::polyLine2f> centerLine;
    lms::WriteDataChannel<street_environment::BoundedObstacles> obstacles;

    std::unique_ptr<ObstaclesFromPointsImpl> impl;

    void configureImpl();
};

#endif  // OBSTACLES_FROM_POINTS_H
