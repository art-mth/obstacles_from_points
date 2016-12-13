#ifndef OBSTACLES_FROM_POINTS_H
#define OBSTACLES_FROM_POINTS_H

#include <memory>

#include <lms/math/point_cloud.h>
#include <lms/math/polyline.h>
#include <lms/math/pose.h>
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
    void configureImpl();
    lms::math::Pose2D getDeltaPose();

    std::unique_ptr<ObstaclesFromPointsImpl> impl;

    lms::Time lastUpdate;

    /////////////////////////////// Data Channels //////////////////////////////
    lms::ReadDataChannel<bool> newData;
    lms::ReadDataChannel<lms::math::PointCloud2f> pointCloud;
    lms::ReadDataChannel<lms::math::polyLine2f> centerLine;
    lms::ReadDataChannel<lms::math::Pose2DHistory> poseHistory;
    lms::WriteDataChannel<lms::math::PointCloud2f> culledPointCloud;
    lms::WriteDataChannel<street_environment::BoundingBox2fVector> obstacles;
};

#endif  // OBSTACLES_FROM_POINTS_H
