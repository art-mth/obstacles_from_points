#ifndef OBSTACLES_FROM_POINTS_IMPL_H
#define OBSTACLES_FROM_POINTS_IMPL_H

#include <vector>

#include <lms/math/point_cloud.h>
#include <lms/math/polyline.h>
#include <lms/math/vertex.h>
#include <street_environment/bounding_box.h>

class ObstaclesFromPointsImpl {
   public:
    std::vector<lms::math::vertex2f> cullValidPoints(
        const lms::math::PointCloud2f& pointCloud,
        const lms::math::polyLine2f& centerLine);

    street_environment::BoundingBox2fVector cullOldObstacles(
        const street_environment::BoundingBox2fVector& obstacles);

    street_environment::BoundingBox2fVector getNewObstacles(
        const lms::math::PointCloud2f& pointCloud);

    void moveObstacles(street_environment::BoundingBox2fVector& obstacles,
                       const lms::math::vertex2f& deltaPosition,
                       float deltaRotation);

    ////////////////////////////// Config Setters //////////////////////////////
    void setLaneWidthMeter(float laneWidthMeter) {
        m_laneWidthMeter = laneWidthMeter;
    }
    void setObstacleDistanceThresholdMeter(
        float obstacleDistanceThresholdMeter) {
        m_obstacleDistanceThresholdMeter = obstacleDistanceThresholdMeter;
    }
    void setObstaclePointThreshold(unsigned obstaclePointThreshold) {
        m_obstaclePointThreshold = obstaclePointThreshold;
    }
    void setObstaclePointMinXOffsetFront(float minOffset) {
        m_obstaclePointMinXOffsetFront = minOffset;
    }
    void setObstaclePointMinXOffsetBack(float minOffset) {
        m_obstaclePointMinXOffsetBack = minOffset;
    }
    void setObstaclePointMinYOffsetLeft(float minOffset) {
        m_obstaclePointMinYOffsetLeft = minOffset;
    }
    void setObstaclePointMinYOffsetRight(float minOffset) {
        m_obstaclePointMinYOffsetRight = minOffset;
    }

   private:
    ////////////////////////////// Config Values ///////////////////////////////
    float m_laneWidthMeter;
    float m_obstacleDistanceThresholdMeter;
    unsigned m_obstaclePointThreshold;
    float m_obstaclePointMinXOffsetFront;
    float m_obstaclePointMinXOffsetBack;
    float m_obstaclePointMinYOffsetLeft;
    float m_obstaclePointMinYOffsetRight;
};

#endif  // OBSTACLES_FROM_POINTS_IMPL_H
