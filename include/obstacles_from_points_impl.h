#ifndef OBSTACLES_FROM_POINTS_IMPL_H
#define OBSTACLES_FROM_POINTS_IMPL_H

#include <vector>

#include <lms/math/point_cloud.h>
#include <lms/math/polyline.h>
#include <lms/math/vertex.h>
#include <street_environment/basic_obstacle.h>

class ObstaclesFromPointsImpl {
   public:
    /**
     * @brief Culls all points from pointCloud that are of interest for our
     * obstacle detection. For example we do not care about points that lie
     * beside the road.
     */
    std::vector<lms::math::vertex2f> cullValidPoints(
        const lms::math::PointCloud2f& pointCloud,
        const lms::math::polyLine2f& centerLine);

    /**
     * @brief Everytime we get new point data we discard our old points. We only
     * want to discard the points that we have new objective data for. Meaning
     * everything in our view range. Points that cannot be detected but that are
     * still relevant should be kept.
     * @param obstacles Last cycles obstacles moved into the current
     * cycles cartesian coordinates.
     * @returns All obstacles that are relevant for us. Meaning that lie in a
     * specific area that we have no new data for.
     */
    street_environment::BasicObstacleVector cullOldObstacles(
        const street_environment::BasicObstacleVector& obstacles);

    /**
     * @brief Everytime we get new point data we want to identify relevant
     * obstacles. Since the point data may contain false positives we cluster.
     * An obstacle is only detected if there is a high enough density of points
     * in a specific area.
     * @param pointCloud Culled point data that is relevant for detecting
     * obstacles (e.g On the road).
     * @returns All newly detected obstacles.
     */
    street_environment::BasicObstacleVector getNewObstacles(
        const lms::math::PointCloud2f& pointCloud);

    /**
     * @brief Translates all obstacles by deltaPosition and rotates by
     * deltaRotation. After this operation the obstacles are in accord with the
     * car cartestian plane of the current cycle.
     * @param deltaPosition Position difference in the cartesian plane of the
     * last cycle.
     * @param deltaRotation Rotation difference in radians between last cycles
     * and this cycles cartesian plane.
     */
    void moveObstacles(street_environment::BasicObstacleVector& obstacles,
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
    void setMaxObstacleTranslate(float maxTranslate) {
        m_maxObstacleTranslate = maxTranslate;
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
    float m_maxObstacleTranslate;
};

#endif  // OBSTACLES_FROM_POINTS_IMPL_H
