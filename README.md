# obstacles_from_points

## Data Channels
#### Read
- NEW_DATA(bool)
- POINT_CLOUD(lms::math::PointCloud2f)
- CENTER_LINE(lms::math::PolyLine2f)

#### Write
- CULLED_POINT_CLOUD(lms::math::PointCloud2f)
- OBSTACLES(street_environment::BoundingBox2fVector)

## Configs
- laneWidthMeter(float)
- obstacleDistanceThresholdMeter(float): The maximum distance between subsequent points in a point cloud so that we see them as one entity.
- obstaclePointThreshold(unsigned): The minimum number of points a grouped point cloud must have so that we see it as an obstacle.

The offset values are important for invalidating points that lie on the car; Those might be cables or similiar sticking out. All values should be positive meter offsets.
- obstaclePointMinXOffsetFront(float)
- obstaclePointMinXOffsetBack(float)
- obstaclePointMinYOffsetLeft(float)
- obstaclePointMinYOffsetRight(float)
