# obstacles_from_points
This module is responsible for combining new and old point data for obstacle detection. Since we do not get new point data every cycle we move old points in no data cycles and add new points in data cycles.

## Data Channels
#### Read
- NEW_DATA(bool)
- POINT_CLOUD(lms::math::PointCloud2f)
- CENTER_LINE(lms::math::PolyLine2f)

#### Write
- CULLED_POINT_CLOUD(lms::math::PointCloud2f): On new data we write culled data points to this channel for debugging purposes. The points are currently not moved in no data cycles.
- OBSTACLES(street_environment::BasicObstacles): Obstacles in current car coordinates. These are moved in no data cycles.

## Configs
- laneWidthMeter(float)
- obstacleDistanceThresholdMeter(float): The maximum distance between subsequent points in a point cloud so that we see them as one entity.
- obstaclePointThreshold(unsigned): The minimum number of points a grouped point cloud must have so that we see it as an obstacle.
- maxObstacleTranslate(float): Obstacles that are more than maxObstacleTranslate distance behind the current car position are not interesting for us anymore. We discard them.

The offset values are important for invalidating points that lie on the car; Those might be cables or similiar sticking out. All values should be positive meter offsets.
- obstaclePointMinXOffsetFront(float)
- obstaclePointMinXOffsetBack(float)
- obstaclePointMinYOffsetLeft(float)
- obstaclePointMinYOffsetRight(float)
