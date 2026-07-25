# A single tracked pedestrian observation with distance to the robot.

int32 id                      # stable track id from the upstream detector, -1 if unknown
float64 distance              # distance from the robot to the pedestrian [m]
geometry_msgs/Point position  # pedestrian position in the sensor/robot frame [m]
