# A single tracked vehicle observation, as produced by a vehicle-detection
# perception node (e.g. camera+LiDAR fusion) for the school traffic control
# use case.

int32 id                      # stable track id from the upstream detector, -1 if unknown
float64 distance              # distance from the robot to the vehicle [m]
geometry_msgs/Point position  # vehicle position in the sensor/robot frame [m]
