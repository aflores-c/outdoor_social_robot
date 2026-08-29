# A single tracked vehicle observation, as produced by a vehicle-detection
# perception node (e.g. camera+LiDAR fusion) for the school traffic control
# use case.

int32 id                      # stable track id from the upstream detector, -1 if unknown
float64 distance              # distance from the robot to the vehicle [m]
geometry_msgs/Point position  # vehicle position in the sensor/robot frame [m]
bool stopped                  # true once this vehicle's distance has stayed within
                               # stopped_distance_epsilon_m for stopped_confirm_time_s.
                               # Single-target continuity heuristic (closest vehicle
                               # per frame), not full multi-object tracking — see id.
float64 confidence            # YOLO detection confidence [0,1] for this box
