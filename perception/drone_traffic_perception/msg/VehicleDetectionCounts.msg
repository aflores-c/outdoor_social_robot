# Per-frame vehicle detection counts published live by the VisDrone/YOLO
# TensorRT detector (main.py) running on the Jetson.
#
# header.stamp holds the ROS time at which this message was published.
# log_timestamp preserves the original "YYYY-MM-DD HH:MM:SS.mmm"
# wall-clock timestamp computed by current_timestamp() in main.py at the
# moment this frame finished processing.

std_msgs/Header header
string log_timestamp        # timestamp string from current_timestamp()
int32 raw_detections          # instantaneous YOLO box count for this frame
int32 ema_detections            # EMA-smoothed count (alpha=ema_alpha, ceil-rounded)
int32 average_detections          # rolling average of raw_detections over the
                                    # trailing average_window_seconds, rounded
                                    # to the nearest integer
