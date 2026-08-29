# Periodic drone RTMP link/decode status, published alongside
# VehicleDetectionCounts for field-trial data collection.
#
# decode_latency_ms is NOT true GStreamer/NVDEC decode time — there is no
# instrumentation point inside the actual decode path in this script. It is
# decoded_frame_waiting_age_ms: how stale a frame was (time since it was
# decoded by the capture thread) when the inference loop consumed it — an
# end-to-end staleness/queueing proxy, only meaningful for the RTMP path.

std_msgs/Header header
bool connected                # LatestFrameCapture.connected at publish time
float64 fps                   # frame rate averaged since the previous publish
int32 dropped_frames_total    # cumulative frames decoded but never consumed
float64 decode_latency_ms     # see note above — frame-staleness proxy, not decoder latency
