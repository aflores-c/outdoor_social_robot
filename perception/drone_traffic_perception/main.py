import os
import cv2
import time
import csv
import math
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

from ultralytics import YOLO

import rclpy
from rclpy.node import Node

from drone_traffic_perception.msg import VehicleDetectionCounts, DroneLinkStatus


# ============================================================
# PATHS
# ============================================================

# All relative asset paths (model, video, logs, outputs) are resolved
# against the directory this script lives in, not the current working
# directory. This lets main.py run from anywhere without needing to cd
# into a specific folder first.
SCRIPT_DIR = Path(__file__).resolve().parent


# ============================================================
# VISDRONE CLASSES
# ============================================================

VISDRONE_CLASSES = {
    0: "pedestrian",
    1: "people",
    2: "bicycle",
    3: "car",
    4: "van",
    5: "truck",
    6: "tricycle",
    7: "awning-tricycle",
    8: "bus",
    9: "motor",
}


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def create_parent_directory(file_path):
    """Create the parent directory when one is specified."""

    parent_directory = os.path.dirname(file_path)

    if parent_directory:
        os.makedirs(
            parent_directory,
            exist_ok=True
        )


def current_timestamp():
    """Return local time with millisecond precision."""

    return datetime.now().strftime(
        "%Y-%m-%d %H:%M:%S.%f"
    )[:-3]


def opencv_has_gstreamer():
    """Return True if this OpenCV build supports GStreamer."""

    build_information = cv2.getBuildInformation()

    for line in build_information.splitlines():

        if "GStreamer" in line:

            value = line.split(":")[-1].strip().upper()

            return value == "YES"

    return False


def build_rtmp_gstreamer_pipeline(
    rtmp_url,
    codec="h264"
):
    """
    Build a low-latency RTMP pipeline for NVIDIA Jetson.

    Pipeline:
        RTMP
        -> FLV demux
        -> H.264/H.265 parser
        -> NVIDIA NVDEC hardware decoder
        -> NVIDIA color converter
        -> OpenCV BGR appsink

    Stale buffers are dropped by both the leaky queue and appsink.
    """

    codec = codec.lower()

    if codec in ("h264", "avc"):
        parser = "h264parse"

    elif codec in ("h265", "hevc"):
        parser = "h265parse"

    else:
        raise ValueError(
            f"Unsupported RTMP codec: {codec}"
        )

    pipeline = (
        f'rtmpsrc location="{rtmp_url}" '
        f'! flvdemux '
        f'! queue '
        f'max-size-buffers=2 '
        f'max-size-bytes=0 '
        f'max-size-time=0 '
        f'leaky=downstream '
        f'! {parser} '
        f'! nvv4l2decoder '
        f'! nvvidconv '
        f'! video/x-raw,format=BGRx '
        f'! videoconvert '
        f'! video/x-raw,format=BGR '
        f'! appsink '
        f'drop=true '
        f'max-buffers=1 '
        f'sync=false'
    )

    return pipeline


# ============================================================
# LATEST-FRAME RTMP CAPTURE
# ============================================================

class LatestFrameCapture:
    """
    Decode an RTMP stream continuously in a background thread.

    Only the newest decoded frame is retained. Older frames are
    overwritten rather than queued, preventing latency buildup
    when inference is slower than the incoming stream.
    """

    def __init__(
        self,
        source,
        reconnect_delay=1.0,
        use_nvdec=True,
        codec="h264",
        fallback_to_ffmpeg=True
    ):

        self.source = source
        self.reconnect_delay = reconnect_delay
        self.use_nvdec = use_nvdec
        self.codec = codec.lower()
        self.fallback_to_ffmpeg = fallback_to_ffmpeg

        self.capture = None
        self.thread = None

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame_event = threading.Event()

        self.latest_frame = None
        self.latest_frame_id = -1
        self.latest_capture_time = None

        self.connected = False
        self.reported_fps = 25.0

        self.active_decoder = None

    def _open_gstreamer_nvdec(self):
        """Open RTMP through GStreamer and Jetson NVDEC."""

        if not opencv_has_gstreamer():

            print(
                "❌ This OpenCV build does not support GStreamer"
            )

            return None

        pipeline = build_rtmp_gstreamer_pipeline(
            rtmp_url=self.source,
            codec=self.codec
        )

        print(
            f"🎞️ Opening RTMP with GStreamer/NVDEC "
            f"({self.codec.upper()})"
        )

        cap = cv2.VideoCapture(
            pipeline,
            cv2.CAP_GSTREAMER
        )

        if not cap.isOpened():

            cap.release()

            print(
                "❌ GStreamer/NVDEC pipeline failed to open"
            )

            return None

        self.active_decoder = "GStreamer/NVDEC"

        return cap

    def _open_ffmpeg(self):
        """Open RTMP through the OpenCV FFmpeg backend."""

        print(
            "🎞️ Opening RTMP with FFmpeg decoding"
        )

        cap = cv2.VideoCapture(
            self.source,
            cv2.CAP_FFMPEG
        )

        cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

        if not cap.isOpened():

            cap.release()

            print(
                "❌ FFmpeg RTMP capture failed to open"
            )

            return None

        self.active_decoder = "FFmpeg"

        return cap

    def _open_capture(self):
        """Open RTMP with NVDEC or optional FFmpeg fallback."""

        cap = None

        if self.use_nvdec:

            cap = self._open_gstreamer_nvdec()

            if (
                cap is None
                and self.fallback_to_ffmpeg
            ):

                print(
                    "⚠️ Falling back to FFmpeg decoding"
                )

                cap = self._open_ffmpeg()

        else:

            cap = self._open_ffmpeg()

        if cap is None:
            return None

        reported_fps = cap.get(
            cv2.CAP_PROP_FPS
        )

        if reported_fps and reported_fps > 0:

            self.reported_fps = reported_fps

        print(
            f"✅ Decoder active: {self.active_decoder}"
        )

        return cap

    def start(self):
        """Start the background capture thread."""

        if self.thread is not None:
            return self

        self.stop_event.clear()

        self.thread = threading.Thread(
            target=self._capture_loop,
            name="latest-rtmp-frame-capture",
            daemon=True
        )

        self.thread.start()

        return self

    def _capture_loop(self):
        """Continuously capture and overwrite the newest frame."""

        capture_frame_id = 0

        while not self.stop_event.is_set():

            if self.capture is None:

                self.capture = self._open_capture()

                if self.capture is None:

                    self.connected = False

                    print(
                        "⚠️ Unable to connect to RTMP stream. "
                        "Retrying..."
                    )

                    self.stop_event.wait(
                        self.reconnect_delay
                    )

                    continue

                self.connected = True

                print(
                    "✅ RTMP capture connected"
                )

            ret, frame = self.capture.read()

            if not ret:

                self.connected = False

                print(
                    "⚠️ RTMP connection lost. Reconnecting..."
                )

                self.capture.release()
                self.capture = None

                self.stop_event.wait(
                    self.reconnect_delay
                )

                continue

            capture_time = time.perf_counter()

            with self.lock:

                # The ndarray reference is replaced rather than
                # modified in place. No frame queue is created.
                self.latest_frame = frame
                self.latest_frame_id = capture_frame_id
                self.latest_capture_time = capture_time

            capture_frame_id += 1

            self.frame_event.set()

        if self.capture is not None:

            self.capture.release()
            self.capture = None

        self.connected = False

    def read_latest(
        self,
        last_frame_id=-1,
        timeout=2.0
    ):
        """
        Return only a frame newer than last_frame_id.

        Returns:
            success,
            frame,
            capture_frame_id,
            local_decode_completion_time
        """

        deadline = time.perf_counter() + timeout

        while not self.stop_event.is_set():

            with self.lock:

                if (
                    self.latest_frame is not None
                    and self.latest_frame_id > last_frame_id
                ):

                    return (
                        True,
                        self.latest_frame,
                        self.latest_frame_id,
                        self.latest_capture_time
                    )

            remaining = (
                deadline - time.perf_counter()
            )

            if remaining <= 0:
                break

            self.frame_event.wait(
                min(remaining, 0.05)
            )

            self.frame_event.clear()

        return (
            False,
            None,
            last_frame_id,
            None
        )

    def stop(self):
        """Stop capture and release RTMP resources."""

        self.stop_event.set()
        self.frame_event.set()

        if self.capture is not None:

            self.capture.release()
            self.capture = None

        if self.thread is not None:

            self.thread.join(
                timeout=3.0
            )

            self.thread = None

        with self.lock:

            self.latest_frame = None
            self.latest_frame_id = -1
            self.latest_capture_time = None

        self.connected = False


# ============================================================
# YOLO TENSORRT VIDEO PROCESSOR
# ============================================================

class YOLOTensorRTVideo:

    def __init__(self, model_path):

        if not os.path.isfile(model_path):

            raise FileNotFoundError(
                f"TensorRT engine not found: {model_path}"
            )

        print(
            "🚀 Loading TensorRT engine..."
        )

        self.model = YOLO(
            model_path,
            task="detect"
        )

        print(
            "✅ TensorRT model loaded"
        )

    def run(
        self,
        video_source,
        confidence_threshold=0.60,
        class_filter=None,
        ema_alpha=0.20,
        average_window_seconds=1.0,
        ros2_topic="drone_vehicle_detections",
        ros2_node_name="visdrone_detector",
        save_performance_csv=False,
        csv_path="logs/performance_trt.csv",
        save_output=False,
        output_path="outputs/output_trt.mp4",
        show_debug_window=False,
        flush_interval_frames=30,
        rtmp_reconnect_delay=1.0,
        rtmp_frame_timeout=2.0,
        use_nvdec=True,
        rtmp_codec="h264",
        fallback_to_ffmpeg=True
    ):

        # ----------------------------------------------------
        # Validate settings
        # ----------------------------------------------------

        if not 0.0 <= confidence_threshold <= 1.0:

            raise ValueError(
                "confidence_threshold must be between 0.0 and 1.0"
            )

        if not 0.0 < ema_alpha <= 1.0:

            raise ValueError(
                "ema_alpha must be greater than 0.0 and at most 1.0"
            )

        if average_window_seconds <= 0.0:

            raise ValueError(
                "average_window_seconds must be greater than zero"
            )

        if flush_interval_frames < 1:

            raise ValueError(
                "flush_interval_frames must be at least 1"
            )

        if rtmp_reconnect_delay < 0.0:

            raise ValueError(
                "rtmp_reconnect_delay cannot be negative"
            )

        if rtmp_frame_timeout <= 0.0:

            raise ValueError(
                "rtmp_frame_timeout must be greater than zero"
            )

        if rtmp_codec.lower() not in (
            "h264",
            "avc",
            "h265",
            "hevc"
        ):

            raise ValueError(
                f"Unsupported RTMP codec: {rtmp_codec}"
            )

        if class_filter is not None:

            invalid_classes = [
                class_id
                for class_id in class_filter
                if class_id not in VISDRONE_CLASSES
            ]

            if invalid_classes:

                raise ValueError(
                    f"Invalid VisDrone class IDs: {invalid_classes}"
                )

        if save_performance_csv:

            if not csv_path:

                raise ValueError(
                    "csv_path is required when "
                    "save_performance_csv=True"
                )

            create_parent_directory(
                csv_path
            )

        if save_output:

            if not output_path:

                raise ValueError(
                    "output_path is required when save_output=True"
                )

            create_parent_directory(
                output_path
            )

        # ----------------------------------------------------
        # Source configuration
        # ----------------------------------------------------

        source_text = str(video_source)

        is_rtmp = source_text.lower().startswith(
            (
                "rtmp://",
                "rtmps://"
            )
        )

        print()

        if is_rtmp:

            print(
                f"📡 RTMP stream: {video_source}"
            )

            decoder_name = (
                "GStreamer/NVDEC"
                if use_nvdec
                else "FFmpeg"
            )

            print(
                f"🎞️ Requested RTMP decoder: "
                f"{decoder_name}"
            )

            print(
                f"🎞️ RTMP codec: "
                f"{rtmp_codec.upper()}"
            )

            print(
                f"🔁 FFmpeg fallback: "
                f"{fallback_to_ffmpeg}"
            )

        else:

            print(
                f"🎬 Video file: {video_source}"
            )

        print(
            f"🎯 Confidence threshold: "
            f"{confidence_threshold:.2f}"
        )

        print(
            f"📈 EMA alpha: "
            f"{ema_alpha:.2f}"
        )

        print(
            f"📡 ROS2 topic: enabled ('{ros2_topic}')"
        )

        print(
            f"📊 Performance CSV: "
            f"{'enabled' if save_performance_csv else 'disabled'}"
        )

        print(
            f"💾 Save annotated video: "
            f"{save_output}"
        )

        print(
            f"🖥️ Debug window: "
            f"{show_debug_window}"
        )

        if class_filter is None:

            print(
                "🎯 Class filter: all classes"
            )

        else:

            class_names = [
                VISDRONE_CLASSES[class_id]
                for class_id in class_filter
            ]

            print(
                f"🎯 Class filter: {class_names}"
            )

        # ----------------------------------------------------
        # Open source
        # ----------------------------------------------------

        cap = None
        latest_capture = None

        if is_rtmp:

            latest_capture = LatestFrameCapture(
                source=video_source,
                reconnect_delay=rtmp_reconnect_delay,
                use_nvdec=use_nvdec,
                codec=rtmp_codec,
                fallback_to_ffmpeg=fallback_to_ffmpeg
            ).start()

            fps_input = 25.0

        else:

            cap = cv2.VideoCapture(
                video_source,
                cv2.CAP_FFMPEG
            )

            if not cap.isOpened():

                cap.release()

                raise RuntimeError(
                    f"Unable to open source: {video_source}"
                )

            fps_input = cap.get(
                cv2.CAP_PROP_FPS
            )

            if not fps_input or fps_input <= 0:

                fps_input = 25.0

        # ----------------------------------------------------
        # Runtime state
        # ----------------------------------------------------

        writer = None
        performance_csv = None
        performance_writer = None

        frame_id = 0
        last_capture_frame_id = -1
        total_dropped_frames = 0

        fps_sum = 0.0
        inference_sum_ms = 0.0
        total_sum_ms = 0.0

        decoded_frame_waiting_age_sum_ms = 0.0

        ema_detections = None
        previous_fps = 0.0
        stop_requested = False

        # Rolling window of (perf_counter_time, raw_detections), used to
        # compute average_detections over the trailing average_window_seconds.
        detection_window = deque()
        detection_window_sum = 0

        session_start_timestamp = (
            current_timestamp()
        )

        # ------------------------------------------------
        # ROS 2 setup
        # ------------------------------------------------

        node = None

        rclpy.init()

        node = Node(ros2_node_name)

        detections_publisher = node.create_publisher(
            VehicleDetectionCounts,
            ros2_topic,
            10
        )
        # Field-trial data collection: link/decode status, published
        # alongside the counts topic — see DroneLinkStatus.msg for the
        # decode_latency_ms caveat (frame-staleness proxy, not true
        # decoder latency; there's no instrumentation point inside the
        # actual GStreamer/FFmpeg decode path in this script).
        link_status_publisher = node.create_publisher(
            DroneLinkStatus,
            ros2_topic + '_link_status',
            10
        )
        frames_since_status = 0
        fps_sum_since_status = 0.0

        try:

            # ------------------------------------------------
            # ROS 2 session start log
            # ------------------------------------------------

            node.get_logger().info(
                f"SESSION_START,"
                f"{session_start_timestamp},"
                f"source={video_source},"
                f"confidence={confidence_threshold:.2f},"
                f"classes={class_filter},"
                f"ema_alpha={ema_alpha:.2f},"
                f"requested_nvdec={use_nvdec},"
                f"rtmp_codec={rtmp_codec}"
            )

            # ------------------------------------------------
            # Optional performance CSV
            # ------------------------------------------------

            if save_performance_csv:

                performance_csv = open(
                    csv_path,
                    mode="w",
                    newline="",
                    buffering=8192
                )

                performance_writer = csv.writer(
                    performance_csv
                )

                performance_writer.writerow([
                    "Timestamp",
                    "ProcessedFrame",
                    "CaptureFrame",
                    "RawDetections",
                    "EMADetections",
                    "DecodedFrameWaitingAge(ms)",
                    "DroppedFramesTotal",
                    "Inference(ms)",
                    "TotalProcessing(ms)",
                    "FPS"
                ])

            print(
                "🚀 Running inference..."
            )

            # ------------------------------------------------
            # Main loop
            # ------------------------------------------------

            while True:

                # --------------------------------------------
                # Frame acquisition
                # --------------------------------------------

                if is_rtmp:

                    (
                        ret,
                        frame,
                        capture_frame_id,
                        capture_time
                    ) = latest_capture.read_latest(
                        last_frame_id=(
                            last_capture_frame_id
                        ),
                        timeout=rtmp_frame_timeout
                    )

                    if not ret:

                        if (
                            latest_capture
                            .stop_event
                            .is_set()
                        ):
                            break

                        print(
                            "⚠️ Waiting for a new RTMP frame..."
                        )

                        continue

                    if last_capture_frame_id >= 0:

                        skipped_frames = (
                            capture_frame_id
                            - last_capture_frame_id
                            - 1
                        )

                        if skipped_frames > 0:

                            total_dropped_frames += (
                                skipped_frames
                            )

                    last_capture_frame_id = (
                        capture_frame_id
                    )

                    decoded_frame_waiting_age_ms = (
                        time.perf_counter()
                        - capture_time
                    ) * 1000.0

                else:

                    ret, frame = cap.read()

                    if not ret:
                        break

                    capture_frame_id = frame_id
                    decoded_frame_waiting_age_ms = 0.0

                start_total = time.perf_counter()

                # --------------------------------------------
                # TensorRT inference through Ultralytics
                # --------------------------------------------

                start_inference = (
                    time.perf_counter()
                )

                results = self.model(
                    frame,
                    imgsz=640,
                    conf=confidence_threshold,
                    classes=class_filter,
                    verbose=False
                )[0]

                end_inference = (
                    time.perf_counter()
                )

                # --------------------------------------------
                # Detection count
                # --------------------------------------------

                detections = (
                    len(results.boxes)
                    if results.boxes is not None
                    else 0
                )

                # --------------------------------------------
                # EMA smoothing
                # --------------------------------------------

                if ema_detections is None:

                    ema_detections = float(
                        detections
                    )

                else:

                    ema_detections = (
                        ema_alpha * detections
                        +
                        (1.0 - ema_alpha)
                        * ema_detections
                    )

                smoothed_count = math.ceil(
                    ema_detections
                )

                timestamp = current_timestamp()

                # --------------------------------------------
                # Rolling window average (average_detections)
                # --------------------------------------------

                detection_window.append(
                    (start_total, detections)
                )

                detection_window_sum += detections

                while (
                    detection_window
                    and (
                        start_total
                        - detection_window[0][0]
                    ) > average_window_seconds
                ):

                    _, old_detections = (
                        detection_window.popleft()
                    )

                    detection_window_sum -= old_detections

                average_detections = int(
                    detection_window_sum
                    / len(detection_window)
                    + 0.5
                )

                # --------------------------------------------
                # Primary ROS 2 publish
                # --------------------------------------------

                detection_msg = VehicleDetectionCounts()

                detection_msg.header.stamp = (
                    node.get_clock().now().to_msg()
                )

                detection_msg.log_timestamp = timestamp
                detection_msg.raw_detections = detections
                detection_msg.ema_detections = smoothed_count
                detection_msg.average_detections = average_detections

                detections_publisher.publish(
                    detection_msg
                )

                # --------------------------------------------
                # Annotation
                # --------------------------------------------

                annotated_frame = None

                if (
                    show_debug_window
                    or save_output
                ):

                    annotated_frame = results.plot()

                # --------------------------------------------
                # Debug display
                # --------------------------------------------

                if show_debug_window:

                    cv2.putText(
                        annotated_frame,
                        f"Raw: {detections}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        annotated_frame,
                        f"EMA: {smoothed_count}",
                        (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2
                    )

                    cv2.putText(
                        annotated_frame,
                        f"FPS: {previous_fps:.2f}",
                        (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 255, 0),
                        2
                    )

                    if is_rtmp:

                        cv2.putText(
                            annotated_frame,
                            (
                                "Decoded wait: "
                                f"{decoded_frame_waiting_age_ms:.1f} ms"
                            ),
                            (10, 135),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            (0, 165, 255),
                            2
                        )

                        cv2.putText(
                            annotated_frame,
                            (
                                "Dropped stale: "
                                f"{total_dropped_frames}"
                            ),
                            (10, 170),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            (0, 165, 255),
                            2
                        )

                    cv2.imshow(
                        "YOLO TensorRT Debug",
                        annotated_frame
                    )

                    key = (
                        cv2.waitKey(1)
                        & 0xFF
                    )

                    if key == ord("q"):

                        print(
                            "\n🛑 User requested stop"
                        )

                        stop_requested = True

                # --------------------------------------------
                # Optional annotated video
                # --------------------------------------------

                if save_output:

                    if writer is None:

                        height, width = (
                            annotated_frame.shape[:2]
                        )

                        output_fps = (
                            latest_capture.reported_fps
                            if is_rtmp
                            else fps_input
                        )

                        if (
                            not output_fps
                            or output_fps <= 0
                        ):

                            output_fps = 25.0
                        

                        writer = cv2.VideoWriter(
                            output_path,
                            cv2.VideoWriter_fourcc(
                                *"mp4v"
                            ),
                            output_fps,
                            (width, height)
                        )

                        if not writer.isOpened():

                            raise RuntimeError(
                                "Unable to open output "
                                f"video writer: {output_path}"
                            )

                    writer.write(
                        annotated_frame
                    )

                # --------------------------------------------
                # Timing
                # --------------------------------------------

                end_total = time.perf_counter()

                inference_ms = (
                    end_inference
                    - start_inference
                ) * 1000.0

                total_ms = (
                    end_total
                    - start_total
                ) * 1000.0

                fps = (
                    1000.0 / total_ms
                    if total_ms > 0.0
                    else 0.0
                )

                previous_fps = fps

                # --------------------------------------------
                # Optional performance CSV
                # --------------------------------------------

                if performance_writer is not None:

                    performance_writer.writerow([
                        timestamp,
                        frame_id,
                        capture_frame_id,
                        detections,
                        smoothed_count,
                        f"{decoded_frame_waiting_age_ms:.6f}",
                        total_dropped_frames,
                        f"{inference_ms:.6f}",
                        f"{total_ms:.6f}",
                        f"{fps:.6f}"
                    ])

                fps_sum += fps
                fps_sum_since_status += fps
                frames_since_status += 1

                inference_sum_ms += (
                    inference_ms
                )

                total_sum_ms += total_ms

                decoded_frame_waiting_age_sum_ms += (
                    decoded_frame_waiting_age_ms
                )

                frame_id += 1

                # --------------------------------------------
                # Periodic disk flush
                # --------------------------------------------

                if (
                    frame_id
                    % flush_interval_frames
                    == 0
                ):

                    if performance_csv is not None:

                        performance_csv.flush()

                    link_status_msg = DroneLinkStatus()
                    link_status_msg.header.stamp = node.get_clock().now().to_msg()
                    link_status_msg.connected = (
                        bool(latest_capture.connected)
                        if is_rtmp
                        else True
                    )
                    link_status_msg.fps = (
                        fps_sum_since_status / frames_since_status
                        if frames_since_status > 0
                        else 0.0
                    )
                    link_status_msg.dropped_frames_total = int(total_dropped_frames)
                    link_status_msg.decode_latency_ms = float(decoded_frame_waiting_age_ms)
                    link_status_publisher.publish(link_status_msg)
                    frames_since_status = 0
                    fps_sum_since_status = 0.0

                # --------------------------------------------
                # Console status
                # --------------------------------------------

                if frame_id % 100 == 0:

                    status = (
                        f"Frame {frame_id} | "
                        f"Raw {detections} | "
                        f"EMA {smoothed_count} | "
                        f"Avg {average_detections} | "
                        f"{fps:.2f} FPS | "
                        f"{inference_ms:.2f} ms"
                    )

                    if is_rtmp:

                        status += (
                            " | Decoded wait "
                            f"{decoded_frame_waiting_age_ms:.1f} ms"
                            " | Dropped "
                            f"{total_dropped_frames}"
                        )

                    print(status)

                if stop_requested:
                    break

        except KeyboardInterrupt:

            print(
                "\n🛑 Keyboard interrupt received"
            )

        finally:

            # ------------------------------------------------
            # ROS 2 session end log
            # ------------------------------------------------

            if node is not None:

                session_end_timestamp = (
                    current_timestamp()
                )

                active_decoder = (
                    latest_capture.active_decoder
                    if latest_capture is not None
                    else "file"
                )

                node.get_logger().info(
                    f"SESSION_END,"
                    f"{session_end_timestamp},"
                    f"processed_frames={frame_id},"
                    f"dropped_frames={total_dropped_frames},"
                    f"decoder={active_decoder}"
                )

            # ------------------------------------------------
            # Cleanup
            # ------------------------------------------------

            if cap is not None:
                cap.release()

            if latest_capture is not None:
                latest_capture.stop()

            if writer is not None:
                writer.release()

            if show_debug_window:
                cv2.destroyAllWindows()

            if performance_csv is not None:

                performance_csv.flush()
                performance_csv.close()

            if node is not None:
                node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

        # ----------------------------------------------------
        # Summary
        # ----------------------------------------------------

        average_fps = (
            fps_sum / frame_id
            if frame_id > 0
            else 0.0
        )

        average_inference_ms = (
            inference_sum_ms / frame_id
            if frame_id > 0
            else 0.0
        )

        average_total_ms = (
            total_sum_ms / frame_id
            if frame_id > 0
            else 0.0
        )

        average_decoded_wait_ms = (
            decoded_frame_waiting_age_sum_ms
            / frame_id
            if frame_id > 0
            else 0.0
        )

        print()
        print(
            f"✅ Frames processed: {frame_id}"
        )

        print(
            f"✅ Average FPS: "
            f"{average_fps:.2f}"
        )

        print(
            f"✅ Average inference: "
            f"{average_inference_ms:.2f} ms"
        )

        print(
            f"✅ Average total processing: "
            f"{average_total_ms:.2f} ms"
        )

        if is_rtmp:

            print(
                f"✅ Active decoder: "
                f"{latest_capture.active_decoder}"
            )

            print(
                f"✅ Average decoded-frame wait: "
                f"{average_decoded_wait_ms:.2f} ms"
            )

            print(
                f"✅ Intentionally dropped stale frames: "
                f"{total_dropped_frames}"
            )

        print(
            f"✅ Vehicle detections published on ROS2 topic: "
            f"{ros2_topic}"
        )

        if save_performance_csv:

            print(
                f"✅ Performance CSV saved: "
                f"{csv_path}"
            )

        if save_output:

            print(
                f"✅ Output video saved: "
                f"{output_path}"
            )

    def release(self):
        """Explicitly release the model reference."""

        if hasattr(self, "model"):

            del self.model


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    MODEL_PATH = str(
        SCRIPT_DIR / "models" / "visdrone_orin.engine"
    )

    # --------------------------------------------------------
    # INPUT SOURCE
    # --------------------------------------------------------

    USE_RTMP = True

    LOCAL_VIDEO = str(
        SCRIPT_DIR
        / "videos"
        / "vlc-record-2026-04-20-21h28m35s-20240307_NM_Hasselt_VildersstraatKempischeSteenweg_view1_blurred.mp4-.mp4"
    )

    RTMP_URL = (
        "rtmp://127.0.0.1:1935/live/test" 
    )

    VIDEO_SOURCE = (
        RTMP_URL
        if USE_RTMP
        else LOCAL_VIDEO
    )

    # --------------------------------------------------------
    # RTMP DECODING
    # --------------------------------------------------------

    # True:
    #   Use Jetson GStreamer/NVDEC hardware decoding.
    #
    # False:
    #   Use OpenCV FFmpeg decoding.
    USE_NVDEC = True

    # Use "h264" unless ffprobe identifies HEVC/H.265.
    RTMP_CODEC = "h264"

    # Use FFmpeg automatically if GStreamer/NVDEC cannot open.
    FALLBACK_TO_FFMPEG = True

    # --------------------------------------------------------
    # DETECTION SETTINGS
    # --------------------------------------------------------

    CONFIDENCE_THRESHOLD = 0.45

    # All classes:
    # CLASS_FILTER = None

    # Pedestrian only:
    # CLASS_FILTER = [0]

    # Pedestrian + people:
    # CLASS_FILTER = [0, 1]

    # Vehicles only:
    CLASS_FILTER = [3, 4, 5, 8]

    # Road users:
    # CLASS_FILTER = [0, 1, 2, 3, 4, 5, 8, 9]

    EMA_ALPHA = 1

    # --------------------------------------------------------
    # ROS 2 / AVERAGE WINDOW
    # --------------------------------------------------------

    AVERAGE_WINDOW_SECONDS = 1.0

    ROS2_TOPIC = "drone_vehicle_detections"

    ROS2_NODE_NAME = "visdrone_detector"

    # --------------------------------------------------------
    # OPTIONAL PERFORMANCE CSV
    # --------------------------------------------------------

    SAVE_PERFORMANCE_CSV = True

    CSV_PATH = str(
        SCRIPT_DIR / "logs" / "performance_trt.csv"
    )

    # --------------------------------------------------------
    # OPTIONAL ANNOTATED VIDEO
    # --------------------------------------------------------

    SAVE_OUTPUT = False

    OUTPUT_PATH = str(
        SCRIPT_DIR / "outputs" / "output_trt.mp4"
    )

    # --------------------------------------------------------
    # OPTIONAL DEBUG WINDOW
    # --------------------------------------------------------

    SHOW_DEBUG_WINDOW = False

    # --------------------------------------------------------
    # RUNTIME BEHAVIOR
    # --------------------------------------------------------

    FLUSH_INTERVAL_FRAMES = 30

    RTMP_RECONNECT_DELAY_SECONDS = 1.0

    RTMP_NEW_FRAME_TIMEOUT_SECONDS = 2.0

    # --------------------------------------------------------
    # RUN
    # --------------------------------------------------------

    detector = YOLOTensorRTVideo(
        MODEL_PATH
    )

    try:

        detector.run(
            video_source=VIDEO_SOURCE,
            confidence_threshold=CONFIDENCE_THRESHOLD,
            class_filter=CLASS_FILTER,
            ema_alpha=EMA_ALPHA,
            average_window_seconds=AVERAGE_WINDOW_SECONDS,
            ros2_topic=ROS2_TOPIC,
            ros2_node_name=ROS2_NODE_NAME,
            save_performance_csv=SAVE_PERFORMANCE_CSV,
            csv_path=CSV_PATH,
            save_output=SAVE_OUTPUT,
            output_path=OUTPUT_PATH,
            show_debug_window=SHOW_DEBUG_WINDOW,
            flush_interval_frames=FLUSH_INTERVAL_FRAMES,
            rtmp_reconnect_delay=(
                RTMP_RECONNECT_DELAY_SECONDS
            ),
            rtmp_frame_timeout=(
                RTMP_NEW_FRAME_TIMEOUT_SECONDS
            ),
            use_nvdec=USE_NVDEC,
            rtmp_codec=RTMP_CODEC,
            fallback_to_ffmpeg=FALLBACK_TO_FFMPEG
        )

    finally:

        detector.release()
