/**
 * ndt_localizer
 *
 * 3D NDT scan-matching localizer against a pre-built PCD map.
 * Mirrors AMCL's role but uses the VLP-32C instead of a 2D laser.
 *
 * TF output (same contract as AMCL):
 *   map → odom      (published every aligned scan)
 *
 * Topics published:
 *   /localization/pose       PoseWithCovarianceStamped  (map frame)
 *   /localization/map_cloud  PointCloud2  (latched, map for RViz)
 *
 * Topics subscribed:
 *   <scan_topic>   PointCloud2  (default /velodyne_points)
 *   /initialpose   PoseWithCovarianceStamped  (RViz "2D Pose Estimate" button)
 *
 * Requires:
 *   odom → base_frame TF from the robot's odometry (TIAGo wheel odometry or
 *   FAST-LIO odometry). Without it the localizer still works but initial-guess
 *   quality is lower (uses last NDT pose directly).
 *
 * Initial pose:
 *   Set via RViz "2D Pose Estimate", or pass initial_x / initial_y /
 *   initial_yaw launch arguments.  NDT typically converges when the guess is
 *   within ~2 m and ~20° of the true pose.
 */

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZI;
using Cloud  = pcl::PointCloud<PointT>;

// ── Eigen ↔ geometry_msgs helpers ────────────────────────────────────────────

static Eigen::Affine3d tfToEigen(const geometry_msgs::msg::TransformStamped & tf)
{
  const auto & t = tf.transform.translation;
  const auto & r = tf.transform.rotation;
  Eigen::Affine3d A = Eigen::Affine3d::Identity();
  A.translation() << t.x, t.y, t.z;
  A.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
  return A;
}

static Eigen::Affine3d poseToEigen(const geometry_msgs::msg::Pose & p)
{
  Eigen::Affine3d A = Eigen::Affine3d::Identity();
  A.translation() << p.position.x, p.position.y, p.position.z;
  A.linear() = Eigen::Quaterniond(
    p.orientation.w, p.orientation.x,
    p.orientation.y, p.orientation.z).toRotationMatrix();
  return A;
}

static geometry_msgs::msg::TransformStamped eigenToTf(
  const Eigen::Affine3d & A,
  const std::string & parent,
  const std::string & child,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp    = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id  = child;
  Eigen::Vector3d t = A.translation();
  Eigen::Quaterniond q(A.rotation());
  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  return tf;
}

static geometry_msgs::msg::Pose eigenToPose(const Eigen::Affine3d & A)
{
  geometry_msgs::msg::Pose p;
  p.position.x = A.translation().x();
  p.position.y = A.translation().y();
  p.position.z = A.translation().z();
  Eigen::Quaterniond q(A.rotation());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// ── NdtLocalizer ─────────────────────────────────────────────────────────────

class NdtLocalizer : public rclcpp::Node
{
public:
  NdtLocalizer()
  : Node("ndt_localizer"),
    pose_initialized_(false),
    odom_initialized_(false),
    T_map_base_(Eigen::Affine3d::Identity()),
    T_map_odom_(Eigen::Affine3d::Identity())
  {
    // Parameters ---------------------------------------------------------------
    map_path_      = declare_parameter<std::string>("map_path", "");
    map_frame_     = declare_parameter<std::string>("map_frame", "map");
    odom_frame_    = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_    = declare_parameter<std::string>("base_frame", "base_footprint");
    scan_topic_    = declare_parameter<std::string>("scan_topic", "/velodyne_points");
    input_leaf_    = declare_parameter<double>("input_leaf_size", 0.3);
    min_score_     = declare_parameter<double>("min_convergence_score", 0.5);

    // Initial pose from launch args --------------------------------------------
    double ix  = declare_parameter<double>("initial_x",   0.0);
    double iy  = declare_parameter<double>("initial_y",   0.0);
    double iyaw= declare_parameter<double>("initial_yaw", 0.0);

    // NDT configuration --------------------------------------------------------
    double res   = declare_parameter<double>("ndt_resolution",    1.0);
    double step  = declare_parameter<double>("ndt_step_size",     0.1);
    double eps   = declare_parameter<double>("ndt_epsilon",       0.01);
    int    iters = declare_parameter<int>   ("ndt_max_iterations", 35);

    ndt_.setResolution(static_cast<float>(res));
    ndt_.setStepSize(step);
    ndt_.setTransformationEpsilon(eps);
    ndt_.setMaximumIterations(iters);

    // Load map -----------------------------------------------------------------
    if (!load_map()) {
      RCLCPP_FATAL(get_logger(), "Map load failed — shutting down.");
      throw std::runtime_error("Map load failed");
    }

    // TF -----------------------------------------------------------------------
    tf_buffer_    = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Publishers ---------------------------------------------------------------
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose", 10);

    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/map_cloud",
      rclcpp::QoS(1).transient_local());

    // Subscribers --------------------------------------------------------------
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NdtLocalizer::scan_callback, this, std::placeholders::_1));

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&NdtLocalizer::initial_pose_callback, this, std::placeholders::_1));

    // Apply initial pose from launch args if non-zero --------------------------
    if (ix != 0.0 || iy != 0.0 || iyaw != 0.0) {
      T_map_base_ = Eigen::Affine3d::Identity();
      T_map_base_.translation() << ix, iy, 0.0;
      T_map_base_.linear() = Eigen::AngleAxisd(iyaw, Eigen::Vector3d::UnitZ())
                              .toRotationMatrix();
      pose_initialized_ = true;
      RCLCPP_INFO(get_logger(),
        "Initial pose from launch args: x=%.2f  y=%.2f  yaw=%.2f", ix, iy, iyaw);
    }

    // Publish map cloud for RViz -----------------------------------------------
    publish_map_cloud();

    RCLCPP_INFO(get_logger(),
      "NDT Localizer ready.  Map: %s  Scan: %s  Frames: %s → %s → %s",
      map_path_.c_str(), scan_topic_.c_str(),
      map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());

    if (!pose_initialized_) {
      RCLCPP_WARN(get_logger(),
        "No initial pose set. Use RViz '2D Pose Estimate' or pass "
        "initial_x / initial_y / initial_yaw launch args.");
    }
  }

private:
  // ── Map loading ─────────────────────────────────────────────────────────────

  bool load_map()
  {
    if (map_path_.empty()) {
      RCLCPP_FATAL(get_logger(), "map_path parameter is empty.");
      return false;
    }

    map_cloud_ = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile<PointT>(map_path_, *map_cloud_) < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to load map PCD: %s", map_path_.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Map loaded: %zu points from %s",
      map_cloud_->size(), map_path_.c_str());

    ndt_.setInputTarget(map_cloud_);
    return true;
  }

  void publish_map_cloud()
  {
    if (!map_cloud_) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_cloud_, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp    = now();
    map_pub_->publish(msg);
  }

  // ── Initial pose from RViz ──────────────────────────────────────────────────

  void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    T_map_base_       = poseToEigen(msg->pose.pose);
    pose_initialized_ = true;
    odom_initialized_ = false;   // recompute map→odom from scratch
    RCLCPP_INFO(get_logger(), "Initial pose set from /initialpose.");
  }

  // ── Main scan callback ───────────────────────────────────────────────────────

  void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!pose_initialized_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for initial pose — use RViz '2D Pose Estimate' or set "
        "initial_x / initial_y / initial_yaw launch args.");
      return;
    }

    // Convert & downsample input scan ------------------------------------------
    auto scan_raw = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *scan_raw);

    auto scan = std::make_shared<Cloud>();
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(static_cast<float>(input_leaf_),
                   static_cast<float>(input_leaf_),
                   static_cast<float>(input_leaf_));
    vg.setInputCloud(scan_raw);
    vg.filter(*scan);

    if (scan->empty()) {
      RCLCPP_WARN(get_logger(), "Empty scan after downsampling — skipping.");
      return;
    }

    // Build initial guess ──────────────────────────────────────────────────────
    Eigen::Matrix4f guess;

    if (odom_initialized_) {
      // Better guess: compose last map→odom with current odom→base
      try {
        auto tf_odom_base = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.15));
        Eigen::Affine3d T_odom_base = tfToEigen(tf_odom_base);
        guess = (T_map_odom_ * T_odom_base).cast<float>().matrix();
      } catch (const tf2::TransformException &) {
        guess = T_map_base_.cast<float>().matrix();
      }
    } else {
      // First alignment after setting initial pose
      guess = T_map_base_.cast<float>().matrix();
    }

    // NDT alignment ────────────────────────────────────────────────────────────
    ndt_.setInputSource(scan);
    Cloud aligned;
    ndt_.align(aligned, guess);

    if (!ndt_.hasConverged()) {
      RCLCPP_WARN(get_logger(), "NDT did not converge — scan skipped.");
      return;
    }

    double score = ndt_.getTransformationProbability();
    if (score < min_score_) {
      RCLCPP_WARN(get_logger(),
        "NDT score %.4f below threshold %.4f — scan skipped.", score, min_score_);
      return;
    }

    // Update state ─────────────────────────────────────────────────────────────
    T_map_base_ = Eigen::Affine3d(ndt_.getFinalTransformation().cast<double>());

    // Compute map→odom from result and current odom→base
    try {
      auto tf_odom_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.15));
      Eigen::Affine3d T_odom_base = tfToEigen(tf_odom_base);
      T_map_odom_       = T_map_base_ * T_odom_base.inverse();
      odom_initialized_ = true;
    } catch (const tf2::TransformException & ex) {
      // No odometry available — publish map→odom ≈ map→base (no separate odom)
      T_map_odom_ = T_map_base_;
      RCLCPP_DEBUG(get_logger(), "No odom TF (%s) — map→odom set to NDT pose.", ex.what());
    }

    // Publish map→odom TF (Nav2 contract) ─────────────────────────────────────
    auto tf_out = eigenToTf(T_map_odom_, map_frame_, odom_frame_, msg->header.stamp);
    tf_broadcaster_->sendTransform(tf_out);

    // Publish localization pose ────────────────────────────────────────────────
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp    = msg->header.stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.pose       = eigenToPose(T_map_base_);

    // Diagonal covariance from NDT score (higher score → lower uncertainty)
    double cov = 1.0 / (score + 1e-6);
    pose_msg.pose.covariance[0]  = cov;   // x
    pose_msg.pose.covariance[7]  = cov;   // y
    pose_msg.pose.covariance[14] = cov * 4.0;  // z (less constrained)
    pose_msg.pose.covariance[35] = cov;   // yaw
    pose_pub_->publish(pose_msg);

    RCLCPP_DEBUG(get_logger(),
      "NDT aligned: score=%.4f  x=%.2f y=%.2f z=%.2f",
      score,
      T_map_base_.translation().x(),
      T_map_base_.translation().y(),
      T_map_base_.translation().z());
  }

  // ── State ────────────────────────────────────────────────────────────────────

  std::mutex state_mutex_;
  bool pose_initialized_;
  bool odom_initialized_;
  Eigen::Affine3d T_map_base_;   // map  → base_footprint  (from NDT)
  Eigen::Affine3d T_map_odom_;   // map  → odom            (published to Nav2)

  // ── Parameters ───────────────────────────────────────────────────────────────

  std::string map_path_, map_frame_, odom_frame_, base_frame_, scan_topic_;
  double input_leaf_, min_score_;

  // ── PCL ──────────────────────────────────────────────────────────────────────

  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
  Cloud::Ptr map_cloud_;

  // ── ROS ──────────────────────────────────────────────────────────────────────

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NdtLocalizer>());
  rclcpp::shutdown();
  return 0;
}
