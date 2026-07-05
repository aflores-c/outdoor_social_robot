/**
 * ndt_map_creator
 *
 * Preprocesses a raw FAST-LIO PCD map into a compact localization map:
 *   1. VoxelGrid downsample  (leaf_size metres, default 0.2 m)
 *   2. StatisticalOutlierRemoval (optional, removes noise)
 *   3. Saves result as binary PCD → used by ndt_localizer as the map file
 *
 * Usage (one-shot; node exits after saving):
 *   ros2 run ndt_localization ndt_map_creator \
 *       --ros-args -p input_path:=/home/cas/fast_lio_map.pcd \
 *                  -p output_path:=/home/cas/ndt_map.pcd
 *
 * Or via launch:
 *   ros2 launch ndt_localization ndt_map_creator.launch.py \
 *       input_path:=/home/cas/fast_lio_map.pcd \
 *       output_path:=/home/cas/ndt_map.pcd
 */

#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

using PointT = pcl::PointXYZI;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ndt_map_creator");

  const auto input_path    = node->declare_parameter<std::string>("input_path", "");
  const auto output_path   = node->declare_parameter<std::string>("output_path", "");
  const auto leaf_size     = node->declare_parameter<double>("leaf_size", 0.2);
  const auto remove_noise  = node->declare_parameter<bool>("remove_outliers", true);
  const auto sor_mean_k    = node->declare_parameter<int>("sor_mean_k", 50);
  const auto sor_std_mul   = node->declare_parameter<double>("sor_std_mul_thresh", 2.0);

  if (input_path.empty() || output_path.empty()) {
    RCLCPP_FATAL(node->get_logger(),
      "Parameters input_path and output_path must both be set. Exiting.");
    rclcpp::shutdown();
    return 1;
  }

  // Load -----------------------------------------------------------------------
  auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
  if (pcl::io::loadPCDFile<PointT>(input_path, *cloud) < 0) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load PCD: %s", input_path.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Loaded %zu points from: %s",
    cloud->size(), input_path.c_str());

  // VoxelGrid downsample -------------------------------------------------------
  {
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(static_cast<float>(leaf_size),
                   static_cast<float>(leaf_size),
                   static_cast<float>(leaf_size));
    vg.setInputCloud(cloud);
    auto tmp = std::make_shared<pcl::PointCloud<PointT>>();
    vg.filter(*tmp);
    RCLCPP_INFO(node->get_logger(),
      "VoxelGrid (%.2f m): %zu → %zu points", leaf_size, cloud->size(), tmp->size());
    cloud = tmp;
  }

  // Statistical outlier removal ------------------------------------------------
  if (remove_noise) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(sor_mean_k);
    sor.setStddevMulThresh(static_cast<float>(sor_std_mul));
    auto tmp = std::make_shared<pcl::PointCloud<PointT>>();
    sor.filter(*tmp);
    RCLCPP_INFO(node->get_logger(),
      "StatOutlierRemoval: %zu → %zu points", cloud->size(), tmp->size());
    cloud = tmp;
  }

  // Save -----------------------------------------------------------------------
  cloud->width  = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  if (pcl::io::savePCDFileBinary(output_path, *cloud) < 0) {
    RCLCPP_FATAL(node->get_logger(), "Failed to save PCD: %s", output_path.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(),
    "NDT map saved (%zu pts) → %s", cloud->size(), output_path.c_str());

  rclcpp::shutdown();
  return 0;
}
