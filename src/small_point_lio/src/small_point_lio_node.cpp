/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "lidar_adapter/livox_lidar.h"
#include "lidar_adapter/unitree_lidar.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

namespace small_point_lio {

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string lidar_type = declare_parameter<std::string>("lidar_type");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        bool save_pcd = declare_parameter<bool>("save_pcd");

        // Compatibility with point_lio + loam_interface pipeline
        // - Publish nav_msgs/Odometry on `state_estimation_topic` (default: aft_mapped_to_init)
        // - Publish PointCloud2 on `registered_scan_topic` (default: cloud_registered)
        // - Use `world_frame` (default: camera_init) and `body_frame` (default: body)
        const std::string state_estimation_topic =
            declare_parameter<std::string>("state_estimation_topic", "aft_mapped_to_init");
        const std::string registered_scan_topic =
            declare_parameter<std::string>("registered_scan_topic", "cloud_registered");
        const std::string world_frame = declare_parameter<std::string>("world_frame", "camera_init");
        const std::string body_frame = declare_parameter<std::string>("body_frame", "body");
        const bool publish_tf = declare_parameter<bool>("publish_tf", false);

        // Publish-side point filtering (only affects published PointCloud2, not internal estimation)
        // This helps reduce near-vehicle dense points in RViz and avoids propagating NaN/Inf points downstream.
        const double publish_min_distance = declare_parameter<double>("publish_min_distance", 0.0);
        const double publish_max_distance = declare_parameter<double>("publish_max_distance", 0.0);
        const double publish_min_distance_sq = publish_min_distance > 0.0 ? publish_min_distance * publish_min_distance : 0.0;
        const double publish_max_distance_sq = publish_max_distance > 0.0 ? publish_max_distance * publish_max_distance : 0.0;

        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);

        // Note: do NOT use absolute topic names here (leading '/'), to support multi-robot namespaces.
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>(state_estimation_topic, 1000);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(registered_scan_topic, 1000);
        if (publish_tf) {
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, world_frame](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    bool save_pcd_enabled = false;
                    (void)get_parameter("save_pcd", save_pcd_enabled);
                    if (!save_pcd_enabled) {
                        res->success = false;
                        res->message = "pcd save is disabled (set parameter save_pcd:=true)";
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "%s", res->message.c_str());
                        return;
                    }

                    const std::string output_path = ROOT_DIR + "/pcd/scan.pcd";
                    res->success = true;
                    res->message = "pcd saving started: " + output_path;
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "waiting for pcd saving ...");

                    std::shared_ptr<std::vector<Eigen::Vector3f>> pointcloud_to_save_copy;
                    {
                        std::lock_guard<std::mutex> lock(pointcloud_to_save_mutex);
                        pointcloud_to_save_copy = std::make_shared<std::vector<Eigen::Vector3f>>(pointcloud_to_save);
                    }
                    std::thread([this, pointcloud_to_save_copy, world_frame]() {
                        voxelgrid_sampling::VoxelgridSampling downsampler;
                        std::vector<Eigen::Vector3f> downsampled;
                        downsampler.voxelgrid_sampling_omp(*pointcloud_to_save_copy, downsampled, 0.02);
                        pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
                        pcl_pointcloud.header.frame_id = world_frame;
                        pcl_pointcloud.header.stamp = static_cast<uint64_t>(last_odometry.timestamp * 1e6);
                        pcl_pointcloud.points.reserve(downsampled.size());
                        for (const auto &point: downsampled) {
                            pcl::PointXYZI new_point;
                            new_point.x = point.x();
                            new_point.y = point.y();
                            new_point.z = point.z();
                            new_point.intensity = 0.0f;
                            pcl_pointcloud.points.push_back(new_point);
                        }
                        pcl_pointcloud.width = pcl_pointcloud.points.size();
                        pcl_pointcloud.height = 1;
                        pcl_pointcloud.is_dense = true;
                        pcl::PCDWriter writer;
                        writer.writeBinary(ROOT_DIR + "/pcd/scan.pcd", pcl_pointcloud);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "save pcd success: %s", (ROOT_DIR + "/pcd/scan.pcd").c_str());
                    }).detach();
                });
        small_point_lio->set_odometry_callback([this, world_frame, body_frame](const common::Odometry &odometry) {
            last_odometry = odometry;

            builtin_interfaces::msg::Time time_msg;
            time_msg.sec = std::floor(odometry.timestamp);
            time_msg.nanosec = static_cast<uint32_t>((odometry.timestamp - time_msg.sec) * 1e9);

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = time_msg;
            odometry_msg.header.frame_id = world_frame;
            odometry_msg.child_frame_id = body_frame;
            odometry_msg.pose.pose.position.x = odometry.position.x();
            odometry_msg.pose.pose.position.y = odometry.position.y();
            odometry_msg.pose.pose.position.z = odometry.position.z();
            odometry_msg.pose.pose.orientation.x = odometry.orientation.x();
            odometry_msg.pose.pose.orientation.y = odometry.orientation.y();
            odometry_msg.pose.pose.orientation.z = odometry.orientation.z();
            odometry_msg.pose.pose.orientation.w = odometry.orientation.w();

            // TODO it is lidar_odom->lidar_frame, we need to transform it to odom->base_link
            // odometry_msg.twist.twist.linear.x = odometry.velocity.x();
            // odometry_msg.twist.twist.linear.y = odometry.velocity.y();
            // odometry_msg.twist.twist.linear.z = odometry.velocity.z();
            // odometry_msg.twist.twist.angular.x = odometry.angular_velocity.x();
            // odometry_msg.twist.twist.angular.y = odometry.angular_velocity.y();
            // odometry_msg.twist.twist.angular.z = odometry.angular_velocity.z();

            if (tf_broadcaster) {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = time_msg;
                transform_stamped.header.frame_id = world_frame;
                transform_stamped.child_frame_id = body_frame;
                transform_stamped.transform.translation.x = odometry.position.x();
                transform_stamped.transform.translation.y = odometry.position.y();
                transform_stamped.transform.translation.z = odometry.position.z();
                transform_stamped.transform.rotation.x = odometry.orientation.x();
                transform_stamped.transform.rotation.y = odometry.orientation.y();
                transform_stamped.transform.rotation.z = odometry.orientation.z();
                transform_stamped.transform.rotation.w = odometry.orientation.w();
                tf_broadcaster->sendTransform(transform_stamped);
            }
            odometry_publisher->publish(odometry_msg);
        });
        small_point_lio->set_pointcloud_callback([this, save_pcd, world_frame, publish_min_distance_sq, publish_max_distance_sq](const std::vector<Eigen::Vector3f> &pointcloud) {
            if (pointcloud_publisher->get_subscription_count() > 0) {
                builtin_interfaces::msg::Time time_msg;
                time_msg.sec = std::floor(last_odometry.timestamp);
                time_msg.nanosec = static_cast<uint32_t>((last_odometry.timestamp - time_msg.sec) * 1e9);

                pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
                pcl_pointcloud.points.reserve(pointcloud.size());
                for (const auto &point: pointcloud) {
                    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
                        continue;
                    }
                    const double dist_sq = static_cast<double>(point.x()) * point.x() +
                                           static_cast<double>(point.y()) * point.y() +
                                           static_cast<double>(point.z()) * point.z();
                    if (!std::isfinite(dist_sq)) {
                        continue;
                    }
                    if (publish_min_distance_sq > 0.0 && dist_sq < publish_min_distance_sq) {
                        continue;
                    }
                    if (publish_max_distance_sq > 0.0 && dist_sq > publish_max_distance_sq) {
                        continue;
                    }
                    pcl::PointXYZI new_point;
                    new_point.x = point.x();
                    new_point.y = point.y();
                    new_point.z = point.z();
                    new_point.intensity = 0.0f;
                    pcl_pointcloud.points.push_back(new_point);
                }
                pcl_pointcloud.width = pcl_pointcloud.points.size();
                pcl_pointcloud.height = 1;
                pcl_pointcloud.is_dense = true;
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(pcl_pointcloud, msg);
                msg.header.stamp = time_msg;
                msg.header.frame_id = world_frame;
                pointcloud_publisher->publish(msg);
            }
            if (save_pcd) {
                std::lock_guard<std::mutex> lock(pointcloud_to_save_mutex);
                pointcloud_to_save.insert(pointcloud_to_save.end(), pointcloud.begin(), pointcloud.end());
            }
        });
        if (lidar_type == "livox") {
#ifdef HAVE_LIVOX_DRIVER
            lidar_adapter = std::make_unique<LivoxLidarAdapter>();
#else
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Livox driver requested but not available!");
            rclcpp::shutdown();
            return;
#endif
        } else if (lidar_type == "unilidar") {
            lidar_adapter = std::make_unique<UnilidarAdapter>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknwon lidar type");
            rclcpp::shutdown();
            return;
        }
        lidar_adapter->setup_subscription(this, lidar_topic, [this](const std::vector<common::Point> &pointcloud) {
            small_point_lio->on_point_cloud_callback(pointcloud);
            small_point_lio->handle_once();
        });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
