/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#define ASIO_NO_DEPRECATED
// Prefer standalone Asio if available, otherwise fall back to Boost.Asio.
#if defined(__has_include)
#  if __has_include(<asio.hpp>)
#    include <asio.hpp>
#  elif __has_include(<boost/asio.hpp>)
#    include <boost/asio.hpp>
#    define RM_USE_BOOST_ASIO 1
#  else
#    include <asio.hpp>
#  endif
#else
#  include <asio.hpp>
#endif

#ifdef RM_USE_BOOST_ASIO
namespace asio = boost::asio;
#endif
#include <atomic>
#include <functional>
#include <unordered_map>
#include <vector>

namespace mid360_driver {

    struct Point {
        double timestamp;
        float x, y, z;
        float intensity;
    };

    struct ImuMsg {
        double timestamp;
        float angular_velocity_x;
        float angular_velocity_y;
        float angular_velocity_z;
        float linear_acceleration_x;
        float linear_acceleration_y;
        float linear_acceleration_z;
    };

    struct IpAddressHasher {
        std::size_t operator()(const asio::ip::address &addr) const noexcept;
    };

    class Mid360Driver {
    private:
        std::atomic<bool> is_running = true;
        asio::ip::address host_ip;
        asio::ip::udp::socket receive_pointcloud_socket;
        asio::ip::udp::socket receive_imu_socket;
        std::vector<Point> points;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> delta_time_map;
        std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> on_receive_pointcloud;
        std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> on_receive_imu;

    public:
        Mid360Driver(asio::io_context &io_context,
                     const asio::ip::address &host_ip,
                     const std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> &on_receive_pointcloud,
                     const std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> &on_receive_imu);

        ~Mid360Driver();

        void stop();

        asio::awaitable<void> receive_pointcloud();

        asio::awaitable<void> receive_imu();
    };

}// namespace mid360_driver
