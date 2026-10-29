#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "driver.h"
#include "fog_clock.hpp"
#include "serial_unix.h"

namespace tauv_kvh {

class KvhNode : public rclcpp::Node {
   public:
    KvhNode(const rclcpp::NodeOptions& options) : Node("kvh_node", options) {
        // Parameters
        port_ = declare_parameter<std::string>("port", "/dev/ttyTHS1");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id", "os/base_link");
        // false falls back to stamping with host receive time
        use_imu_timestamps_ = declare_parameter<bool>("use_imu_timestamps", true);

        // Gyro corrections, all off by default. Either a startup bias taken at
        // rest (already contains the Earth rate), or a stored sensor bias from
        // scripts/fog_two_position_cal.py together with earth_rate_compensation.
        earth_rate_compensation_ = declare_parameter<bool>("earth_rate_compensation", false);
        const double latitude_deg = declare_parameter<double>("latitude_deg", 40.44);
        earth_rate_up_ = 7.2921159e-5 * std::sin(latitude_deg * M_PI / 180.0);
        startup_bias_seconds_ = declare_parameter<double>("startup_bias_seconds", 0.0);
        startup_bias_max_std_ = declare_parameter<double>("startup_bias_max_std", 8.7e-4);  // rad/s
        const auto stored_bias = declare_parameter<std::vector<double>>("gyro_bias_deg_h", {0.0, 0.0, 0.0});
        if (earth_rate_compensation_ && startup_bias_seconds_ > 0.0) {
            RCLCPP_WARN(get_logger(), "earth_rate_compensation and startup_bias_seconds both set; "
                                      "using startup bias only");
            earth_rate_compensation_ = false;
        }
        if (stored_bias.size() == 3 && startup_bias_seconds_ <= 0.0 &&
            (stored_bias[0] != 0.0 || stored_bias[1] != 0.0 || stored_bias[2] != 0.0)) {
            const double from_deg_h = M_PI / 180.0 / 3600.0;
            gyro_bias_ = tf2::Vector3(stored_bias[0], stored_bias[1], stored_bias[2]) * from_deg_h;
            bias_done_ = true;
            RCLCPP_INFO(get_logger(), "Using stored FOG gyro bias (deg/h, sensor frame): %.2f %.2f %.2f%s",
                        stored_bias[0], stored_bias[1], stored_bias[2],
                        earth_rate_compensation_ ? " + Earth-rate compensation" : "");
        }

        // Independent frames
        frame_id_gyro_ = declare_parameter<std::string>("frame_id_gyro", "imu_link_fog_gyro");
        frame_id_accel_x_ = declare_parameter<std::string>("frame_id_accel_x", "imu_link_fog_accel_x");
        frame_id_accel_y_ = declare_parameter<std::string>("frame_id_accel_y", "imu_link_fog_accel_y");
        frame_id_accel_z_ = declare_parameter<std::string>("frame_id_accel_z", "imu_link_fog_accel_z");

        // Covariance
        const double gyro_nd = declare_parameter<double>("gyro_noise_density", 1.45e-5);
        const double accel_nd = declare_parameter<double>("accel_noise_density", 4.9e-4);
        const double bw = declare_parameter<double>("bandwidth_hz", 50.0);
        gyro_var_ = gyro_nd * gyro_nd * bw;
        accel_var_ = accel_nd * accel_nd * bw;

        // Publishers
        gyro_pub_ = create_publisher<sensor_msgs::msg::Imu>("os/sensors/fog_kvh/gyro", 20);
        accel_x_pub_ = create_publisher<sensor_msgs::msg::Imu>("os/sensors/fog_kvh/accel_x", 20);
        accel_y_pub_ = create_publisher<sensor_msgs::msg::Imu>("os/sensors/fog_kvh/accel_y", 20);
        accel_z_pub_ = create_publisher<sensor_msgs::msg::Imu>("os/sensors/fog_kvh/accel_z", 20);

        // Subscriptions & TF (For Gravity Removal)
        current_odom_base_quat_.setRPY(0, 0, 0); // Default to upright
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                tf2::fromMsg(msg->pose.pose.orientation, current_odom_base_quat_);
            });

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Construct driver
        serial_ = std::make_unique<SerialUnix>(port_);
        driver_ = std::make_unique<Driver>(*serial_);

        if (!serial_->isPortOpened()) {
            RCLCPP_WARN(get_logger(), "Could not open %s at startup — will retry in read loop", port_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "KVH 1775 connected on %s", port_.c_str());
        }

        stop_flag_ = false;
        read_thread_ = std::thread(&KvhNode::readLoop, this);
    }

    ~KvhNode() {
        stop_flag_ = true;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

   private:
    void readLoop() {
        const tf2::Vector3 GRAVITY_WORLD(0.0, 0.0, 9.79615);

        while (!stop_flag_) {
            if (!driver_->tryReadingNewPacket()) {
                if (!serial_->isPortOpened()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }

            if (!driver_->isChecksumGood()) {
                window_crc_errors_++;
                reportStats();
                continue;
            }

            // Every CRC-good packet goes through the clock so sequence gaps
            // only count packets that never arrived.
            const int64_t recv_ns = now().nanoseconds();
            const int64_t imu_stamp_ns = fog_clock_.update(
                driver_->getTimestampUs(), driver_->getInternalMeasurmentCounter(), recv_ns);
            const int64_t latency_ns = recv_ns - imu_stamp_ns;
            window_latency_sum_ns_ += latency_ns;
            window_latency_max_ns_ = std::max(window_latency_max_ns_, latency_ns);
            window_packets_++;

            if (!driver_->isSensorStatusGood()) {
                window_status_errors_++;
                reportStats();
                continue;
            }
            reportStats();

            const rclcpp::Time stamp(use_imu_timestamps_ ? imu_stamp_ns : recv_ns, RCL_ROS_TIME);
            const auto gyro = driver_->getGyroData();
            const auto accel = driver_->getAccData();

            // ==========================================
            // GRAVITY REMOVAL MATH
            // ==========================================
            tf2::Vector3 accel_no_g(accel.x, accel.y, accel.z);
            tf2::Vector3 gyro_out(gyro.x, gyro.y, gyro.z);

            // 1. Fetch static TF once (avoids heavy lookups in the 500Hz loop)
            if (!static_tf_acquired_) {
                try {
                    // Get transform that maps vectors FROM base_link TO imu
                    auto tf_stamped = tf_buffer_->lookupTransform(
                        frame_id_gyro_, base_frame_id_, tf2::TimePointZero);
                    tf2::fromMsg(tf_stamped.transform.rotation, static_base_imu_quat_);
                    static_tf_acquired_ = true;
                    RCLCPP_INFO(get_logger(), "Acquired static TF for gravity projection");
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                        "Waiting for static TF to remove gravity: %s", ex.what());
                }
            }

            if (static_tf_acquired_) {
                tf2::Quaternion q_odom_base;
                {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    q_odom_base = current_odom_base_quat_;
                }

                // q_odom_base rotates vectors FROM base_link TO odom.
                // We need gravity in base_link, so we use the inverse.
                tf2::Vector3 gravity_base = tf2::quatRotate(q_odom_base.inverse(), GRAVITY_WORLD);

                // static_base_imu_quat_ maps vectors FROM base_link TO IMU.
                tf2::Vector3 gravity_imu = tf2::quatRotate(static_base_imu_quat_, gravity_base);

                // Remove the projected gravity from the raw 3D measurement
                accel_no_g.setX(accel.x - gravity_imu.x());
                accel_no_g.setY(accel.y - gravity_imu.y());
                accel_no_g.setZ(accel.z - gravity_imu.z());

                // The gyro also senses Earth's rotation; remove its vertical
                // component (needs only roll/pitch, so unreferenced yaw is fine).
                if (earth_rate_compensation_) {
                    const tf2::Vector3 earth_world(0.0, 0.0, earth_rate_up_);
                    gyro_out -= tf2::quatRotate(static_base_imu_quat_,
                                                tf2::quatRotate(q_odom_base.inverse(), earth_world));
                }
            }

            if (startup_bias_seconds_ > 0.0 && !bias_done_) {
                updateStartupBias(gyro_out);
            } else if (bias_done_) {
                gyro_out -= gyro_bias_;
            }

            // ==========================================
            // 1. GYRO PUBLISHER
            // ==========================================
            auto msg_gyro = std::make_unique<sensor_msgs::msg::Imu>();
            msg_gyro->header.frame_id = frame_id_gyro_;
            msg_gyro->header.stamp = stamp;
            msg_gyro->angular_velocity.x = gyro_out.x();
            msg_gyro->angular_velocity.y = gyro_out.y();
            msg_gyro->angular_velocity.z = gyro_out.z();
            msg_gyro->orientation_covariance[0] = -1.0;
            msg_gyro->linear_acceleration_covariance[0] = -1.0;
            msg_gyro->angular_velocity_covariance[0] = gyro_var_;
            msg_gyro->angular_velocity_covariance[4] = gyro_var_;
            msg_gyro->angular_velocity_covariance[8] = gyro_var_;
            gyro_pub_->publish(std::move(msg_gyro));

            // ==========================================
            // 2. ACCEL X PUBLISHER (Using Gravity-Removed Data)
            // ==========================================
            auto msg_ax = std::make_unique<sensor_msgs::msg::Imu>();
            msg_ax->header.frame_id = frame_id_accel_x_;
            msg_ax->header.stamp = stamp;
            msg_ax->linear_acceleration.x = accel_no_g.x();
            msg_ax->orientation_covariance[0] = -1.0;
            msg_ax->angular_velocity_covariance[0] = -1.0;
            msg_ax->linear_acceleration_covariance[0] = accel_var_;
            msg_ax->linear_acceleration_covariance[4] = 1e9;
            msg_ax->linear_acceleration_covariance[8] = 1e9;
            accel_x_pub_->publish(std::move(msg_ax));

            // ==========================================
            // 3. ACCEL Y PUBLISHER (Using Gravity-Removed Data)
            // ==========================================
            auto msg_ay = std::make_unique<sensor_msgs::msg::Imu>();
            msg_ay->header.frame_id = frame_id_accel_y_;
            msg_ay->header.stamp = stamp;
            msg_ay->linear_acceleration.y = accel_no_g.y();
            msg_ay->orientation_covariance[0] = -1.0;
            msg_ay->angular_velocity_covariance[0] = -1.0;
            msg_ay->linear_acceleration_covariance[0] = 1e9;
            msg_ay->linear_acceleration_covariance[4] = accel_var_;
            msg_ay->linear_acceleration_covariance[8] = 1e9;
            accel_y_pub_->publish(std::move(msg_ay));

            // ==========================================
            // 4. ACCEL Z PUBLISHER (Using Gravity-Removed Data)
            // ==========================================
            auto msg_az = std::make_unique<sensor_msgs::msg::Imu>();
            msg_az->header.frame_id = frame_id_accel_z_;
            msg_az->header.stamp = stamp;
            msg_az->linear_acceleration.z = accel_no_g.z();
            msg_az->orientation_covariance[0] = -1.0;
            msg_az->angular_velocity_covariance[0] = -1.0;
            msg_az->linear_acceleration_covariance[0] = 1e9;
            msg_az->linear_acceleration_covariance[4] = 1e9;
            msg_az->linear_acceleration_covariance[8] = accel_var_;
            accel_z_pub_->publish(std::move(msg_az));
        }
    }

    // Averages the gyro over startup_bias_seconds; restarts the window if the
    // spread shows the vehicle was moving.
    void updateStartupBias(const tf2::Vector3& w) {
        const auto now_steady = std::chrono::steady_clock::now();
        if (bias_n_ == 0) {
            bias_start_ = now_steady;
        }
        bias_sum_ += w;
        bias_sq_ += w * w;
        bias_n_++;
        if (now_steady - bias_start_ < std::chrono::duration<double>(startup_bias_seconds_)) {
            return;
        }

        const tf2::Vector3 mean = bias_sum_ / static_cast<double>(bias_n_);
        const tf2::Vector3 var = bias_sq_ / static_cast<double>(bias_n_) - mean * mean;
        const double max_std = std::sqrt(std::max({var.x(), var.y(), var.z(), 0.0}));
        const double to_deg_h = 180.0 / M_PI * 3600.0;
        if (max_std <= startup_bias_max_std_) {
            gyro_bias_ = mean;
            bias_done_ = true;
            RCLCPP_INFO(get_logger(), "FOG startup gyro bias over %lu samples (deg/h): %.2f %.2f %.2f, max std %.4f deg/s",
                        bias_n_, mean.x() * to_deg_h, mean.y() * to_deg_h, mean.z() * to_deg_h,
                        max_std * 180.0 / M_PI);
        } else {
            RCLCPP_WARN(get_logger(), "FOG startup bias: vehicle moving (max std %.4f deg/s), retrying",
                        max_std * 180.0 / M_PI);
        }
        bias_sum_ = bias_sq_ = tf2::Vector3(0, 0, 0);
        bias_n_ = 0;
    }

    // Every 5 s: WARN if anything went wrong in the window, otherwise an INFO
    // summary once a minute.
    void reportStats() {
        const auto now_steady = std::chrono::steady_clock::now();
        if (now_steady - window_start_ < std::chrono::seconds(5)) {
            return;
        }
        const auto& s = fog_clock_.stats();
        const uint64_t missing = s.lost - reported_.lost;
        const uint64_t anomalies = s.anomalies - reported_.anomalies;
        const uint64_t resyncs = s.resyncs - reported_.resyncs;
        const double secs = std::chrono::duration<double>(now_steady - window_start_).count();
        const double mean_latency_ms =
            window_packets_ ? window_latency_sum_ns_ / 1e6 / window_packets_ : 0.0;
        const bool bad = missing || anomalies || resyncs || window_crc_errors_ || window_status_errors_;

        if (bad || now_steady - last_info_ >= std::chrono::seconds(60)) {
            char line[320];
            std::snprintf(line, sizeof(line),
                "FOG last %.1fs: %lu pkts (%.1f Hz), missing %lu, crc errors %lu, status errors %lu, "
                "timestamp anomalies %lu, offset resyncs %lu, read latency mean %.2f ms max %.2f ms",
                secs, window_packets_, window_packets_ / secs, missing, window_crc_errors_,
                window_status_errors_, anomalies, resyncs, mean_latency_ms, window_latency_max_ns_ / 1e6);
            if (bad) {
                RCLCPP_WARN(get_logger(), "%s", line);
            } else {
                RCLCPP_INFO(get_logger(), "%s", line);
            }
            last_info_ = now_steady;
        }

        reported_ = s;
        window_start_ = now_steady;
        window_packets_ = window_crc_errors_ = window_status_errors_ = 0;
        window_latency_sum_ns_ = window_latency_max_ns_ = 0;
    }

    // Parameters
    std::string port_;
    std::string odom_topic_;
    std::string base_frame_id_;
    std::string frame_id_gyro_;
    std::string frame_id_accel_x_;
    std::string frame_id_accel_y_;
    std::string frame_id_accel_z_;
    double gyro_var_{};
    double accel_var_{};

    // Driver stack
    std::unique_ptr<SerialUnix> serial_;
    std::unique_ptr<Driver> driver_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gyro_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_x_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_y_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_z_pub_;

    // TF and Gravity Removal State
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::mutex odom_mutex_;
    tf2::Quaternion current_odom_base_quat_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Quaternion static_base_imu_quat_;
    bool static_tf_acquired_{false};

    // Read thread
    std::atomic<bool> stop_flag_{false};
    std::thread read_thread_;

    // Timestamps and health counters (read thread only)
    bool use_imu_timestamps_{true};
    FogClock fog_clock_;
    FogClock::Stats reported_;
    std::chrono::steady_clock::time_point window_start_{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point last_info_{std::chrono::steady_clock::now()};
    uint64_t window_packets_{0};
    uint64_t window_crc_errors_{0};
    uint64_t window_status_errors_{0};
    int64_t window_latency_sum_ns_{0};
    int64_t window_latency_max_ns_{0};

    // Gyro corrections (read thread only)
    bool earth_rate_compensation_{false};
    double earth_rate_up_{0.0};
    double startup_bias_seconds_{0.0};
    double startup_bias_max_std_{0.0};
    bool bias_done_{false};
    tf2::Vector3 gyro_bias_{0, 0, 0};
    tf2::Vector3 bias_sum_{0, 0, 0};
    tf2::Vector3 bias_sq_{0, 0, 0};
    uint64_t bias_n_{0};
    std::chrono::steady_clock::time_point bias_start_;
};

}  // namespace tauv_kvh

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tauv_kvh::KvhNode)