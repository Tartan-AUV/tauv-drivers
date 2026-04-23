#include <pthread.h>
#include <sched.h>

#include <atomic>
#include <cstdint>
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
#include "serial_unix.h"

namespace tauv_kvh {

class KvhNode : public rclcpp::Node {
   public:
    KvhNode(const rclcpp::NodeOptions& options) : Node("kvh_node", options) {
        // Parameters
        port_ = declare_parameter<std::string>("port", "/dev/ttyTHS1");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id", "os/base_link");

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
        const tf2::Vector3 GRAVITY_WORLD(0.0, 0.0, 9.80665);

        while (!stop_flag_) {
            if (!driver_->tryReadingNewPacket()) {
                if (!serial_->isPortOpened()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }

            if (!driver_->isChecksumGood() || !driver_->isSensorStatusGood()) {
                continue; // Optionally add your warning throttles back here
            }

            auto stamp = computeTimestamp(driver_->getTimestampUs());
            const auto gyro = driver_->getGyroData();
            const auto accel = driver_->getAccData();

            // ==========================================
            // GRAVITY REMOVAL MATH
            // ==========================================
            tf2::Vector3 accel_no_g(accel.x, accel.y, accel.z);

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
            }

            // ==========================================
            // 1. GYRO PUBLISHER
            // ==========================================
            auto msg_gyro = std::make_unique<sensor_msgs::msg::Imu>();
            msg_gyro->header.frame_id = frame_id_gyro_;
            msg_gyro->header.stamp = stamp;
            msg_gyro->angular_velocity.x = gyro.x;
            msg_gyro->angular_velocity.y = gyro.y;
            msg_gyro->angular_velocity.z = gyro.z;
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

    rclcpp::Time computeTimestamp(uint32_t imu_us) {
        return this->now();
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
};

}  // namespace tauv_kvh

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tauv_kvh::KvhNode)