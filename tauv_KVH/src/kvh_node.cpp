#include <pthread.h>
#include <sched.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>

#include "driver.h"
#include "serial_unix.h"

class KvhNode : public rclcpp::Node {
   public:
    KvhNode() : Node("kvh_imu") {
        // Parameters
        port_ = declare_parameter<std::string>("port", "/dev/ttyTHS1");
        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");

        // Covariance: variance = (noise_density)^2 * bandwidth_hz
        // KVH 1775 ARW ~0.05 deg/sqrt(hr) -> ~1.45e-5 rad/s/sqrt(Hz)
        // KVH 1775 accel noise ~50 ug/sqrt(Hz) -> ~4.9e-4 m/s^2/sqrt(Hz)
        // bandwidth = output_rate / 2 = 500 Hz
        const double gyro_nd = declare_parameter<double>("gyro_noise_density", 1.45e-5);
        const double accel_nd = declare_parameter<double>("accel_noise_density", 4.9e-4);
        const double bw = declare_parameter<double>("bandwidth_hz", 50.0);
        gyro_var_ = gyro_nd * gyro_nd * bw;
        accel_var_ = accel_nd * accel_nd * bw;

        // Publisher (depth 20 to absorb 1 kHz bursts)
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("os/sensors/fog_kvh", 20);

        // Construct driver (Driver constructor calls serial.open())
        serial_ = std::make_unique<SerialUnix>(port_);
        driver_ = std::make_unique<Driver>(*serial_);

        if (!serial_->isPortOpened()) {
            RCLCPP_WARN(get_logger(),
                        "Could not open %s at startup — will retry in read loop",
                        port_.c_str());
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
    // -----------------------------------------------------------------------
    // Main read / publish loop — runs in dedicated thread
    // -----------------------------------------------------------------------
    void readLoop() {
        while (!stop_flag_) {
            if (!driver_->tryReadingNewPacket()) {
                // poll() timed out or port not yet open; avoid busy-spin
                if (!serial_->isPortOpened()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }

            if (!driver_->isChecksumGood()) {
                RCLCPP_WARN_THROTTLE(get_logger(),
                                     *get_clock(),
                                     5000,
                                     "KVH CRC error — packet dropped");
                continue;
            }

            if (!driver_->isSensorStatusGood()) {
                RCLCPP_WARN_THROTTLE(get_logger(),
                                     *get_clock(),
                                     5000,
                                     "KVH sensor status byte != 0x77");
                // Still publish — individual axis validity is in the bits;
                // the data is usually still usable.
            }

            auto msg = sensor_msgs::msg::Imu{};
            msg.header.frame_id = frame_id_;
            msg.header.stamp = computeTimestamp(driver_->getTimestampUs());

            const auto gyro = driver_->getGyroData();
            const auto accel = driver_->getAccData();

            msg.angular_velocity.x = gyro.x;
            msg.angular_velocity.y = gyro.y;
            msg.angular_velocity.z = gyro.z;

            msg.linear_acceleration.x = accel.x;
            msg.linear_acceleration.y = accel.y;
            msg.linear_acceleration.z = accel.z;

            // REP-145: orientation_covariance[0] = -1 means "no orientation"
            msg.orientation_covariance[0] = -1.0;

            // Diagonal covariance from datasheet noise density
            msg.angular_velocity_covariance[0] = gyro_var_;
            msg.angular_velocity_covariance[4] = gyro_var_;
            msg.angular_velocity_covariance[8] = gyro_var_;

            msg.linear_acceleration_covariance[0] = accel_var_;
            msg.linear_acceleration_covariance[4] = accel_var_;
            msg.linear_acceleration_covariance[8] = accel_var_;

            imu_pub_->publish(msg);
        }
    }

    // -----------------------------------------------------------------------
    // Timestamp computation using the IMU's own microsecond counter.
    //
    // Strategy: anchor the IMU counter to host time on the first packet, then
    // compute all subsequent stamps as first_host_time + elapsed_imu_us.
    // This removes USB/FTDI jitter without long-term host-clock drift.
    //
    // The UINT32 counter wraps every ~71 minutes; we accumulate a UINT64
    // running total so we handle multiple wraps correctly.
    // -----------------------------------------------------------------------
    rclcpp::Time computeTimestamp(uint32_t imu_us) {
        if (first_packet_) {
            first_host_time_ = now();
            prev_imu_us_ = imu_us;
            total_elapsed_us_ = 0;
            first_packet_ = false;
            return first_host_time_;
        }

        // Delta handles wraparound: if the counter went backwards it wrapped.
        uint32_t delta;
        if (imu_us >= prev_imu_us_) {
            delta = imu_us - prev_imu_us_;
        } else {
            delta = (static_cast<uint64_t>(UINT32_MAX) - prev_imu_us_ + imu_us + 1);
        }
        prev_imu_us_ = imu_us;
        total_elapsed_us_ += delta;

        const int64_t elapsed_ns = static_cast<int64_t>(total_elapsed_us_) * 1000LL;
        return first_host_time_ + rclcpp::Duration::from_nanoseconds(elapsed_ns);
    }

    // Parameters
    std::string port_;
    std::string frame_id_;
    double gyro_var_{};
    double accel_var_{};

    // Driver stack
    std::unique_ptr<SerialUnix> serial_;
    std::unique_ptr<Driver> driver_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Read thread
    std::atomic<bool> stop_flag_{false};
    std::thread read_thread_;

    // Timestamp state
    bool first_packet_{true};
    uint32_t prev_imu_us_{0};
    uint64_t total_elapsed_us_{0};
    rclcpp::Time first_host_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KvhNode>());
    rclcpp::shutdown();
    return 0;
}
