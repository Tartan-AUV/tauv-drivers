#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_

#include <chrono>
#include <cstring>

// for logging
#include <fstream>

#include "EigenWrapper.h"
#include "driver.h"
#include "serial_unix.h"

#define PATH "/dev/ttyUSB0"

typedef std::chrono::time_point<std::chrono::steady_clock> Timestamp_t;

class IMUDriver {
   private:
    SerialUnix serialDriver_{PATH};

    Timestamp_t prevIMUmsgTimestamp_;

    bool unitTesting_;

    std::ofstream imu_data_log;   // for logging
    bool first_time_logging_;
    void logIMUData();

    Vector3 angular_velocity_;
    Vector3 specific_force_;

    uint64_t newPackets_;
    uint64_t imuCRCErrors_;
    uint64_t imuInternalErrors_;
    uint64_t validPackets_;
    uint64_t negativedtCount_;
    uint64_t excessDtCount_;
    bool firstIMUPacket_;

   public:
    Driver driver_{serialDriver_};

    int32_t initialize();

    void processIMUData();

    void printIMUPacket();

    explicit IMUDriver(bool unitTesting = false)
        : unitTesting_{unitTesting}, first_time_logging_{true} {
        serialDriver_.resetPath(PATH);
    }
};

#endif   //_IMU_DRIVER_H_ header
