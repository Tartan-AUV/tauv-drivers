#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>

#include "imu_driver.h"
#include "utils.h"

using namespace std;
using namespace std::chrono;
using namespace chrono_literals;

int32_t IMUDriver::initialize() {
    for (int attempts = 0; not serialDriver_.isPortOpened() and attempts < 10;
         attempts++) {
        usleep(5'000'000);
        serialDriver_.reconnect();
    }
    if (not serialDriver_.isPortOpened()) {   // still not connected
        cout << "IMU Driver COULD NOT CONNECT\n";
        return 1;
    }

    cout << "IMU Driver initialized\n";
    return 0;
}

void IMUDriver::processIMUData() {
    if (!driver_.tryReadingNewPacket()) {
        return;
    }
    newPackets_++;
    bool error = false;
    if (!driver_.isChecksumGood()) {
        imuCRCErrors_++;
        cout << "CRC error\n";
        error = true;
    } else if (!driver_.isSensorStatusGood()) {
        imuInternalErrors_++;
        cout << "Internal hardware error\n";
        error = true;
    }
    if (not error) {
        validPackets_++;
        angular_velocity_ = driver_.getGyroData();
        specific_force_   = driver_.getAccData();
        logIMUData();
    }
}

static double time_of_first_log = 0;
void IMUDriver::logIMUData() {
    double t = get_time();
    if (first_time_logging_) {   // write header
        cout << "time [s]" << "," << "Valid Packet counter" << ","
             << "Gyro x [rad/s]" << "," << "Gyro y [rad/s]" << ","
             << "Gyro z [rad/s]" << "," << "Accel x [m/s^2]" << ","
             << "Accel y [m/s^2]" << "," << "Accel z [m/s^2]\n";
        first_time_logging_ = false;
        time_of_first_log   = t;
    }

    cout << fixed << setprecision(5);
    cout << t - time_of_first_log << ",";
    cout << validPackets_ << ",";        //
    cout << setprecision(15);            //
    cout << angular_velocity_.x << ","   //
         << angular_velocity_.y << ","   //
         << angular_velocity_.z << ","   //
         << specific_force_.x << ","     //
         << specific_force_.y << ","     //
         << specific_force_.z << endl;
}

// Defining the member functions for IMUDriver
int main() {
    static IMUDriver imuDriver;
    double start           = get_time();
    double last_hearbeat   = -1e99;   // [s]
    double hearbeat_period = 1.0;     // [s]
    while (true) {
        double t       = get_time();
        double elapsed = t - last_hearbeat;
        if (elapsed > hearbeat_period) {
            // cout << setprecision(10) << "HEARTBEAT: " << t - start << endl;
            last_hearbeat = t;
        }
        imuDriver.processIMUData();
    }
}