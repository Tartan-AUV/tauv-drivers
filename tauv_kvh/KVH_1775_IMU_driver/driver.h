
#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "EigenWrapper.h"
#include "datagram_parser.h"
#include "serial_driver.h"

class Driver {
   public:
    explicit Driver(SerialDriver &serial_driver);

    /** \brief Return the Accelerometers values
     */
    Vector3 getAccData() const;

    /** \brief Return the Gyroscopes values
     */
    Vector3 getGyroData() const;

    double getAverageTemp() const;
    std::string getDevicePath() const;

    void setDevicePath(const char *newPath);

    bool isChecksumGood() const;
    bool isSensorStatusGood() const;
    uint8_t getInternalMeasurmentCounter() const;
    uint32_t getTimestampUs() const;
    bool tryReadingNewPacket();

    // helpful to reset / force state for unit tests
    void reset();
    void forceState(bool checksum_is_ok, bool no_internal_error);

   private:
    SerialDriver &serial_driver_;
    bool checksum_is_ok_;
    bool no_internal_error_;
    SensorData sensor_data_;
};

uint32_t calculateChecksum(const buffer_iterator_t &begin,
                           const buffer_iterator_t &end);

#endif   // _DRIVER_H_
