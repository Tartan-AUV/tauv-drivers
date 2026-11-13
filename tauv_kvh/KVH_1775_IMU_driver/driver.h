
#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "EigenWrapper.h"
#include "datagram_parser.h"
#include "serial_driver.h"

struct PacketStatistics {
    uint64_t valid_packets = 0;
    uint64_t crc_failures = 0;  // Header-aligned candidates, not discarded byte windows.
    uint64_t status_failures = 0;
    uint64_t sequence_gaps = 0;  // Modulo-128 lower bound; not an exact count after long outages.
    uint64_t duplicate_packets = 0;
    uint64_t discarded_bytes = 0;
};

class Driver {
   public:
    explicit Driver(SerialDriver &serial_driver);

    /** \brief Return the Accelerometers values
     */
    Vector3 getAccData() const;

    /** \brief Return the Gyroscopes values
     */
    Vector3 getGyroData() const;

    const SensorData& data() const { return sensor_data_; }
    const PacketStatistics& statistics() const { return statistics_; }
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
    PacketStatistics statistics_;
    bool parseBufferedPacket();
    bool have_sequence_ = false;
    uint8_t last_sequence_ = 0;
    uint64_t connection_ = 0;
};

uint32_t calculateChecksum(const buffer_iterator_t &begin,
                           const buffer_iterator_t &end);

#endif   // _DRIVER_H_
