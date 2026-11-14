#ifndef TAUV_KVH_FOG_CLOCK_HPP
#define TAUV_KVH_FOG_CLOCK_HPP

#include <algorithm>
#include <cstdint>
#include <cstdlib>

namespace tauv_kvh {

// Converts the KVH 1775's on-board timestamp into host time.
//
// The Format B timestamp is in microseconds but does NOT wrap at 2^32: it
// wraps every 64585975 us (~64.6 s, i.e. 2^32 ticks of the ~66.5 MHz internal
// clock; measured on our unit, identical across wraps). Treating it as a full
// 32-bit counter makes every stamp jump ~70 min into the future after the
// first wrap, and robot_localization then holds those measurements in its
// queue forever.
//
// stamp = imu_elapsed + offset - transit. The offset is tracked as the minimum
// of (receive time - imu_elapsed): read delays only ever make that difference
// larger, so the minimum comes from the packet read with the least delay. The
// estimate may rise at up to leak_ppm so it can follow drift between the IMU
// clock and the host clock (chrony steers the host clock by tens of ppm).
class FogClock {
   public:
    struct Config {
        uint32_t wrap_us = 64585975;
        uint32_t period_us = 2000;                     // 1 / data rate (500 Hz)
        int64_t transit_ns = 868'000;                  // one 40-byte datagram at 460800 baud
        double leak_ppm = 500.0;                       // max rise rate of the offset estimate
        int64_t resync_threshold_ns = 50'000'000;      // offset jump that, if it persists...
        int64_t resync_hold_ns = 1'000'000'000;        // ...this long, forces a resync
    };

    struct Stats {
        uint64_t packets = 0;
        uint64_t lost = 0;        // packets missing according to the sequence number
        uint64_t anomalies = 0;   // counter steps inconsistent with sequence number and host time
        uint64_t resyncs = 0;     // offset estimate reset after a persistent jump
    };

    FogClock() : FogClock(Config{}) {}
    explicit FogClock(const Config& cfg) : cfg_(cfg) {}

    // Host-time stamp (ns) for a packet with counter `imu_us` and sequence
    // number `seq` (0-127) that finished arriving at host time `recv_ns`.
    int64_t update(uint32_t imu_us, uint8_t seq, int64_t recv_ns) {
        stats_.packets++;

        if (!initialized_) {
            initialized_ = true;
            elapsed_ns_ = 0;
            offset_ns_ = recv_ns;
        } else {
            const int64_t step_us = counterStepUs(imu_us, seq, recv_ns);
            elapsed_ns_ += step_us * 1000;

            const int64_t observed = recv_ns - elapsed_ns_;
            const auto leak = static_cast<int64_t>(
                static_cast<double>(recv_ns - prev_recv_ns_) * cfg_.leak_ppm * 1e-6);
            offset_ns_ = std::min(observed, offset_ns_ + leak);
            checkResync(observed, recv_ns);
        }

        prev_imu_us_ = imu_us;
        prev_seq_ = seq;
        prev_recv_ns_ = recv_ns;

        int64_t stamp = elapsed_ns_ + offset_ns_ - cfg_.transit_ns;
        if (has_stamp_ && stamp <= last_stamp_ns_) {
            stamp = last_stamp_ns_ + 1000;
        }
        has_stamp_ = true;
        last_stamp_ns_ = stamp;
        return stamp;
    }

    const Stats& stats() const { return stats_; }

   private:
    // Microseconds of IMU time since the previous packet.
    int64_t counterStepUs(uint32_t imu_us, uint8_t seq, int64_t recv_ns) {
        const int64_t raw_us = imu_us >= prev_imu_us_
                                   ? static_cast<int64_t>(imu_us) - prev_imu_us_
                                   : static_cast<int64_t>(imu_us) + cfg_.wrap_us - prev_imu_us_;
        int64_t seq_gap = (seq - prev_seq_) & 0x7F;
        if (seq_gap == 0) {
            seq_gap = 128;
        }
        const int64_t period = cfg_.period_us;
        const int64_t host_dt_us = (recv_ns - prev_recv_ns_) / 1000;

        // Normal case: counter agrees with the sequence number.
        if (std::llabs(raw_us - seq_gap * period) <= period / 2) {
            stats_.lost += seq_gap - 1;
            return raw_us;
        }
        // Outage longer than the 128-packet sequence wrap: trust the counter
        // if it roughly agrees with host time.
        if (host_dt_us > 100 * period &&
            std::llabs(raw_us - host_dt_us) < 20'000 + host_dt_us / 100) {
            stats_.lost += std::max<int64_t>(0, (raw_us + period / 2) / period - 1);
            return raw_us;
        }
        // Counter jumped (e.g. IMU power-cycled): step by the sequence gap and
        // re-anchor the offset on this packet.
        stats_.anomalies++;
        stats_.lost += seq_gap - 1;
        const int64_t step_us = seq_gap * period;
        offset_ns_ = recv_ns - (elapsed_ns_ + step_us * 1000);
        return step_us;
    }

    // A persistent jump up in (recv - elapsed) is a host clock step forward
    // (or the IMU clock stalling); the leak alone would take minutes to follow.
    void checkResync(int64_t observed, int64_t recv_ns) {
        if (observed - offset_ns_ <= cfg_.resync_threshold_ns) {
            pending_ = false;
            return;
        }
        if (!pending_) {
            pending_ = true;
            pending_since_ns_ = recv_ns;
            pending_min_ns_ = observed;
            return;
        }
        pending_min_ns_ = std::min(pending_min_ns_, observed);
        if (recv_ns - pending_since_ns_ >= cfg_.resync_hold_ns) {
            offset_ns_ = pending_min_ns_;
            stats_.resyncs++;
            pending_ = false;
        }
    }

    Config cfg_;
    Stats stats_;

    bool initialized_{false};
    uint32_t prev_imu_us_{0};
    uint8_t prev_seq_{0};
    int64_t prev_recv_ns_{0};
    int64_t elapsed_ns_{0};
    int64_t offset_ns_{0};

    bool has_stamp_{false};
    int64_t last_stamp_ns_{0};

    bool pending_{false};
    int64_t pending_since_ns_{0};
    int64_t pending_min_ns_{0};
};

}  // namespace tauv_kvh

#endif  // TAUV_KVH_FOG_CLOCK_HPP
