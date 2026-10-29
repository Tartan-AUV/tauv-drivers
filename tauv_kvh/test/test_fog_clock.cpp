#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "fog_clock.hpp"

using tauv_kvh::FogClock;

namespace {

constexpr int64_t kPeriodNs = 2'000'000;
constexpr int64_t kTransitNs = 868'000;
constexpr uint32_t kWrapUs = 64585975;

struct SimOptions {
    double seconds = 30.0;
    double drift_ppm = 0.0;          // host clock rate relative to IMU clock
    uint32_t counter_start_us = 12'345'678;
    double drop_prob = 0.0;
    int64_t stall_every_ns = 0;      // read thread blocked for stall_ns every stall_every_ns
    int64_t stall_ns = 0;
    int64_t reset_counter_at_ns = -1;   // IMU counter jumps to 0 at this time
    int64_t host_step_at_ns = -1;       // host clock jumps forward at this time
    int64_t host_step_ns = 0;
    int64_t outage_at_ns = -1;          // no packets for outage_ns
    int64_t outage_ns = 0;
    uint32_t sim_wrap_us = kWrapUs;
};

struct Sample {
    int64_t true_ns;   // host-clock time the sample was taken
    int64_t recv_ns;
    int64_t stamp_ns;
};

// Generates the packet stream a host would see and feeds it through FogClock.
std::vector<Sample> simulate(FogClock& clock, const SimOptions& o) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int64_t> jitter(0, 200'000);   // 0-200 us read delay
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    const int64_t t0 = 1'700'000'000'000'000'000LL;
    const auto n = static_cast<int64_t>(o.seconds * 1e9 / kPeriodNs);
    std::vector<Sample> out;
    int64_t counter_base = o.counter_start_us;
    int64_t reset_index = -1;

    for (int64_t k = 0; k < n; ++k) {
        const int64_t imu_ns = k * kPeriodNs;
        if (o.reset_counter_at_ns >= 0 && reset_index < 0 && imu_ns >= o.reset_counter_at_ns) {
            reset_index = k;
        }
        int64_t counter = reset_index >= 0 ? (k - reset_index) * 2000 : counter_base + k * 2000;
        const auto imu_us = static_cast<uint32_t>(counter % o.sim_wrap_us);
        const auto seq = static_cast<uint8_t>(k & 0x7F);

        if (o.drop_prob > 0 && uni(rng) < o.drop_prob) continue;
        if (o.outage_at_ns >= 0 && imu_ns >= o.outage_at_ns && imu_ns < o.outage_at_ns + o.outage_ns)
            continue;

        int64_t true_ns = t0 + static_cast<int64_t>(imu_ns * (1.0 + o.drift_ppm * 1e-6));
        if (o.host_step_at_ns >= 0 && imu_ns >= o.host_step_at_ns) true_ns += o.host_step_ns;

        int64_t recv_ns = true_ns + kTransitNs + jitter(rng);
        if (o.stall_every_ns > 0) {
            // Packets arriving during a stall are read when it ends.
            const int64_t phase = imu_ns % o.stall_every_ns;
            if (phase < o.stall_ns) recv_ns = std::max(recv_ns, true_ns - phase + o.stall_ns);
        }
        out.push_back({true_ns, recv_ns, clock.update(imu_us, seq, recv_ns)});
    }
    return out;
}

// Max |stamp - true| over samples at or after `from_ns` past the start.
double maxErrorUs(const std::vector<Sample>& s, int64_t from_ns, int64_t to_ns = INT64_MAX) {
    double worst = 0;
    const int64_t start = s.front().true_ns;
    for (const auto& x : s) {
        const int64_t t = x.true_ns - start;
        if (t < from_ns || t >= to_ns) continue;
        worst = std::max(worst, std::abs(static_cast<double>(x.stamp_ns - x.true_ns)) / 1e3);
    }
    return worst;
}

void expectMonotonicAndNotFuture(const std::vector<Sample>& s) {
    for (size_t i = 0; i < s.size(); ++i) {
        ASSERT_LE(s[i].stamp_ns, s[i].recv_ns) << "stamp in the future at " << i;
        if (i > 0) {
            ASSERT_GT(s[i].stamp_ns, s[i - 1].stamp_ns) << "non-monotonic at " << i;
        }
    }
}

}  // namespace

TEST(FogClock, WrapsEvery64sWithoutJumping) {
    FogClock clock;
    SimOptions o;
    o.seconds = 200;   // three wraps
    auto s = simulate(clock, o);
    expectMonotonicAndNotFuture(s);
    EXPECT_LT(maxErrorUs(s, 1'000'000'000), 100.0);
    EXPECT_EQ(clock.stats().anomalies, 0u);
    EXPECT_EQ(clock.stats().lost, 0u);
}

// What the old code did: assume a 2^32 wrap. Stamps leap ~70 min ahead.
TEST(FogClock, FullRangeWrapAssumptionIsWrong) {
    FogClock::Config cfg;
    cfg.wrap_us = 0xFFFFFFFFu;
    FogClock clock(cfg);
    SimOptions o;
    o.seconds = 70;
    auto s = simulate(clock, o);
    // The sequence check catches the bad step instead of jumping 70 minutes.
    EXPECT_EQ(clock.stats().anomalies, 1u);
    EXPECT_LT(maxErrorUs(s, 1'000'000'000), 300.0);
}

TEST(FogClock, FollowsClockDriftBothWays) {
    for (double ppm : {-150.0, 150.0}) {
        FogClock clock;
        SimOptions o;
        o.seconds = 150;
        o.drift_ppm = ppm;
        auto s = simulate(clock, o);
        expectMonotonicAndNotFuture(s);
        EXPECT_LT(maxErrorUs(s, 1'000'000'000), 150.0) << ppm << " ppm";
    }
}

TEST(FogClock, CountsLostPackets) {
    FogClock clock;
    SimOptions o;
    o.seconds = 60;
    o.drop_prob = 0.01;
    auto s = simulate(clock, o);
    // Drops before the first or after the last received packet are invisible.
    const int64_t span = (s.back().true_ns - s.front().true_ns) / kPeriodNs + 1;
    const auto expected = static_cast<uint64_t>(span) - s.size();
    EXPECT_EQ(clock.stats().lost, expected);
    EXPECT_EQ(clock.stats().anomalies, 0u);
    EXPECT_LT(maxErrorUs(s, 1'000'000'000), 100.0);
}

// The read thread stalling (load, scheduling) must not move the stamps.
TEST(FogClock, ReadStallsDoNotShiftStamps) {
    FogClock clock;
    SimOptions o;
    o.seconds = 60;
    o.stall_every_ns = 1'000'000'000;
    o.stall_ns = 40'000'000;
    auto s = simulate(clock, o);
    expectMonotonicAndNotFuture(s);
    EXPECT_LT(maxErrorUs(s, 1'000'000'000), 150.0);
}

TEST(FogClock, RecoversFromCounterReset) {
    FogClock clock;
    SimOptions o;
    o.seconds = 30;
    o.reset_counter_at_ns = 10'000'000'000;
    auto s = simulate(clock, o);
    expectMonotonicAndNotFuture(s);
    EXPECT_EQ(clock.stats().anomalies, 1u);
    EXPECT_LT(maxErrorUs(s, 11'000'000'000), 150.0);
}

TEST(FogClock, RecoversFromHostClockStepForward) {
    FogClock clock;
    SimOptions o;
    o.seconds = 30;
    o.host_step_at_ns = 10'000'000'000;
    o.host_step_ns = 200'000'000;
    auto s = simulate(clock, o);
    EXPECT_EQ(clock.stats().resyncs, 1u);
    EXPECT_LT(maxErrorUs(s, 12'000'000'000), 150.0);
}

TEST(FogClock, RecoversFromHostClockStepBackward) {
    FogClock clock;
    SimOptions o;
    o.seconds = 30;
    o.host_step_at_ns = 10'000'000'000;
    o.host_step_ns = -200'000'000;
    auto s = simulate(clock, o);
    // Stamps are held monotonic, so they catch up once real time passes the last stamp.
    EXPECT_LT(maxErrorUs(s, 11'000'000'000), 150.0);
}

TEST(FogClock, HandlesOutageLongerThanSequenceWrap) {
    FogClock clock;
    SimOptions o;
    o.seconds = 30;
    o.outage_at_ns = 10'000'000'000;
    o.outage_ns = 3'000'000'000;   // 1500 packets, beyond the 128-packet sequence wrap
    auto s = simulate(clock, o);
    expectMonotonicAndNotFuture(s);
    EXPECT_EQ(clock.stats().anomalies, 0u);
    EXPECT_EQ(clock.stats().lost, 1500u);
    // The first packet after the gap re-anchors with its own read delay (<= 200 us here).
    EXPECT_LT(maxErrorUs(s, 1'000'000'000), 250.0);
}

// A unit whose wrap differs slightly from our constant degrades to a small,
// counted re-anchor at each wrap instead of a big jump.
TEST(FogClock, WrongWrapConstantStaysBounded) {
    FogClock clock;
    SimOptions o;
    o.seconds = 200;
    o.sim_wrap_us = kWrapUs - 5000;
    auto s = simulate(clock, o);
    expectMonotonicAndNotFuture(s);
    EXPECT_EQ(clock.stats().anomalies, 3u);
    EXPECT_LT(maxErrorUs(s, 1'000'000'000), 400.0);
}
