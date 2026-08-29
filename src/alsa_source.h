#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

namespace sendspin { class SourceRole; }

class AlsaSource {
public:
    AlsaSource(sendspin::SourceRole& role, std::string device, uint32_t sample_rate = 48000,
               uint8_t channels = 2, bool line_sense = true, double signal_threshold_dbfs = -50.0,
               uint32_t signal_attack_ms = 300, uint32_t signal_release_ms = 5000);
    ~AlsaSource();
    AlsaSource(const AlsaSource&) = delete;
    AlsaSource& operator=(const AlsaSource&) = delete;

    bool start();
    void stop();
    /// Publish line-sense changes on the daemon/main thread.
    void poll();

private:
    void run();
    void set_signal_from_pcm(const int16_t* samples, size_t sample_count);

    sendspin::SourceRole& role_;
    std::string device_;
    uint32_t sample_rate_;
    uint8_t channels_;
    bool line_sense_;
    double signal_threshold_dbfs_;
    uint32_t signal_attack_ms_;
    uint32_t signal_release_ms_;

    // Line-sense hysteresis state. Only touched by the capture thread.
    bool signal_present_{false};
    int candidate_signal_{-1}; // -1 none, 0 absent, 1 present
    std::chrono::steady_clock::time_point candidate_since_{};

    std::atomic<bool> running_{false};
    std::atomic<int> pending_signal_{-1}; // enabled line-sense publishes initial "absent"
    std::thread thread_;
};
