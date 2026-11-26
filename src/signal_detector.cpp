// Copyright 2026 sendspin-cpp-cli Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "signal_detector.h"

#include <algorithm>
#include <cmath>

namespace sendspin_cli {

namespace {

int64_t read_sample(const uint8_t* p, size_t bytes) {
    uint32_t raw = 0;
    for (size_t i = 0; i < bytes; ++i) {
        raw |= static_cast<uint32_t>(p[i]) << (8U * i);
    }
    // Sign-extend from the top bit of the sample's own width.
    const unsigned shift = 32U - static_cast<unsigned>(bytes * 8U);
    return static_cast<int64_t>(static_cast<int32_t>(raw << shift) >> shift);
}

}  // namespace

double pcm_level_dbfs(const uint8_t* data, size_t length, uint8_t bit_depth) {
    size_t bytes = 0;
    switch (bit_depth) {
        case 16:
            bytes = 2;
            break;
        case 24:
            bytes = 3;
            break;
        case 32:
            bytes = 4;
            break;
        default:
            return SILENCE_DBFS;
    }
    const size_t samples = length / bytes;
    if (data == nullptr || samples == 0) {
        return SILENCE_DBFS;
    }
    const double full_scale = std::ldexp(1.0, static_cast<int>(bytes * 8U) - 1);
    double sum = 0.0;
    for (size_t i = 0; i < samples; ++i) {
        const double sample =
            static_cast<double>(read_sample(data + i * bytes, bytes)) / full_scale;
        sum += sample * sample;
    }
    const double rms = std::sqrt(sum / static_cast<double>(samples));
    if (rms <= 0.0) {
        return SILENCE_DBFS;
    }
    return std::max(SILENCE_DBFS, 20.0 * std::log10(rms));
}

SignalDetector::SignalDetector(double threshold_dbfs, uint32_t attack_ms, uint32_t release_ms)
    : threshold_dbfs_(threshold_dbfs), attack_ms_(attack_ms), release_ms_(release_ms) {}

std::optional<bool> SignalDetector::feed(double level_dbfs, uint32_t duration_ms) {
    if (level_dbfs > this->threshold_dbfs_) {
        this->quiet_ms_ = 0;
        this->loud_ms_ = std::min(this->loud_ms_ + duration_ms, this->attack_ms_);
        if (this->present_ != true && this->loud_ms_ >= this->attack_ms_) {
            this->present_ = true;
            return true;
        }
        return std::nullopt;
    }
    this->loud_ms_ = 0;
    const uint32_t needed = this->present_.has_value() ? this->release_ms_ : this->attack_ms_;
    this->quiet_ms_ = std::min(this->quiet_ms_ + duration_ms, needed);
    if (this->present_ != false && this->quiet_ms_ >= needed) {
        this->present_ = false;
        return false;
    }
    return std::nullopt;
}

std::optional<bool> SignalDetector::reset() {
    const bool was_present = this->present_ == true;
    this->present_.reset();
    this->loud_ms_ = 0;
    this->quiet_ms_ = 0;
    if (was_present) {
        this->present_ = false;
        return false;
    }
    return std::nullopt;
}

}  // namespace sendspin_cli
