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

/// --signal-threshold: whether a capture input carries a signal, from its level.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace sendspin_cli {

/// What --signal-threshold accepts, in dBFS, either side inclusive.
inline constexpr int MIN_SIGNAL_THRESHOLD_DBFS = -90;
inline constexpr int MAX_SIGNAL_THRESHOLD_DBFS = -1;

/// Floor pcm_level_dbfs() reports for digital silence.
inline constexpr double SILENCE_DBFS = -120.0;

/// How long the level must stay above the threshold before the signal counts as present.
inline constexpr uint32_t SIGNAL_ATTACK_MS = 500;

/// How long it must stay below before a present signal counts as absent.
inline constexpr uint32_t SIGNAL_RELEASE_MS = 5000;

/// RMS level of interleaved little-endian signed PCM across all channels, in dBFS.
/// @param bit_depth 16, 24 (3 packed bytes) or 32; anything else reads as silence.
double pcm_level_dbfs(const uint8_t* data, size_t length, uint8_t bit_depth);

/// Turns a stream of levels into present/absent with attack and release hysteresis.
class SignalDetector {
public:
    explicit SignalDetector(double threshold_dbfs, uint32_t attack_ms = SIGNAL_ATTACK_MS,
                            uint32_t release_ms = SIGNAL_RELEASE_MS);

    /// Feeds the level of `duration_ms` of audio.
    /// @return the new state when it changed: true for present. The first decision always counts,
    /// and an absent one needs only the attack time, so a quiet input is reported promptly.
    std::optional<bool> feed(double level_dbfs, uint32_t duration_ms);

    /// Forgets the state, as when the device was lost; reports absent if it was present.
    std::optional<bool> reset();

    std::optional<bool> state() const {
        return this->present_;
    }

private:
    double threshold_dbfs_;
    uint32_t attack_ms_;
    uint32_t release_ms_;
    std::optional<bool> present_;
    uint32_t loud_ms_{0};
    uint32_t quiet_ms_{0};
};

}  // namespace sendspin_cli
