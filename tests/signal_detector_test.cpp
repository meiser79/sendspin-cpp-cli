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

/// --signal-threshold: the level meter and the present/absent hysteresis.

#include "signal_detector.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace sendspin_cli {
namespace {

std::vector<uint8_t> constant_samples(int32_t value, size_t count, size_t bytes) {
    std::vector<uint8_t> out;
    for (size_t i = 0; i < count; ++i) {
        const auto raw = static_cast<uint32_t>(value);
        for (size_t b = 0; b < bytes; ++b) {
            out.push_back(static_cast<uint8_t>(raw >> (8U * b)));
        }
    }
    return out;
}

// pcm_level_dbfs

TEST(PcmLevelDbfs, DigitalSilenceIsTheFloor) {
    const std::vector<uint8_t> zeros(480 * 4, 0);
    EXPECT_EQ(pcm_level_dbfs(zeros.data(), zeros.size(), 16), SILENCE_DBFS);
    EXPECT_EQ(pcm_level_dbfs(nullptr, 0, 16), SILENCE_DBFS);
}

TEST(PcmLevelDbfs, HalfScaleIsAboutMinusSixAtEveryDepth) {
    const std::vector<uint8_t> s16 = constant_samples(16384, 100, 2);
    const std::vector<uint8_t> s24 = constant_samples(4194304, 100, 3);
    const std::vector<uint8_t> s32 = constant_samples(1073741824, 100, 4);

    EXPECT_NEAR(pcm_level_dbfs(s16.data(), s16.size(), 16), -6.02, 0.01);
    EXPECT_NEAR(pcm_level_dbfs(s24.data(), s24.size(), 24), -6.02, 0.01);
    EXPECT_NEAR(pcm_level_dbfs(s32.data(), s32.size(), 32), -6.02, 0.01);
}

TEST(PcmLevelDbfs, NegativeSamplesAreSignExtended) {
    // -4194304 in 24-bit packs to 0xC00000: read unsigned, it would be near full scale.
    const std::vector<uint8_t> s24 = constant_samples(-4194304, 100, 3);
    EXPECT_NEAR(pcm_level_dbfs(s24.data(), s24.size(), 24), -6.02, 0.01);
}

TEST(PcmLevelDbfs, AnUnknownDepthReadsAsSilence) {
    const std::vector<uint8_t> data(64, 0x7f);
    EXPECT_EQ(pcm_level_dbfs(data.data(), data.size(), 8), SILENCE_DBFS);
}

// SignalDetector

TEST(SignalDetector, PresentOnlyAfterTheAttackTime) {
    SignalDetector detector(-50.0, 500, 5000);
    for (int i = 0; i < 24; ++i) {
        EXPECT_EQ(detector.feed(-20.0, 20), std::nullopt) << "read " << i;
    }
    EXPECT_EQ(detector.feed(-20.0, 20), std::optional<bool>(true));
    EXPECT_EQ(detector.feed(-20.0, 20), std::nullopt);
}

TEST(SignalDetector, AQuietStartIsReportedAfterTheAttackTimeNotTheRelease) {
    SignalDetector detector(-50.0, 500, 5000);
    std::optional<bool> change;
    uint32_t elapsed = 0;
    while (!change && elapsed < 10000) {
        change = detector.feed(-90.0, 20);
        elapsed += 20;
    }
    EXPECT_EQ(change, std::optional<bool>(false));
    EXPECT_EQ(elapsed, 500U);
}

TEST(SignalDetector, AbsentOnlyAfterTheReleaseTime) {
    SignalDetector detector(-50.0, 100, 1000);
    for (int i = 0; i < 5; ++i) {
        detector.feed(-20.0, 20);
    }
    ASSERT_EQ(detector.state(), std::optional<bool>(true));

    for (int i = 0; i < 49; ++i) {
        EXPECT_EQ(detector.feed(-90.0, 20), std::nullopt) << "read " << i;
    }
    EXPECT_EQ(detector.feed(-90.0, 20), std::optional<bool>(false));
}

TEST(SignalDetector, AShortPauseDoesNotEndTheSignal) {
    SignalDetector detector(-50.0, 100, 1000);
    for (int i = 0; i < 5; ++i) {
        detector.feed(-20.0, 20);
    }
    for (int i = 0; i < 40; ++i) {
        EXPECT_EQ(detector.feed(-90.0, 20), std::nullopt);
    }
    // Loud again before the release ran out: the quiet count starts over.
    detector.feed(-20.0, 20);
    for (int i = 0; i < 49; ++i) {
        EXPECT_EQ(detector.feed(-90.0, 20), std::nullopt);
    }
    EXPECT_EQ(detector.state(), std::optional<bool>(true));
}

TEST(SignalDetector, AShortBurstDoesNotStartTheSignal) {
    SignalDetector detector(-50.0, 500, 5000);
    for (int i = 0; i < 10; ++i) {
        detector.feed(-20.0, 20);
    }
    detector.feed(-90.0, 20);
    for (int i = 0; i < 10; ++i) {
        EXPECT_NE(detector.feed(-20.0, 20), std::optional<bool>(true));
    }
}

TEST(SignalDetector, TheThresholdItselfCountsAsQuiet) {
    SignalDetector detector(-50.0, 20, 20);
    EXPECT_EQ(detector.feed(-50.0, 20), std::optional<bool>(false));
}

TEST(SignalDetector, ResetReportsAbsentOnlyIfItWasPresent) {
    SignalDetector detector(-50.0, 20, 1000);
    EXPECT_EQ(detector.reset(), std::nullopt);

    detector.feed(-20.0, 20);
    ASSERT_EQ(detector.state(), std::optional<bool>(true));
    EXPECT_EQ(detector.reset(), std::optional<bool>(false));
    EXPECT_EQ(detector.state(), std::optional<bool>(false));
    // A device back with a signal is present again after the attack time alone.
    EXPECT_EQ(detector.feed(-20.0, 20), std::optional<bool>(true));
}

}  // namespace
}  // namespace sendspin_cli
