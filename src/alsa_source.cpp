#include "alsa_source.h"
#include "log.h"

#include <alsa/asoundlib.h>
#include <algorithm>
#include <sendspin/source_role.h>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

namespace {
static constexpr const char* LOG_TAG = sendspin_cli::LOG_TAG_AUDIO;
int64_t monotonic_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

AlsaSource::AlsaSource(sendspin::SourceRole& role, std::string device, uint32_t sample_rate,
                       uint8_t channels, bool line_sense, double signal_threshold_dbfs,
                       uint32_t signal_attack_ms, uint32_t signal_release_ms)
    : role_(role), device_(std::move(device)), sample_rate_(sample_rate), channels_(channels),
      line_sense_(line_sense), signal_threshold_dbfs_(signal_threshold_dbfs),
      signal_attack_ms_(signal_attack_ms), signal_release_ms_(signal_release_ms) {
    if (line_sense_) {
        pending_signal_.store(0, std::memory_order_relaxed);
    }
}
AlsaSource::~AlsaSource() { stop(); }

bool AlsaSource::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return true;
    thread_ = std::thread(&AlsaSource::run, this);
    return true;
}
void AlsaSource::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
}

void AlsaSource::poll() {
    if (!line_sense_) return;

    const int signal = pending_signal_.exchange(-1, std::memory_order_acq_rel);
    if (signal < 0) return;

    cli_log(sendspin::LogLevel::DEBUG, "line signal %s", signal != 0 ? "present" : "absent");
    role_.update_signal(signal != 0 ? sendspin::SendspinSourceSignal::PRESENT
                                   : sendspin::SendspinSourceSignal::ABSENT);
}

void AlsaSource::set_signal_from_pcm(const int16_t* samples, size_t sample_count) {
    if (!samples || sample_count == 0) return;

    long double sum = 0.0;
    for (size_t i = 0; i < sample_count; ++i) {
        const long double x = static_cast<long double>(samples[i]) / 32768.0L;
        sum += x * x;
    }

    const double rms = std::sqrt(static_cast<double>(sum / sample_count));
    const double threshold = std::pow(10.0, signal_threshold_dbfs_ / 20.0);
    const int measured_signal = rms >= threshold ? 1 : 0;
    const int current_signal = signal_present_ ? 1 : 0;

    // Already on the stable side of the threshold: cancel any pending transition.
    if (measured_signal == current_signal) {
        candidate_signal_ = -1;
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    // A transition must remain continuously on the new side of the threshold for the full
    // attack/release interval. Any block crossing back resets the timer above.
    if (candidate_signal_ != measured_signal) {
        candidate_signal_ = measured_signal;
        candidate_since_ = now;
        return;
    }

    const uint32_t required_ms = measured_signal != 0 ? signal_attack_ms_ : signal_release_ms_;
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - candidate_since_).count();
    if (elapsed_ms < static_cast<int64_t>(required_ms)) return;

    signal_present_ = measured_signal != 0;
    candidate_signal_ = -1;
    pending_signal_.store(measured_signal, std::memory_order_release);
}

void AlsaSource::run() {
    snd_pcm_t* pcm = nullptr;
    if (snd_pcm_open(&pcm, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0) < 0) {
        sendspin_cli::log_fatal(LOG_TAG, "failed to open ALSA input %s", device_.c_str());
        running_ = false;
        return;
    }

    int err = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                                 channels_, sample_rate_, 1, 50000);
    if (err < 0) {
        sendspin_cli::log_fatal(LOG_TAG, "ALSA input configuration failed: %s", snd_strerror(err));
        snd_pcm_close(pcm);
        running_ = false;
        return;
    }

    // Verify what ALSA actually negotiated. source@v1 announces sample_rate/channels/bit_depth
    // before the first chunk, so silently accepting a different capture format would make the
    // server decode PCM at the wrong speed or layout.
    snd_pcm_hw_params_t* hw = nullptr;
    snd_pcm_hw_params_alloca(&hw);
    unsigned int actual_rate = 0;
    unsigned int actual_channels = 0;
    snd_pcm_format_t actual_format = SND_PCM_FORMAT_UNKNOWN;
    int dir = 0;
    if ((err = snd_pcm_hw_params_current(pcm, hw)) < 0 ||
        (err = snd_pcm_hw_params_get_rate(hw, &actual_rate, &dir)) < 0 ||
        (err = snd_pcm_hw_params_get_channels(hw, &actual_channels)) < 0 ||
        (err = snd_pcm_hw_params_get_format(hw, &actual_format)) < 0) {
        sendspin_cli::log_fatal(LOG_TAG, "failed to query negotiated ALSA input format: %s",
                                snd_strerror(err));
        snd_pcm_close(pcm);
        running_ = false;
        return;
    }
    if (actual_rate != sample_rate_ || actual_channels != channels_ ||
        actual_format != SND_PCM_FORMAT_S16_LE) {
        sendspin_cli::log_fatal(
            LOG_TAG,
            "ALSA negotiated %u Hz/%u ch/%s, but Sendspin announces "
            "%u Hz/%u ch/S16_LE; refusing mismatched capture",
            actual_rate, actual_channels, snd_pcm_format_name(actual_format), sample_rate_,
            channels_);
        snd_pcm_close(pcm);
        running_ = false;
        return;
    }
    cli_log(sendspin::LogLevel::INFO, "ALSA capture format %u Hz, %u ch, S16_LE",
            actual_rate, actual_channels);

    // 20 ms chunks: comfortably within source@v1's 5..150 ms recommendation.
    const snd_pcm_uframes_t frames_per_chunk = std::max<uint32_t>(1, actual_rate / 50);
    std::vector<int16_t> buffer(static_cast<size_t>(frames_per_chunk) * actual_channels);

    // Keep timestamps on one sample-derived timeline while a source stream is active. Using
    // monotonic_us() independently for every read turns scheduler jitter into timestamp jitter;
    // aiosendspin then sees artificial discontinuities even though the PCM itself is continuous.
    bool have_capture_anchor = false;
    bool stream_was_active = false;
    int64_t capture_anchor_us = 0;
    uint64_t frames_since_anchor = 0;

    while (running_) {
        snd_pcm_sframes_t got = snd_pcm_readi(pcm, buffer.data(), frames_per_chunk);
        if (got == -EPIPE) {
            cli_log(sendspin::LogLevel::WARN,
                    "ALSA capture XRUN; resetting timestamp anchor");
            snd_pcm_prepare(pcm);
            have_capture_anchor = false;
            frames_since_anchor = 0;
            continue;
        }
        if (got < 0) {
            const int original_error = static_cast<int>(got);
            got = snd_pcm_recover(pcm, original_error, 1);
            if (got < 0) {
                cli_log(sendspin::LogLevel::ERROR, "ALSA capture failed after recovery: %s",
                        snd_strerror(static_cast<int>(got)));
                break;
            }
            cli_log(sendspin::LogLevel::WARN,
                    "ALSA capture recovered from %s; resetting timestamp anchor",
                    snd_strerror(original_error));
            have_capture_anchor = false;
            frames_since_anchor = 0;
            continue;
        }
        if (got == 0) continue;

        const size_t samples = static_cast<size_t>(got) * actual_channels;
        if (line_sense_) {
            set_signal_from_pcm(buffer.data(), samples);
        }

        const bool stream_active = role_.streaming();
        if (!stream_active) {
            have_capture_anchor = false;
            frames_since_anchor = 0;
            stream_was_active = false;
            continue;
        }

        const int64_t duration_us = static_cast<int64_t>(got) * 1000000LL / actual_rate;
        if (!stream_was_active || !have_capture_anchor) {
            // snd_pcm_readi returns after this block was captured. Anchor the first sample of the
            // first delivered block, then advance solely by delivered sample count.
            capture_anchor_us = monotonic_us() - duration_us;
            frames_since_anchor = 0;
            have_capture_anchor = true;
        }
        stream_was_active = true;

        const int64_t first_sample_us = capture_anchor_us +
            static_cast<int64_t>((frames_since_anchor * 1000000ULL) / actual_rate);
        if (role_.send_audio(reinterpret_cast<const uint8_t*>(buffer.data()),
                             samples * sizeof(int16_t), first_sample_us)) {
            frames_since_anchor += static_cast<uint64_t>(got);
        } else {
            // If transport/time-sync rejected a chunk, do not later compress that lost wall-clock
            // interval into the timestamp stream. Re-anchor at the next deliverable chunk.
            have_capture_anchor = false;
            frames_since_anchor = 0;
        }
    }
    snd_pcm_drop(pcm);
    snd_pcm_close(pcm);
}
