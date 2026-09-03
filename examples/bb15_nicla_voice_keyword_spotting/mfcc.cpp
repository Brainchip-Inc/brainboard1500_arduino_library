/*
 * Derived from Arm's MFCC feature extraction in ML-KWS-for-MCU, Apache-2.0.
 * See `mfcc.h` for the attribution this license requires.
 */

#include "mfcc.h"

#include <math.h>

#include "kiss_fftr.h"

namespace {

// Configuration values from spark: `source/include/audio/mfcc.h` for the
// filterbank size and mel range, `source/Kconfig` for the sample rate.
constexpr float kSampleRateHz = 16000.0f;
constexpr int kMelLowFreqHz = 20;
constexpr int kMelHighFreqHz = 4000;

// spark defines its own pi rather than taking math.h's, so use its literals.
constexpr double kTwoPi = 6.283185307179586476925286766559005;
constexpr double kPi = kTwoPi / 2;

// Bins the real FFT produces, and the count the filterbank scans. spark's
// create_mel_fbank() stops one short of the end, so the Nyquist bin is
// computed and never used.
constexpr int kPowerBins = kMfccFrameSamples / 2 + 1;
constexpr int kScannedBins = kMfccFrameSamples / 2;

// The 40 triangular filters between 20 Hz and 4000 Hz span 308 FFT bins in
// total at this sample rate and frame length. mfcc_begin() verifies the figure
// rather than trusting it.
constexpr int kFilterBankWeights = 308;

// kiss_fftr_alloc() asks for 6688 bytes for a 640-point real forward FFT.
// Handing it this buffer keeps the FFT off the heap. 320 factors into 4, 4, 4
// and 5, so only kissfft's radix-4 and radix-5 butterflies ever run, and
// neither of those allocates either.
constexpr size_t kFftConfigBytes = 6688;

alignas(8) uint8_t g_fft_config_memory[kFftConfigBytes];
kiss_fftr_cfg g_fft_config = nullptr;

float g_window[kMfccFrameSamples];
float g_frame[kMfccFrameSamples];
kiss_fft_cpx g_spectrum[kPowerBins];
float g_power[kPowerBins];
float g_filter_bank[kFilterBankWeights];
int16_t g_filter_first[kMfccFilterBankBins];
int16_t g_filter_last[kMfccFilterBankBins];
int16_t g_filter_offset[kMfccFilterBankBins];
float g_mel_energies[kMfccFilterBankBins];
float g_dct_matrix[kMfccFeatures * kMfccFilterBankBins];

/**
 * @brief spark's mel scale.
 *
 * @param frequency  Frequency in Hz.
 * @return The mel value.
 */
float mel_scale(float frequency) {
  return 1127.0f * logf(1.0f + frequency / 700.0f);
}

/**
 * @brief Fill the periodic Hann window.
 */
void build_window() {
  for (int i = 0; i < kMfccFrameSamples; ++i) {
    g_window[i] = 0.5f - 0.5f * cosf(static_cast<float>(kTwoPi) *
                                     static_cast<float>(i) /
                                     static_cast<float>(kMfccFrameSamples));
  }
}

/**
 * @brief Fill the DCT matrix used to turn log mel energies into coefficients.
 *
 * The cosine is evaluated in double and stored as float, as spark does, so the
 * coefficients round the same way.
 */
void build_dct_matrix() {
  const float normalizer =
      sqrtf(2.0f / static_cast<float>(kMfccFilterBankBins));
  for (int k = 0; k < kMfccFeatures; ++k) {
    for (int n = 0; n < kMfccFilterBankBins; ++n) {
      g_dct_matrix[k * kMfccFilterBankBins + n] = static_cast<float>(
          static_cast<double>(normalizer) *
          cos(kPi / static_cast<double>(kMfccFilterBankBins) *
              (static_cast<double>(n) + 0.5) * static_cast<double>(k)));
    }
  }
}

/**
 * @brief Build the triangular mel filterbank into one flat weight array.
 *
 * Each filter's weights are non-zero over a contiguous run of FFT bins,
 * because the mel scale rises with frequency, so the weights can be written
 * as they are computed and addressed later by a per-filter offset.
 *
 * @return True when the weights exactly filled the static array.
 */
bool build_filter_bank() {
  const float fft_bin_width = kSampleRateHz / kMfccFrameSamples;
  const float mel_low = mel_scale(kMelLowFreqHz);
  const float mel_high = mel_scale(kMelHighFreqHz);
  const float mel_delta =
      (mel_high - mel_low) / (kMfccFilterBankBins + 1);

  int written = 0;
  for (int bin = 0; bin < kMfccFilterBankBins; ++bin) {
    const float left_mel = mel_low + bin * mel_delta;
    const float center_mel = mel_low + (bin + 1) * mel_delta;
    const float right_mel = mel_low + (bin + 2) * mel_delta;
    int first = -1;
    int last = -1;
    g_filter_offset[bin] = static_cast<int16_t>(written);

    for (int i = 0; i < kScannedBins; ++i) {
      const float mel = mel_scale(fft_bin_width * i);
      if (mel <= left_mel || mel >= right_mel) {
        continue;
      }
      if (written >= kFilterBankWeights) {
        return false;
      }
      g_filter_bank[written++] =
          mel <= center_mel ? (mel - left_mel) / (center_mel - left_mel)
                            : (right_mel - mel) / (right_mel - center_mel);
      if (first == -1) {
        first = i;
      }
      last = i;
    }
    g_filter_first[bin] = static_cast<int16_t>(first);
    g_filter_last[bin] = static_cast<int16_t>(last);
  }
  return written == kFilterBankWeights;
}

}  // namespace

bool mfcc_begin() {
  size_t available = sizeof(g_fft_config_memory);
  g_fft_config =
      kiss_fftr_alloc(kMfccFrameSamples, 0, g_fft_config_memory, &available);
  if (g_fft_config == nullptr) {
    return false;
  }
  build_window();
  build_dct_matrix();
  return build_filter_bank();
}

void mfcc_compute(const int16_t* audio, float* out) {
  // The same normalization to (-1, 1) TensorFlow's MFCC op applies.
  for (int i = 0; i < kMfccFrameSamples; ++i) {
    g_frame[i] = static_cast<float>(audio[i]) / (1 << 15);
  }
  for (int i = 0; i < kMfccFrameSamples; ++i) {
    g_frame[i] *= g_window[i];
  }

  kiss_fftr(g_fft_config, g_frame, g_spectrum);
  for (int i = 0; i < kPowerBins; ++i) {
    g_power[i] = g_spectrum[i].r * g_spectrum[i].r +
                 g_spectrum[i].i * g_spectrum[i].i;
  }

  for (int bin = 0; bin < kMfccFilterBankBins; ++bin) {
    const float* weights = &g_filter_bank[g_filter_offset[bin]];
    float energy = 0.0f;
    int weight = 0;
    for (int i = g_filter_first[bin]; i <= g_filter_last[bin]; ++i) {
      energy += g_power[i] * weights[weight++];
    }
    g_mel_energies[bin] = energy;
  }
  for (int bin = 0; bin < kMfccFilterBankBins; ++bin) {
    g_mel_energies[bin] = logf(g_mel_energies[bin] + 1e-6f);
  }

  for (int i = 0; i < kMfccFeatures; ++i) {
    float sum = 0.0f;
    for (int j = 0; j < kMfccFilterBankBins; ++j) {
      sum += g_dct_matrix[i * kMfccFilterBankBins + j] * g_mel_energies[j];
    }
    out[i] = sum;
  }
}
