/*
 * MFCC front end for the BB15 Nicla Voice keyword spotting demo.
 *
 * The arithmetic is spark's `mfcc_compute()` from
 * `source/core/common/audio/mfcc.c`, which is in turn Arm's MFCC from
 * ML-KWS-for-MCU (Apache-2.0). It is kept value for value, because the
 * keyword model was trained against exactly these features and a front end
 * that is subtly different fails the model in a way that is hard to diagnose.
 *
 * What differs from spark is storage, not maths. Every buffer here is a
 * fixed-size static sized for this demo's single configuration, because the
 * nRF52832 has 64 KB of RAM and spark's malloc-per-buffer arrangement does not
 * belong on it.
 */

#ifndef BB15_KWS_MFCC_H_
#define BB15_KWS_MFCC_H_

#include <stdint.h>

/// Samples per MFCC frame: two 20 ms hops, and the real FFT length.
constexpr int kMfccFrameSamples = 640;

/// Coefficients kept per frame, the model input's second dimension.
constexpr int kMfccFeatures = 10;

/// Mel filterbank bins, spark's `NUM_FBANK_BINS`.
constexpr int kMfccFilterBankBins = 40;

/**
 * @brief Build the window, mel filterbank, DCT matrix and FFT configuration.
 *
 * Call once before mfcc_compute(). Allocates nothing.
 *
 * @return True on success. False means a static buffer is mis-sized, which is
 *         a build-time mistake rather than a runtime condition.
 */
bool mfcc_begin();

/**
 * @brief Turn one frame of PCM into kMfccFeatures coefficients.
 *
 * @param audio  kMfccFrameSamples samples, DC blocked by the caller.
 * @param out    Receives kMfccFeatures coefficients.
 */
void mfcc_compute(const int16_t* audio, float* out);

#endif  // BB15_KWS_MFCC_H_
