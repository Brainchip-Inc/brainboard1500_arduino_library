/*
 * MFCC front end for the BB15 Nicla Voice keyword spotting demo.
 *
 * The arithmetic here is derived from Arm's MFCC feature extraction in
 * ML-KWS-for-MCU, by way of BrainChip's own port of it:
 *
 *     Copyright (C) 2018 Arm Limited or its affiliates. All rights reserved.
 *     SPDX-License-Identifier: Apache-2.0
 *
 *     Licensed under the Apache License, Version 2.0 (the License); you may
 *     not use this file except in compliance with the License. You may obtain
 *     a copy of the License at www.apache.org/licenses/LICENSE-2.0
 *
 * A copy of that license text is in `LICENSE-APACHE-2.0` at the repository
 * root. This file and `mfcc.cpp` are new code rather than copies, but they
 * reproduce that arithmetic value for value.
 *
 * Every buffer here is a fixed-size static sized for this demo's single
 * configuration.
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
