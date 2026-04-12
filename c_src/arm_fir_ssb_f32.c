/*
 * Implements the SSB filter
 *
 * Copyright (C) 2025 Christian Riesch, christian@riesch.at
 * based on the ARM CMSIS DSP Library, FilteringFunctions/arm_fir_f32.c
 * Copyright (C) 2010-2021 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "arm_compiler_specific.h"
#include "dsp/filtering_functions.h"

/**
  @brief         Processing function for floating-point SSB filter.
  @param[in]     S          points to an instance of the floating-point FIR filter structure
  @param[in]     pSrc       points to the block of input data
  @param[out]    pDstI      points to the block of in phase output data
  @param[out]    pDstQ      points to the block of quadrature output data
  @param[in]     blockSize  number of samples to process

  @par Coefficient layout
  pCoeffs contains the interleaved real and imaginary taps:
    [ci[0], cq[0], ci[1], cq[1], ..., ci[numTaps/2], cq[numTaps/2-1]]
  where numTaps is the total (odd) number of coefficient values.
  Even-indexed coefficients drive the I path; odd-indexed drive the Q path.
  Both paths share the same delay-line (pState).

  @par Loop-unrolled variant (ARM_MATH_LOOPUNROLL)
  When ARM_MATH_LOOPUNROLL is defined, four output sample pairs are computed
  simultaneously.  For each coefficient pair k, the required state values are
  shared between adjacent outputs:

    acc0i += state[2k+0] * ci[k]    acc0q += state[2k+1] * cq[k]
    acc1i += state[2k+1] * ci[k]    acc1q += state[2k+2] * cq[k]
    acc2i += state[2k+2] * ci[k]    acc2q += state[2k+3] * cq[k]
    acc3i += state[2k+3] * ci[k]    acc3q += state[2k+4] * cq[k]

  State values at odd positions are thus used by both the Q path of output n
  and the I path of output n+1, reducing the number of loads by ~33 % compared
  to four independent scalar passes.
 */
void arm_fir_ssb_f32(
  const arm_fir_instance_f32 * S,
  const float32_t * pSrc,
        float32_t * pDstI,
        float32_t * pDstQ,
        uint32_t blockSize)
{
        float32_t *pState = S->pState;                 /* State pointer */
  const float32_t *pCoeffs = S->pCoeffs;               /* Coefficient pointer */
        float32_t *pStateCurnt;                        /* Points to the current sample of the state */
        float32_t *px;                                 /* Temporary pointer for state buffer */
  const float32_t *pb;                                 /* Temporary pointer for coefficient buffer */
        float32_t acc0i, acc0q;                        /* Accumulators for one output pair */
        uint32_t numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
        uint32_t i, tapCnt, blkCnt;                    /* Loop counters */

#if defined(ARM_MATH_LOOPUNROLL)
        float32_t acc1i, acc2i, acc3i;                 /* Additional I accumulators */
        float32_t acc1q, acc2q, acc3q;                 /* Additional Q accumulators */
        float32_t x0, x1, x2, x3, x4;                 /* Rolling state window */
        float32_t ci, cq;                              /* Coefficient temporaries */
#endif

  /* S->pState points to state array which contains previous frame (numTaps - 1) samples */
  /* pStateCurnt points to the location where the new input data should be written */
  pStateCurnt = &(S->pState[(numTaps - 1U)]);

#if defined(ARM_MATH_LOOPUNROLL)

  /* Loop unrolling: compute 4 output sample pairs simultaneously.
   * The variables acc0i/q ... acc3i/q hold the four output pairs being computed. */

  blkCnt = blockSize >> 2U;

  while (blkCnt > 0U)
  {
    /* Copy 4 new input samples into the state buffer */
    *pStateCurnt++ = *pSrc++;
    *pStateCurnt++ = *pSrc++;
    *pStateCurnt++ = *pSrc++;
    *pStateCurnt++ = *pSrc++;

    /* Clear all 8 accumulators */
    acc0i = 0.0f; acc0q = 0.0f;
    acc1i = 0.0f; acc1q = 0.0f;
    acc2i = 0.0f; acc2q = 0.0f;
    acc3i = 0.0f; acc3q = 0.0f;

    /* Initialise state and coefficient pointers */
    px = pState;
    pb = pCoeffs;

    /* Pre-load first 4 state values into the rolling window */
    x0 = *px++;
    x1 = *px++;
    x2 = *px++;
    x3 = *px++;

    /* Inner loop: one (ci, cq) coefficient pair = 2 taps per iteration.
     * Each state value loaded into the window serves two multiply-accumulates
     * — one for the Q path of output n and one for the I path of output n+1. */
    tapCnt = numTaps >> 1U;

    while (tapCnt > 0U)
    {
      /* Load the next state value and the coefficient pair */
      x4 = *px++;
      ci = *pb++;
      cq = *pb++;

      /* 8 multiply-accumulates across 4 output pairs */
      acc0i += x0 * ci;
      acc0q += x1 * cq;
      acc1i += x1 * ci;
      acc1q += x2 * cq;
      acc2i += x2 * ci;
      acc2q += x3 * cq;
      acc3i += x3 * ci;
      acc3q += x4 * cq;

      /* Slide the rolling window forward by 2 */
      x0 = x2;
      x1 = x3;
      x2 = x4;
      x3 = *px++;

      /* Decrement loop counter */
      tapCnt--;
    }

    /* Handle the remaining single real tap when numTaps is odd */
    if (numTaps & 1U)
    {
      ci = *pb++;
      acc0i += x0 * ci;
      acc1i += x1 * ci;
      acc2i += x2 * ci;
      acc3i += x3 * ci;
    }

    /* Advance state pointer by 4 for the next group of 4 output pairs */
    pState += 4;

    /* Store 4 I/Q output pairs */
    *pDstI++ = acc0i;
    *pDstQ++ = acc0q;
    *pDstI++ = acc1i;
    *pDstQ++ = acc1q;
    *pDstI++ = acc2i;
    *pDstQ++ = acc2q;
    *pDstI++ = acc3i;
    *pDstQ++ = acc3q;

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Loop unrolling: compute remaining output samples (blockSize % 4) */
  blkCnt = blockSize & 3U;

#else

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

#endif /* #if defined(ARM_MATH_LOOPUNROLL) */

  while (blkCnt > 0U)
  {
    /* Copy one sample at a time into state buffer */
    *pStateCurnt++ = *pSrc++;

    /* Set the accumulators to zero */
    acc0i = 0.0f;
    acc0q = 0.0f;

    /* Initialize state pointer */
    px = pState;

    /* Initialize Coefficient pointer */
    pb = pCoeffs;

    i = numTaps;

    /* Perform the multiply-accumulates */
    while (i > 1U)
    {
      /* acci =  b[numTaps-1] * x[n-numTaps-1] + b[numTaps-3] * x[n-numTaps-3] +...+ b[0] * x[0] */
      /* accq =  b[numTaps-2] * x[n-numTaps-2] + b[numTaps-4] * x[n-numTaps-4] +...+ b[1] * x[1] */
      acc0i += *px++ * *pb++;
      acc0q += *px++ * *pb++;

      i -= 2;
    }

    if (i > 0U)
    {
      acc0i += *px++ * *pb++;
    }

    /* Store result in destination buffer. */
    *pDstI++ = acc0i;
    *pDstQ++ = acc0q;

    /* Advance state pointer by 1 for the next sample */
    pState = pState + 1U;

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Processing is complete.
     Now copy the last numTaps - 1 samples to the start of the state buffer.
     This prepares the state buffer for the next function call. */

  /* Points to the start of the state buffer */
  pStateCurnt = S->pState;

#if defined(ARM_MATH_LOOPUNROLL)

  /* Loop unrolling: copy 4 samples at a time */
  tapCnt = (numTaps - 1U) >> 2U;

  while (tapCnt > 0U)
  {
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;

    /* Decrement loop counter */
    tapCnt--;
  }

  /* Calculate remaining number of copies */
  tapCnt = (numTaps - 1U) & 3U;

#else

  /* Initialize tapCnt with number of taps */
  tapCnt = (numTaps - 1U);

#endif /* #if defined(ARM_MATH_LOOPUNROLL) */

  /* Copy remaining data */
  while (tapCnt > 0U)
  {
    *pStateCurnt++ = *pState++;

    /* Decrement loop counter */
    tapCnt--;
  }

}
