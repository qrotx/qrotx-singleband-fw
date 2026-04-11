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
        float32_t acc0i, acc0q;                        /* Accumulator */
        uint32_t numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
        uint32_t i, tapCnt, blkCnt;                    /* Loop counters */

  /* S->pState points to state array which contains previous frame (numTaps - 1) samples */
  /* pStateCurnt points to the location where the new input data should be written */
  pStateCurnt = &(S->pState[(numTaps - 1U)]);

  /* Initialize blkCnt with number of taps */
  blkCnt = blockSize;

  while (blkCnt > 0U)
  {
    /* Copy one sample at a time into state buffer */
    *pStateCurnt++ = *pSrc++;

    /* Set the accumulator to zero */
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

      i-=2;
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

  /* Initialize tapCnt with number of taps */
  tapCnt = (numTaps - 1U);

  /* Copy remaining data */
  while (tapCnt > 0U)
  {
    *pStateCurnt++ = *pState++;

    /* Decrement loop counter */
    tapCnt--;
  }

}
