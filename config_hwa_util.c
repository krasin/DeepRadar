/**
 *   @file  config_hwa_util.c
 *
 *   @brief
 *      Hardware accelerator Configuration Utility API implementation.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/hwa/hwa.h>

#include "config_hwa_util.h"
#include "mmw.h"


uint32_t log2Approx(uint32_t x)
{
    uint32_t idx, detectFlag = 0;

    if ( x < 2)
    {
        return (0);
    }

    idx = 32U;
    while((detectFlag==0U) || (idx==0U))
    {
        if(x & 0x80000000U)
        {
            detectFlag = 1;
        }
        x <<= 1U;
        idx--;
    }

    if(x != 0)
    {
        idx = idx + 1;
    }

    return(idx);
}

void HWAutil_configRangeFFT(HWA_Handle handle,
                            uint32_t  paramSetStartIdx,
                            uint32_t numAdcSamples,
                            uint32_t numRangeBins,
                            uint8_t numRxAnt,
                            uint32_t windowOffsetBytes,

                            uint8_t dmaTriggerSourcePing,
                            uint8_t dmaTriggerSourcePong,

                            uint8_t dmaDestChannelPing,
                            uint8_t dmaDestChannelPong,

                            uint16_t hwaMemAdcBufOffset,

                            uint16_t hwaMemDestPingOffset,
                            uint16_t hwaMemDestPongOffset,
                            uint8_t  hwaTriggerMode)
{
    HWA_InterruptConfig     paramISRConfig;
    int32_t errCode = 0;
    uint32_t paramsetIdx = paramSetStartIdx;
    uint32_t pingParamSetIdx = 0;
    HWA_ParamConfig hwaParamCfg[HWAUTIL_NUM_PARAM_SETS_1D];

    memset(hwaParamCfg,0,sizeof(hwaParamCfg));

    /***********************/
    /* PING DUMMY PARAMSET */
    /***********************/
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA; //Software triggered  - in demo this will be HWA_TRIG_MODE_DMA
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = dmaTriggerSourcePing; //in demo this will be first EDMA Src channel id
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_NONE; //dummy
    errCode = HWA_configParamSet(handle,paramsetIdx,&hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramsetIdx, errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /***********************/
    /* PING PROCESS PARAMSET */
    /***********************/
    paramsetIdx++;
    pingParamSetIdx = paramsetIdx;
    hwaParamCfg[paramsetIdx].triggerMode = hwaTriggerMode;
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; //do FFT

    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemAdcBufOffset; // address is relative to start of MEM0
    hwaParamCfg[paramsetIdx].source.srcAcnt = numAdcSamples - 1; //this is samples - 1
    hwaParamCfg[paramsetIdx].source.srcAIdx = numRxAnt * sizeof(uint32_t); // 16 bytes
    hwaParamCfg[paramsetIdx].source.srcBcnt = numRxAnt-1; //no iterations here
    hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint32_t); //should be dont care
    hwaParamCfg[paramsetIdx].source.srcShift = 0; //no shift
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; //no shift
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].source.srcScale = 8;
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0; //bpm removal not enabled
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0; //dont care

    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemDestPingOffset; // address is relative to start of MEM0
    hwaParamCfg[paramsetIdx].dest.dstAcnt = numRangeBins-1; //this is samples - 1
    hwaParamCfg[paramsetIdx].dest.dstAIdx = numRxAnt * sizeof(uint32_t); //
    hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint32_t); //should be dont care
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].dest.dstScale = 0;
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = log2Approx(numRangeBins);
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 1; //enabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = windowOffsetBytes; //start of window RAM
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = 0; //non-symmetric - in demo do we make this symmetric
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; //fftsize is less than 1K
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples

    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    errCode = HWA_configParamSet(handle,paramsetIdx,&hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n",errCode,paramsetIdx);
        MmwDemo_debugAssert (0);
        return;
    }
    /* enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = dmaDestChannelPing;  //TODO sync this define EDMA channel to trigger to copy the data out
    //paramISRConfig.cpu.callbackArg = paramSetSem;//TODO check if NULL is required
    errCode = HWA_enableParamSetInterrupt(handle,paramsetIdx,&paramISRConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enableParamSetInterrupt(PING DMA) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /***********************/
    /* PONG DUMMY PARAMSET */
    /***********************/
    paramsetIdx++;
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = dmaTriggerSourcePong; //in demo this will be second EDMA Src channel id
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_NONE; //dummy
    errCode = HWA_configParamSet(handle,paramsetIdx,&hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n",errCode,paramsetIdx);
        MmwDemo_debugAssert (0);
        return;
    }

    /***********************/
    /* PONG PROCESS PARAMSET */
    /***********************/
    paramsetIdx++;
    hwaParamCfg[paramsetIdx] = hwaParamCfg[pingParamSetIdx];
    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemDestPongOffset;
    errCode = HWA_configParamSet(handle,paramsetIdx,&hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n",errCode,paramsetIdx);
        MmwDemo_debugAssert (0);
        return;
    }
    
    /* enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = dmaDestChannelPong;
    //paramISRConfig.cpu.callbackArg = paramSetSem;//TODO check if NULL is required
    errCode = HWA_enableParamSetInterrupt(handle,paramsetIdx,&paramISRConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enableParamSetInterrupt(PING DMA) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

void HWAutil_configDopplerFFT
(
                                HWA_Handle handle,
                                uint32_t paramSetStartIdx,
                                uint32_t dopplerFftSize,
                                uint8_t numVirtualAnt,
                                uint32_t numRangeBinsPerIter,
                                uint32_t windowOffsetBytes,

                                uint8_t dmaTriggerSourcePing,
                                uint8_t dmaTriggerSourcePong,

                                uint8_t dmaDestChannelPing,
                                uint8_t dmaDestChannelPong,

                                uint16_t hwaMemSourcePingOffset,
                                uint16_t hwaMemSourcePongOffset,

                                uint16_t hwaMemDestPingOffset,
                                uint16_t hwaMemDestPongOffset,
                                uint32_t option
)
{
    HWA_ParamConfig hwaParamCfg[HWAUTIL_NUM_PARAM_SETS_2D];
    uint32_t paramsetIdx = 0;
    int32_t errCode = 0;
    uint32_t k;
    HWA_InterruptConfig     paramISRConfig;
    uint32_t numDopplerBins = dopplerFftSize;

    memset( (void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));

    //Doppler FFT
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA; //Immediate following first - in demo this should be HWA_TRIG_MODE_DFE
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = dmaTriggerSourcePing; //Immediate following first - in demo this should be HWA_TRIG_MODE_DFE
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; //do FFT

    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemSourcePingOffset; // address is relative to start of MEM0
    hwaParamCfg[paramsetIdx].source.srcAcnt = dopplerFftSize - 1; //size in samples - 1

    hwaParamCfg[paramsetIdx].source.srcAIdx = numVirtualAnt * sizeof(uint32_t); //
    hwaParamCfg[paramsetIdx].source.srcBcnt = numVirtualAnt - 1; //no iterations here
    hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint32_t); //should be dont care
    hwaParamCfg[paramsetIdx].source.srcShift = 0; //no shift
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; //no shift
    hwaParamCfg[paramsetIdx].source.srcRealComplex = 0; //complex data
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].source.srcScale = 0;
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0; //bpm removal not enabled
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0; //dont care

    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemDestPingOffset; // address is relative to start of MEM0
    hwaParamCfg[paramsetIdx].dest.dstAcnt = dopplerFftSize - 1; //this is samples - 1
    hwaParamCfg[paramsetIdx].dest.dstAIdx = numVirtualAnt * sizeof(uint32_t); // 16 bytes
    hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint32_t); //should be dont care
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = 0; //same as input - complex
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].dest.dstScale = 8;
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = log2Approx(dopplerFftSize);

    /* scaling is enabled in all stages except in the first one because of the
     * Hanning window scaling by half */
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = (dopplerFftSize - 1) >> 1;

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 1; //enabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = windowOffsetBytes; //start of window RAM
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = 1; //non-symmetric - in demo do we make this symmetric
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; //fftsize is less than 1K

    if(option == (uint32_t)DATA_PATH_CHAIN_SEPARATE_LOGMAG)
    {
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    }
    else    
    {
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;
    }
        
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    errCode = HWA_configParamSet(handle, paramSetStartIdx+paramsetIdx, &hwaParamCfg[paramsetIdx], NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx+paramsetIdx, errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    //Doppler FFT
    for (paramsetIdx = 1; paramsetIdx < numRangeBinsPerIter; paramsetIdx++)
    {
        hwaParamCfg[paramsetIdx] = hwaParamCfg[paramsetIdx-1];
        hwaParamCfg[paramsetIdx].source.srcAddr += sizeof(uint32_t) * numVirtualAnt * dopplerFftSize;
        hwaParamCfg[paramsetIdx].dest.dstAddr += sizeof(uint32_t) * numVirtualAnt * dopplerFftSize;
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;

        errCode = HWA_configParamSet(handle, paramSetStartIdx+paramsetIdx, &hwaParamCfg[paramsetIdx], NULL);
        if (errCode != 0)
        {
            //System_printf("Error: Doppler FFT: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx+paramsetIdx, errCode);
            MmwDemo_debugAssert (0);
            return;
        }
    }

    //log2 magnitude
    memset( (void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; //Immediate following first - in demo this should be HWA_TRIG_MODE_DFE
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; //do FFT

    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemDestPingOffset; // address is relative to start of MEM0
    hwaParamCfg[paramsetIdx].source.srcAcnt = numDopplerBins * numVirtualAnt * numRangeBinsPerIter - 1; //size in samples - 1
    hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint32_t); //
    hwaParamCfg[paramsetIdx].source.srcBcnt = 0; //no iterations here
    hwaParamCfg[paramsetIdx].source.srcBIdx = 0; //should be dont care
    hwaParamCfg[paramsetIdx].source.srcShift = 0; //no shift
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; //no shift
    hwaParamCfg[paramsetIdx].source.srcRealComplex = 0; //complex data
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].source.srcScale = 0;
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0; //bpm removal not enabled
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0; //dont care

    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemSourcePingOffset; // address is relative to start of MEM0
    hwaParamCfg[paramsetIdx].dest.dstAcnt = numDopplerBins * numVirtualAnt * numRangeBinsPerIter - 1; //this is samples - 1
    hwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint16_t); // 2 bytes
    hwaParamCfg[paramsetIdx].dest.dstBIdx = 0; //should be dont care
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = 1; //same as input - complex
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED; //same as input - signed
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].dest.dstScale = 0;
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = 1;//TODO remove later after driver corrects
    if(option == (uint32_t)DATA_PATH_CHAIN_SEPARATE_LOGMAG)
    {
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;
    }
    else
    {
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    }
    
    errCode = HWA_configParamSet(handle, paramSetStartIdx+paramsetIdx, &hwaParamCfg[paramsetIdx], NULL);
    if (errCode != 0)
    {
        //System_printf("Error: Log2 HWA_configParamSet(%d) returned %d \n", paramSetStartIdx+paramsetIdx, errCode);
        //System_printf("numDopplerBins %d numVirtualAnt %d numRangeBinsPerIter %d\n", numDopplerBins,numVirtualAnt, numRangeBinsPerIter);
        MmwDemo_debugAssert (0);
        return;
    }

    paramsetIdx++;
    //Sum of magnitudes
    memset( (void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; //Immediate
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; //do FFT

    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemSourcePingOffset; // address is relative to start of MEM0
    hwaParamCfg[paramsetIdx].source.srcAcnt = numVirtualAnt-1; //size in samples - 1

    hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint16_t); //
    hwaParamCfg[paramsetIdx].source.srcBcnt = numDopplerBins * numRangeBinsPerIter - 1; //no iterations here
    hwaParamCfg[paramsetIdx].source.srcBIdx = numVirtualAnt * sizeof(uint16_t); //should be dont care
    hwaParamCfg[paramsetIdx].source.srcShift = 0; //no shift
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; //no shift
    hwaParamCfg[paramsetIdx].source.srcRealComplex = 1; //real data
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED; //signed
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].source.srcScale = 2;
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0; //bpm removal not enabled
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0; //dont care

    hwaParamCfg[paramsetIdx].dest.dstAddr = (uint16_t) (hwaMemDestPingOffset + numDopplerBins *
        numVirtualAnt * numRangeBinsPerIter * sizeof(uint32_t)); // address is relative to start of MEM0

    hwaParamCfg[paramsetIdx].dest.dstAcnt = 1 - 1; //this is samples - 1
    hwaParamCfg[paramsetIdx].dest.dstAIdx =  sizeof(uint16_t); // 16 bytes
    hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint16_t); //should be dont care
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = 1; //same as input - complex
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED; //same as input - signed
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].dest.dstScale = 8;
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    if(numVirtualAnt == 1)
    {
        /*If number of virtual antennas is 1, do not use FFT to compute sum magnitude.
        There is nothing to sum and azimuth can not be estimated.*/
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = 1;
    }
    else
    {
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = log2Approx(numVirtualAnt);
    }
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3FF; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 0; //enabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = 0; //start of window RAM
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = 1; //non-symmetric - in demo do we make this symmetric
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; //fftsize is less than 1K
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples

    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    errCode = HWA_configParamSet(handle, paramSetStartIdx+paramsetIdx, &hwaParamCfg[paramsetIdx], NULL);
    if (errCode != 0)
    {
        //System_printf("Error: Sum of mag HWA_configParamSet(%d) returned %d\n", paramSetStartIdx+paramsetIdx, errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    /* Enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = dmaDestChannelPing;  //EDMA channel to trigger to copy the data out
    paramISRConfig.cpu.callbackArg = NULL;
    errCode = HWA_enableParamSetInterrupt(handle, paramSetStartIdx+paramsetIdx, &paramISRConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enableParamSetInterrupt(PING DMA) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }


    //programming HWACC for the pong buffer
    for (k=0; k < numRangeBinsPerIter + 2; k++)
    {
        paramsetIdx =  k + numRangeBinsPerIter + 2;
        hwaParamCfg[paramsetIdx] = hwaParamCfg[k];
        hwaParamCfg[paramsetIdx].source.srcAddr += (hwaMemSourcePongOffset - hwaMemSourcePingOffset);
        hwaParamCfg[paramsetIdx].dest.dstAddr += (hwaMemDestPongOffset - hwaMemDestPingOffset);//#def??

        if (k == 0)
        {
            hwaParamCfg[paramsetIdx].dmaTriggerSrc = dmaTriggerSourcePong;
        }


        errCode = HWA_configParamSet(handle, paramSetStartIdx + paramsetIdx, &hwaParamCfg[paramsetIdx], NULL);
        if (errCode != 0)
        {
            //System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx + paramsetIdx, errCode);
            MmwDemo_debugAssert (0);
            return;
        }
    }

    /* Enable the DMA hookup to the last paramset */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = dmaDestChannelPong;  //EDMA channel to trigger to copy the data out
    paramISRConfig.cpu.callbackArg = NULL;
    errCode = HWA_enableParamSetInterrupt(handle, paramSetStartIdx + 2*(numRangeBinsPerIter+2)-1, &paramISRConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enableParamSetInterrupt(PONG DMA) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /***********************/
    /* CONFIG WINDOW RAM   */
    /***********************/
    /* if windowing is enabled, load the window coefficients in RAM */
    //errCode = HWA_configRam(handle, HWA_RAM_TYPE_WINDOW_RAM, (uint8_t *)win_data16, sizeof(win_data16), windowOffsetBytes);

 }

void HWAutil_configDopplerFFTSingleRangeBin(HWA_Handle handle,
                                uint32_t paramSetStartIdx,
                                uint32_t dopplerFftSize,
                                uint8_t numVirtualAnt,
                                uint32_t windowOffsetBytes,
                                uint8_t dmaTriggerSource,      
                                uint16_t hwaMemAzimSource,
                                uint16_t hwaMemAzimDest)
{
    HWA_ParamConfig hwaParamCfg;
    int32_t errCode = 0;

    memset( (void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));

    //Doppler FFT
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA; //HWA_TRIG_MODE_IMMEDIATE; //Immediate following first - in demo this should be HWA_TRIG_MODE_DFE
    hwaParamCfg.dmaTriggerSrc = dmaTriggerSource; //0; //Immediate following first - in demo this should be HWA_TRIG_MODE_DFE
    hwaParamCfg.accelMode = HWA_ACCELMODE_FFT; //do FFT

    hwaParamCfg.source.srcAddr = hwaMemAzimSource; // address is relative to start of MEM0
    hwaParamCfg.source.srcAcnt = dopplerFftSize - 1; //size in samples - 1

    hwaParamCfg.source.srcAIdx = numVirtualAnt * sizeof(uint32_t); //
    hwaParamCfg.source.srcBcnt = numVirtualAnt -1; //no iterations here
    hwaParamCfg.source.srcBIdx = sizeof(uint32_t); //should be dont care
    hwaParamCfg.source.srcShift = 0; //no shift
    hwaParamCfg.source.srcCircShiftWrap = 0; //no shift
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED; //signed
    hwaParamCfg.source.srcConjugate = 0; //no conjugate
    hwaParamCfg.source.srcScale = 0;
    hwaParamCfg.source.bpmEnable = 0; //bpm removal not enabled
    hwaParamCfg.source.bpmPhase = 0; //dont care

    hwaParamCfg.dest.dstAddr = hwaMemAzimDest; // address is relative to start of MEM0
    hwaParamCfg.dest.dstAcnt = dopplerFftSize - 1; //this is samples - 1
    hwaParamCfg.dest.dstAIdx = numVirtualAnt * sizeof(uint32_t); // 
    hwaParamCfg.dest.dstBIdx = sizeof(uint32_t) ; //should be dont care
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    hwaParamCfg.dest.dstConjugate = 0; //no conjugate
    hwaParamCfg.dest.dstScale = 8 ; 
    hwaParamCfg.dest.dstSkipInit = 0; // no skipping

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg.accelModeArgs.fftMode.fftSize = log2Approx(dopplerFftSize);
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0x3FF; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    hwaParamCfg.accelModeArgs.fftMode.windowEn = 1; //enabled
    hwaParamCfg.accelModeArgs.fftMode.windowStart = windowOffsetBytes; //start of window RAM
    hwaParamCfg.accelModeArgs.fftMode.winSymm = 1; //non-symmetric - in demo do we make this symmetric
    hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0; //fftsize is less than 1K
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;//HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples

    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    errCode = HWA_configParamSet(handle, paramSetStartIdx, &hwaParamCfg, NULL);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx, errCode);
        return;
    }

 }

void HWAutil_configCFAR(HWA_Handle handle,
                             uint32_t  paramSetStartIdx,
                             uint32_t numRangeBins,
                             uint32_t numDopplerBins,
                             uint32_t winLen,
                             uint32_t guardLen,
                             uint32_t noiseDivRightShift,
                             uint8_t peakGrouping,
                             uint8_t cyclicMode,
                             uint8_t nAvgMode,
                             uint16_t detObjectListSize,
                             uint8_t dmaTriggerSource,
                             uint8_t dmaDestChannel,
                             uint16_t hwaSourceBufOffset,
                             uint16_t hwaDestBufOffset
                             )
{
    HWA_ParamConfig hwaParamCfg;
    HWA_InterruptConfig     paramISRConfig;
    int32_t errCode = 0;


    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
    hwaParamCfg.dmaTriggerSrc = dmaTriggerSource;

    hwaParamCfg.accelModeArgs.cfarMode.peakGroupEn = peakGrouping;
    hwaParamCfg.accelMode = HWA_ACCELMODE_CFAR;

    //cfarInpMode = 1, cfarLogMode = 1, cfarAbsMode = 00b
    hwaParamCfg.accelModeArgs.cfarMode.operMode = HWA_CFAR_OPER_MODE_LOG_INPUT_REAL;


    hwaParamCfg.source.srcAddr = hwaSourceBufOffset;
    hwaParamCfg.source.srcAcnt = numRangeBins-1;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    hwaParamCfg.source.srcAIdx = numDopplerBins*2;
    hwaParamCfg.source.srcBIdx = sizeof(uint16_t);
    hwaParamCfg.source.srcBcnt = numDopplerBins-1;
    hwaParamCfg.source.srcScale = 8;

    hwaParamCfg.dest.dstAddr = hwaDestBufOffset;
    hwaParamCfg.dest.dstAcnt = detObjectListSize - 1;
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstAIdx = 8;
    hwaParamCfg.dest.dstBIdx = 4096;
    hwaParamCfg.dest.dstScale = 8;

    hwaParamCfg.accelModeArgs.cfarMode.numGuardCells = guardLen;
    hwaParamCfg.accelModeArgs.cfarMode.nAvgDivFactor = noiseDivRightShift;
    hwaParamCfg.accelModeArgs.cfarMode.cyclicModeEn = cyclicMode;
    hwaParamCfg.accelModeArgs.cfarMode.nAvgMode = nAvgMode;
    hwaParamCfg.accelModeArgs.cfarMode.numNoiseSamplesRight = winLen >> 1;
    hwaParamCfg.accelModeArgs.cfarMode.numNoiseSamplesLeft =  winLen >> 1;
    hwaParamCfg.accelModeArgs.cfarMode.outputMode = HWA_CFAR_OUTPUT_MODE_I_PEAK_IDX_Q_NEIGHBOR_NOISE_VAL;

    errCode = HWA_configParamSet(handle, paramSetStartIdx, &hwaParamCfg, NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx, errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /* Enable the DMA hookup to the last paramset */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = dmaDestChannel;  //EDMA channel to trigger to copy the data out
    paramISRConfig.cpu.callbackArg = NULL;
    errCode = HWA_enableParamSetInterrupt(handle, paramSetStartIdx, &paramISRConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enableParamSetInterrupt(PONG DMA) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

}

void HWAutil_configAngleEstAzimuth(HWA_Handle handle,
                                     uint32_t  paramSetStartIdx,
                                     uint32_t numVirtualAnt,
                                     uint32_t fftOutSize,
                                     uint32_t numIter,
                                     uint16_t hwaSourceBufOffset,
                                     uint16_t hwaDestBufOffset)
{
    HWA_ParamConfig hwaParamCfg;
    int32_t errCode = 0;

    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;

    hwaParamCfg.source.srcAddr = hwaSourceBufOffset;
    hwaParamCfg.source.srcAcnt = numVirtualAnt-1;
    hwaParamCfg.source.srcAIdx = sizeof(uint32_t);
    hwaParamCfg.source.srcBIdx = numVirtualAnt * sizeof(uint32_t);
    hwaParamCfg.source.srcBcnt = numIter-1;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcScale = 8;


    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftSize = log2Approx(fftOutSize);//assumes power of 2;
    hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.winSymm = 1;
    hwaParamCfg.accelModeArgs.fftMode.windowStart = 512; //do not care
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling

    hwaParamCfg.dest.dstAddr =  hwaDestBufOffset;
    hwaParamCfg.dest.dstAcnt = fftOutSize-1;
    hwaParamCfg.dest.dstAIdx = sizeof(uint16_t);//abs
    hwaParamCfg.dest.dstBIdx = fftOutSize * sizeof(uint16_t);
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg.dest.dstScale = 3;
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;

    errCode = HWA_configParamSet(handle, paramSetStartIdx, &hwaParamCfg, NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx, errCode);
        MmwDemo_debugAssert (0);
        return;
    }

}

void HWAutil_configAngleEstAzimuthElevation(HWA_Handle handle,
                                     uint32_t  paramSetStartIdx,
                                     uint32_t numVirtualAntAzim,
                                     uint32_t numVirtualAntElev,
                                     uint32_t fftOutSize,
                                     uint32_t numIter,
                                     uint16_t hwaSourceAzimBufOffset,
                                     uint32_t hwaSourceElevBufOffset,
                                     uint16_t hwaDestAzimAbsBufOffset,
                                     uint16_t hwaDestAzimCplxBufOffset,
                                     uint16_t hwaDestElevCplxBufOffset)
{
    HWA_ParamConfig hwaParamCfg;
    int32_t errCode = 0;

    /**************First Param set computes the complex values of the elevation-FFT **********/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;

    hwaParamCfg.source.srcAddr = (uint16_t) hwaSourceElevBufOffset;
    hwaParamCfg.source.srcAcnt = numVirtualAntElev-1;
    hwaParamCfg.source.srcAIdx = sizeof(uint32_t);
    hwaParamCfg.source.srcBIdx = numVirtualAntElev * sizeof(uint32_t);
    hwaParamCfg.source.srcBcnt = numIter-1;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcScale = 8;


    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg.accelModeArgs.fftMode.fftSize = log2Approx(fftOutSize);//assumes power of 2;
    hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling

    hwaParamCfg.dest.dstAddr =  (uint16_t) hwaDestElevCplxBufOffset;
    hwaParamCfg.dest.dstAcnt = fftOutSize-1;
    hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
    hwaParamCfg.dest.dstBIdx = fftOutSize * sizeof(uint32_t);
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.dest.dstScale = 3;

    errCode = HWA_configParamSet(handle, paramSetStartIdx, &hwaParamCfg, NULL);
    if (errCode != 0)
    {
       //System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx, errCode);
       MmwDemo_debugAssert (0);
       return;
    }

    /**************Second Param set computes the absolute values of the azimuth-FFT **********/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;

    hwaParamCfg.source.srcAddr = (uint16_t) hwaSourceAzimBufOffset;
    hwaParamCfg.source.srcAcnt = numVirtualAntAzim-1;
    hwaParamCfg.source.srcAIdx = sizeof(uint32_t);
    hwaParamCfg.source.srcBIdx = numVirtualAntAzim * sizeof(uint32_t);
    hwaParamCfg.source.srcBcnt = numIter-1;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcScale = 8;

    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftSize = log2Approx(fftOutSize);//assumes power of 2;
    hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling

    hwaParamCfg.dest.dstAddr =  (uint16_t) hwaDestAzimAbsBufOffset;
    hwaParamCfg.dest.dstAcnt = fftOutSize-1;
    hwaParamCfg.dest.dstAIdx = sizeof(uint16_t);//abs
    hwaParamCfg.dest.dstBIdx = fftOutSize * sizeof(uint16_t);
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg.dest.dstScale = 3;
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;

    errCode = HWA_configParamSet(handle, paramSetStartIdx + 1, &hwaParamCfg, NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx + 1, errCode);
        MmwDemo_debugAssert (0);
        return;
    }

   /**************Third Param set computes the complex values of the azimuth-FFT **********/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;

    hwaParamCfg.source.srcAddr = (uint16_t) hwaSourceAzimBufOffset;
    hwaParamCfg.source.srcAcnt = numVirtualAntAzim-1;
    hwaParamCfg.source.srcAIdx = sizeof(uint32_t);
    hwaParamCfg.source.srcBIdx = numVirtualAntAzim * sizeof(uint32_t);
    hwaParamCfg.source.srcBcnt = numIter-1;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcScale = 8;


    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg.accelModeArgs.fftMode.fftSize = log2Approx(fftOutSize);//assumes power of 2;
    hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling

    hwaParamCfg.dest.dstAddr =  (uint16_t) hwaDestAzimCplxBufOffset;
    hwaParamCfg.dest.dstAcnt = fftOutSize-1;
    hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
    hwaParamCfg.dest.dstBIdx = fftOutSize * sizeof(uint32_t);
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.dest.dstScale = 3;

    errCode = HWA_configParamSet(handle, paramSetStartIdx + 2, &hwaParamCfg, NULL);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramSetStartIdx + 2, errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}
