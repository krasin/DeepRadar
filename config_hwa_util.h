/**
 *   @file  config_hwa_util.h
 *
 *   @brief
 *      Hardware Accelerator Configuration Utility APIs.
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

#ifndef CONFIG_HWA_PARAMS_H
#define CONFIG_HWA_PARAMS_H

#include <ti/drivers/hwa/hwa.h>

#define HWAUTIL_NUM_PARAM_SETS_1D 4
#define HWAUTIL_NUM_PARAM_SETS_2D 8
#define HWAUTIL_NUM_PARAM_SETS_CFAR 1
#define HWAUTIL_NUM_PARAM_SETS_ANGLE 1
#define HWAUTIL_NUM_PARAM_SETS_2DDET_SINGLEBIN 1


#ifdef __cplusplus
extern "C" {
#endif

/** @brief Configures ParameterSet for Range FFT on ADC samples from DFE
 *
 *   @param[in] handle                  HWA driver handle
 *
 *   @param[in] paramSetStartIdx        HWA parameter set start index
 *
 *   @param[in] numAdcSamples           Number of ADC samples
 *
 *   @param[in] numRangeBins            Number of range bins (1st D FFT size)
 *
 *   @param[in] numRxAnt                Number of Rx antennas
 *
 *   @param[in] windowOffsetBytes       Window offset in bytes (for pre-FFT windowing)
 *
 *   @param[in] dmaTriggerSourcePing    DMA trigger source channel for Ping param sets
 *
 *   @param[in] dmaTriggerSourcePong    DMA trigger source channel for Pong param sets
 *
 *   @param[in] dmaDestChannelPing      DMA destination channel for Ping param sets
 *
 *   @param[in] dmaDestChannelPong      DMA destination channel for Pong param sets
 *
 *   @param[in] hwaMemAdcBufOffset      HWA memory offset ADC buffer
 *
 *   @param[in] hwaMemDestPingOffset    HWA memory offset for Ping destination
 *
 *   @param[in] hwaMemDestPongOffset    HWA memory offset for Pong destination
 *
 *   @param[in] hwaTriggerMode          HWA trigger mode
 *
 */
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
							uint8_t  hwaTriggerMode);

/** @brief Configures ParameterSet for Range FFT on ADC samples from DFE
 *
 *   @param[in] handle                  HWA driver handle
 *
 *   @param[in] paramSetStartIdx        HWA parameter set start index
 *
 *   @param[in] dopplerFftSize          Doppler FFT size
 *
 *   @param[in] numVirtualAnt           Number of Rx virtual antennas
 *
 *   @param[in] numRangeBinsPerIter     Number of range bins per iteration (currently has to be 2)
 *
 *   @param[in] windowOffsetBytes       Window offset in bytes (for pre-FFT windowing)
 *
 *   @param[in] dmaTriggerSourcePing    DMA trigger source channel for Ping param
 *                                      set (Ping param set calculates 2nd D FFT
 *                                      for two rows, (even pair of rows))
 *
 *   @param[in] dmaTriggerSourcePong    DMA trigger source channel for Pong param
 *                                      sets (Pong param set calculates 2nd D FFT
 *                                      for two rows, (odd pair of rows))
 *
 *   @param[in] dmaDestChannelPing      DMA destination channel for Ping param sets
 *
 *   @param[in] dmaDestChannelPong      DMA destination channel for Pong param sets
 *
 *   @param[in] hwaMemSourcePingOffset  HWA memory offset with Ping input data,
 *                                      two rows (range bins) of 1st D FFT symbols
 *
 *   @param[in] hwaMemSourcePongOffset  HWA memory offset with Pong input data,
 *                                      two rows (range bins) with 1st D FFT symbols
 *
 *   @param[in] hwaMemDestPingOffset    HWA memory offset with Ping output data,
 *                                      two rows (range bins) with 2nd D FFT complex symbols,
 *                                      followed by two rows (range bins) with summed log
 *                                      magnitude 2nd D FFT symbols across all antennas
 *
 *   @param[in] hwaMemDestPongOffset    HWA memory offset with Ping output data,
 *                                      two rows (range bins) with 2nd D FFT complex symbols,
 *                                      followed by two rows (range bins) with summed log
 *                                      magnitude 2nd D FFT symbols across all antennas
 *
 *   @param[in] option                  Option to seperate/combine 2DFFT and Log-Mag operation.
 *                                      By combining 2DFFT and Log-Mag operation, high precision can be achieved.
 *                                      0 - seperate, 1 - combine
 */
void HWAutil_configDopplerFFT(HWA_Handle handle,
                                uint32_t  paramSetStartIdx,
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
                                uint32_t option);

/** @brief Configures ParameterSet for single range in 2D FFT on 1D FFT data from radarCube
 *
 *   @param[in] handle                  HWA driver handle
 *
 *   @param[in] paramSetStartIdx        HWA parameter set start index
 *
 *   @param[in] dopplerFftSize          Doppler FFT size
 *
 *   @param[in] numVirtualAnt           Number of Rx virtual antennas
 *
 *   @param[in] windowOffsetBytes       Window offset in bytes (for pre-FFT windowing)
 *
 *   @param[in] dmaTriggerSource        DMA trigger source channel for Ping param
 *                                      set (Ping param set calculates 2nd D FFT
 *                                      for two rows, (even pair of rows))
 *
 *
 *   @param[in] hwaMemAzimSource        HWA memory offset with single range bin input data
 *                                      of 1D FFT symbols
 *
 *   @param[in] hwaMemAzimDest          HWA memory offset with single range bin 2D FFT output data
 *
 */
void HWAutil_configDopplerFFTSingleRangeBin(HWA_Handle handle,
                                uint32_t paramSetStartIdx,
                                uint32_t dopplerFftSize,
                                uint8_t numVirtualAnt,
                                uint32_t windowOffsetBytes,
                                uint8_t dmaTriggerSource,      
                                uint16_t hwaMemAzimSource,
                                uint16_t hwaMemAzimDest);

/** @brief Configures ParameterSet for CFAR detection (The CFAR detection runs along range bins).
 *
 *   @param[in] handle              HWA driver handle
 *
 *   @param[in] paramSetStartIdx    HWA parameter set start index
 *
 *   @param[in] numRangeBins        Number of range bins
 *
 *   @param[in] numDopplerBins      NUmber of Doppler bins
 *
 *   @param[in] winLen              window length in CFAR
 *
 *   @param[in] guardLen            guard length in CFAR
 *
 *   @param[in] noiseDivRightShift  noiseDiv factor expressed as right shift in CFAR
 *
 *   @param[in] peakGrouping        peakGrouping flag in CFAR
 *
 *   @param[in] cyclicMode          cyclic mode in CFAR
 *
 *   @param[in] nAvgMode            noise averaging mode in CFAR
 *
 *   @param[in] detObjectListSize   maximum size of list of detected objects
 *
 *   @param[in] dmaTriggerSource    DMA trigger source channel
 *
 *   @param[in] dmaDestChannel      DMA destination channel
 *
 *   @param[in] hwaSourceBufOffset  HWA memory offset with input data:
 *                                  log magnitude 2nd D FFT matrix
 *
 *   @param[in] hwaDestBufOffset    HWA memory offset with CFAR output results:
 *                                  List of detected objects as array of structures
 *                                  cfarDetOutput_t
 *
 */
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
                             );

/** @brief Configures ParameterSet for azimuth-FFT processing
 *
 *   @param[in] handle              HWA driver handle
 *
 *   @param[in] paramSetStartIdx    HWA parameter set start index
 *
 *   @param[in] numVirtualAnt       number of Virtual antennas (num_tx x num_rx)
 *
 *   @param[in] fftOutSize          Azimuth FFT length
 *
 *   @param[in] numIter             number of vectors on which angle-FFT has to be computed
 *
 *   @param[in] hwaSourceBufOffset  HWA memory offset with input data
 *
 *   @param[in] hwaDestBufOffset    HWA memory offset with computed angle-FFTs
 *
 *
*/
void HWAutil_configAngleEstAzimuth(HWA_Handle handle,
                                     uint32_t  paramSetStartIdx,
                                     uint32_t numVirtualAnt,
                                     uint32_t fftOutSize,
                                     uint32_t numIter,
                                     uint16_t hwaSourceBufOffset,
                                     uint16_t hwaDestBufOffset);

/** @brief Configures ParameterSet for azimuth/elevation-FFT processing
 *
 *   @param[in] handle              HWA driver handle
 *
 *   @param[in] paramSetStartIdx    HWA parameter set start index
 *
 *   @param[in] numVirtualAntAzim       number of Azimuth Virtual antennas (num_azimuth_tx x num_rx)
 *
 *   @param[in] numVirtualAntElev       number of Elevation Virtual antennas (num_elevation_tx x num_rx)
 *
 *   @param[in] fftOutSize          Azimuth FFT length
 *
 *   @param[in] numIter             number of vectors on which angle-FFT has to be computed
 *
 *   @param[in] hwaSourceAzimBufOffset  HWA memory offset with azimuth input data
 *
 *   @param[in] hwaSourceElevBufOffset  HWA memory offset with elevation input data
 *
 *   @param[in] hwaDestAzimAbsBufOffset   HWA memory offset with computed azimuth-FFTs (log magnitude values)
 *
 *   @param[in] hwaDestAzimCplxBufOffset   HWA memory offset with computed azimuth-FFTs (complex values)
 *
 *   @param[in] hwaDestElevCplxBufOffset   HWA memory offset with computed elevation-FFTs (complex values)
 *
*/
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
                                     uint16_t hwaDestElevCplxBufOffset);

/**@}*/
#ifdef __cplusplus
}
#endif

#endif



