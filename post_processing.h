/**
 *   @file  post_processing.h
 *
 *   @brief
 *      Routines for post processing (detection/angle estimation) of data output
 *      by the Hardware Accelerator.
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

#ifndef POST_PROCESSING_H
#define POST_PROCESSING_H

#include "config_hwa_util.h"
#include "data_path.h"
#include <ti/demo/io_interface/mmw_config.h>



/*!
 *  @brief  Function is executed after the HWA CFAR algorithm. It reduces the
 *  number of detected objects performed by CFAR algorithm. It selects only
 *  those objects with peaks greater than its neighbors.
 *
 *
 *  @param[in]  numDetectedObjects  Number of detected objects by HWA CFAR algorithm
 *
 *  @param[in]  detBuffptr          Pointer to range-doppler matrix which was
 *                                  used by CFAR alogorithm to generate detected
 *                                  objects, size number of range bins times number
 *                                  of doppler bins.
 *
 *  @param[in]  cfarObjptr          Pointer to the output structure generated
 *                                  by CFAR algorithm, contains list of detected objects.
 *
 *  @param[in]  fftBuffptr          Pointer to 2nd D FFT buffer with complex symbols. 
 *                                  The size of buffer is 
 *                                  numRangeBins * numDopplerBins * (numVirtualAntAzim + numVirtualAntElev).
 *
 *  @param[out]  objOut             Output pointer to the structure with with selected objects. 
 *                                  The function fills range and doppler index of the object.
 *
 *  @param[out]  dstPtrAzim         Output pointer (local memory of HWA) filled with antenna 
 *                                  symbols of selected objects for azimuth calculation.
 *
 *  @param[out]  dstPtrElev         Output pointer (local memory of HWA) filled with antenna 
 *                                  symbols of selected objects for elevation calculation.
 *
 *  @param[in]  numTxAnt            Number of Tx antennas
 *
 *  @param[in]  numRxAnt            Number of Rx antennas
 *
 *  @param[in]  numVirtualAntAzim   Number of virtual azimuth Rx antennas
 *
 *  @param[in]  numVirtualAntElev   Number of virtual elevation Rx antennas
 *
 *  @param[in]  numDopplerBins      Number of Doppler bins
 *
 *  @param[in]  numRangeBins        NUmber of Range bins
 *
 *  @param[in]  minRangeIdx         minimum range index to be exported
 *
 *  @param[in]  maxRangeIdx         maximum range index to be exported
 *
 *
 *  @param[in]  groupInDopplerDirection    peak grouping in Doppler direction 0-disabled, 1-enabled
 *
 *  @param[in]  groupInRangeDirection      peak grouping in Range direction 0-disabled, 1-enabled
 *
 *  @return numObjOut Number of selected objects
 *
 */
extern uint32_t postDetectionProcessing(uint32_t numDetectedObjects,
                                    uint16_t* detBuffptr,
                                    cfarDetOutput_t*  cfarObjptr,
                                    uint32_t* fftBuffptr,
                                    MmwDemo_detectedObj *objOut,
                                    uint32_t *dstPtrAzim,
                                    uint32_t *dstPtrElev,
                                    uint32_t numTxAnt,
                                    uint32_t numRxAnt,
                                    uint32_t numVirtualAntAzim,
                                    uint32_t numVirtualAntElev,
                                    uint32_t numDopplerBins,
                                    uint32_t numRangeBins,
                                    uint32_t minRangeIdx,
                                    uint32_t maxRangeIdx,
                                    uint32_t groupInDopplerDirection,
                                    uint32_t groupInRangeDirection);

/*!
 *  @brief  Function is executed after the HWA CFAR algorithm or Peak grouping if enabled. It compensates the 
 *          Siganl on Azimuth and Elevation attennas due to signal transmit time shift from different TX antennas.
 *
 *  @param[in]  numDetectedObjects Number of detected objects by HWA CFAR algorithm
 *
 *  @param[in]  srcPtrAzim          Input pointer (local memory of HWA) filled with antenna symbols of selected 
 *                                  objects for azimuth calculation.
 *
 *  @param[in]  srcPtrElev          Input pointer (local memory of HWA) filled with antenna symbols of selected 
 *                                  objects for elevation calculation.
 *
 *  @param[in]  objOut              Output pointer to the structure with with selected objects. The function fills 
 *                                  range and doppler index of the object.
 *
 *  @param[out]  dstPtrAzim         Output pointer (local memory of HWA) filled with antenna symbols of selected 
 *                                  objects for azimuth calculation.
 *
 *  @param[out]  dstPtrElev         Output pointer (local memory of HWA) filled with antenna symbols of selected 
 *                                  objects for elevation calculation.
 *
 *  @param[in]  numTxAnt            Number of Tx antennas
 *
 *  @param[in]  numRxAnt            Number of Rx antennas
 *
 *  @param[in]  numVirtualAntAzim   Number of virtual Rx antennas
 *
 *  @param[in]  numVirtualAntElev   Number of virtual Rx antennas
 *
 *  @param[in]  numDopplerBins      Number of Doppler bins
 *
 *  @return None
 *
 */
extern void MmwDemo_dopplerCompensation
(   uint32_t numDetectedObjects,
    uint32_t *srcPtrAzim,
    uint32_t *srcPtrElev,
    MmwDemo_detectedObj *objOut,
    uint32_t *dstPtrAzim,
    uint32_t *dstPtrElev,
    uint32_t numTxAnt,
    uint32_t numRxAnt,
    uint32_t numVirtualAntAzim,
    uint32_t numVirtualAntElev,
    uint32_t numDopplerBins
);

/*!
 *  @brief  Function is executed after the HWA CFAR algorithm or Peak grouping if enabled. It compensates the 
 *          Siganl on Azimuth and Elevation attennas due to signal transmit time shift from different TX antennas.
 *
 *  @param[in]  azimuthHeapMap      Pointer to Azimuth Heap map that filled with antenna symbols.
 *                                  The compensation will be applied in place.
 *
 *  @param[in]  dopplerIdx          Doppler Index used for doppler compensation
 *
 *  @param[in]  numTxAnt            Number of Tx antennas
 *
 *  @param[in]  numRxAnt            Number of Rx antennas
 *
 *  @param[in]  numVirtualAntAzim   Number of virtual Rx antennas
 *
 *  @param[in]  numVirtualAntElev   Number of virtual Rx antennas
 *
 *  @param[in]  numDopplerBins      Number of Doppler bins
 *
 *  @param[in]  numRangeBins        Number of Range bins
 *
 *  @return None
 *
 */
void MmwDemo_azimHeapMapDopplerCompensation
(
    cmplx16ImRe_t *azimuthHeapMap,
    uint16_t dopplerIdx,
    uint32_t numTxAnt,
    uint32_t numRxAnt,
    uint32_t numVirtualAntAzim,
    uint32_t numVirtualAntElev,
    uint32_t numDopplerBins,
    uint32_t numRangeBins
);

/*!
 *  @brief  Function is executed after the HWA CFAR algorithm. It reduces the
 *  number of detected objects performed by CFAR algorithm. It selects only
 *  those objects with peaks greater than its neighbors.
 *
 *
 *  @param[in]  numDetectedObjects Number of detected objects by HWA CFAR algorithm
 *
 *  @param[in]  detBuffptr          Pointer to range-doppler matrix which was
 *                                  used by CFAR alogorithm to generate detected
 *                                  objects, size number of range bins times number
 *                                  of doppler bins.
 *
 *  @param[in]  cfarObjptr          Pointer to the output structure generated
 *                                  by CFAR algorithm, contains list of detected objects.
 *
 *  @param[in]  fftBuffptr          Pointer to 2nd D FFT buffer with complex symbols. The size of buffer is 
 *                                  numRangeBins * numDopplerBins * (numVirtualAntAzim + numVirtualAntElev).
 *
 *  @param[out]  objOut             Output pointer to the structure with with selected objects. The function 
 *                                  fills range and doppler index of the object.
 *
 *  @param[out]  peakGrpAzimOut     Output pointer (local memory of HWA) filled with antenna symbols of selected 
 *                                  objects for azimuth calculation.
 *
 *  @param[out]  peakGrpElevOut     Output pointer (local memory of HWA) filled with antenna symbols of selected 
 *                                  objects for elevation calculation.
 *
 *  @param[in]  numVirtualAntAzim   Number of virtual Rx antennas
 *
 *  @param[in]  numVirtualAntElev   Number of virtual Rx antennas
 *
 *  @param[in]  numDopplerBins      Number of Doppler bins
 *
 *  @param[in]  minRangeIdx         Minimum range index to be exported
 *
 *  @param[in]  maxRangeIdx         Maximum range index to be exported
 *
 *
 *  @param[in]  groupInDopplerDirection    peak grouping in Doppler direction 0-disabled, 1-enabled
 *
 *  @param[in]  groupInRangeDirection      peak grouping in Range direction 0-disabled, 1-enabled
 *
 *  @return numObjOut Number of selected objects
 *
 */
extern uint32_t MmwDemo_peakGrouping(uint32_t numDetectedObjects,
                                uint16_t* detBuffptr,
                                cfarDetOutput_t*  cfarObjptr,
                                uint32_t* fftBuffptr,
                                MmwDemo_detectedObj *objOut,
                                uint32_t *peakGrpAzimOut,
                                uint32_t *peakGrpElevOut,
                                uint32_t numVirtualAntAzim,
                                uint32_t numVirtualAntElev,
                                uint32_t numDopplerBins,
                                uint32_t minRangeIdx,
                                uint32_t maxRangeIdx,
                                uint32_t groupInDopplerDirection,
                                uint32_t groupInRangeDirection);

/*!
 *  @brief  Function finds for each detected object the peak in the azimut FFT
 *          and the based on complex values of azimuth and elevation FFT
 *          corresponding to peak position, calculates the x,y,z coordinates
 *          of the object
 *
 *
 *  @param[in]  azimFFTAbsPtr      Pointer to array with magnitude values of azimuth FFTs of
 *                                 detected objects. The length of each FFT is numAngleBins.
 *                                 The size of input array is  numAngleBins * numObjOut
 *
 *  @param[in]  azimFFTPtr         Pointer to array with complex values of azimuth FFTs of
 *                                 detected objects. The length of each FFT is numAngleBins.
 *                                 The size of input array is  numAngleBins * numObjOut
 *
 *  @param[in]  elevFFTPtr         Pointer to array with complex values of elevation FFTs of
 *                                 detected objects. The length of each FFT is numAngleBins.
 *                                 The size of input array is  numAngleBins * numObjOut
 *
 *  @param[in]  obj                Pointer to Data path object.
 *
 */
extern void angleEstimationAzimElev(uint16_t* azimFFTAbsPtr,
                                    uint16_t* azimFFTPtr,
                                    uint16_t * elevFFTPtr,
                                    MmwDemo_DataPathObj *obj);


/*!
 *   @brief     Calculate noise energy (diagnostic only)
 *              This  routine calculates the noise  energy
 *              Assumes a predominantly static scene- and averages
 *              the values in the high velocity bins to estimate noise
 *              energy
 *
 *  @param[in] radarCubedMemPtr    Pointer to 2nd D FFT buffer with complex symbols.
 *                                  The size of buffer is numRangeBins *
 *                                  numDopplerBins * (numVirtualAntAzim + numVirtualAntElev)
 *
 *  @param[in] numDopplerBins      Number of Doppler bins
 *
 *  @param[in] numRangeBins        Number of range bins
 *
 *  @param[in] numVirtualAnt       Number of virtual antennas
 *
 */
extern uint32_t calcNoiseFloor(uint32_t *radarCubedMemPtr,
                        uint32_t numDopplerBins,
                        uint32_t numRangeBins,
                        uint32_t numVirtualAnt);

#endif
