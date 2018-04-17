/**
 *   @file  post_processing.c
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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>

#include "post_processing.h"

/* When elevation is enabled, FFT buffers can hold a maximum of 64 objects.
   A: Hwa memory size (M3) = 16K
   B: Azimuth FFT size = 64
   C: Complex sample size = 4
   Maximum number of objects = A / (B*C) = 64*/
#define MMW_MAX_OBJ_ELEVATION 64

/*!
 *  @brief  Function is calculate doppler compensation for Azimuth antennas.
 *
 *  @param[in]  in           Pointer to the Input Symbol 
 *
 *  @param[in]  out          Pointer to the Output Symbol
 *
 *  @param[in]  Cos          Cos value depending on doppler index
 *
 *  @param[in]  Sin          Sin value depending on doppler index
 *
 *  @return None
 *
 */
static void MmwDemo_dopplerComp
(
    cmplx16ImRe_t *in,
    cmplx16ImRe_t *out,
    float  Cos,
    float  Sin
)
{
    float           yRe, yIm;

    /* Rotate symbol (correct the phase) */
    yRe = in->real * Cos + in->imag * Sin;
    yIm = in->imag * Cos - in->real * Sin;
    out->real = (int16_t) yRe;
    out->imag = (int16_t) yIm;
}

/*!
 *  @brief  Function is calculate doppler compensation index for a given doppler index
 *
 *  @param[in]  dopplerIdx           Doppler index 
 *
 *  @param[in]  numDopplerBins       Number of Doppler bins
 *
 *  @param[in]  numTxAnt             Number of Tx Antennas
 *
 *  @return Doppler compensation index
 *
 */
static float MmwDemo_calcCompIdx
(
    uint16_t dopplerIdx,
    uint32_t numDopplerBins,
    uint32_t numTxAnt
)
{
    float      dopplerCompensationIdx;

    /* Doppler compensation index calculation */
    if (dopplerIdx >= numDopplerBins/2)
    {
        dopplerCompensationIdx =  ((int32_t) dopplerIdx - (int32_t) numDopplerBins);
    }
    else
    {
        dopplerCompensationIdx =  dopplerIdx;
    }
    
    /* Doppler phase correction is 1/2 or (1/3 in elevation case) of the phase between two chirps of the same antenna */
    dopplerCompensationIdx = dopplerCompensationIdx / (float) numTxAnt;
    if (dopplerCompensationIdx < 0)
    {
        dopplerCompensationIdx +=  (float) numDopplerBins;
    }

    return dopplerCompensationIdx;
}

void MmwDemo_dopplerCompensation
(
    uint32_t numDetectedObjects,
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
)
{
    uint32_t   index;
    uint32_t   j;
    uint16_t   dopplerIdx;
    float      dopplerCompensationIdx;
    
    for(index = 0; index < numDetectedObjects; index ++)
    {
        dopplerIdx = objOut->dopplerIdx;

        /* transfer data corresponding to azimuth virtual antennas (corresponding to chirp of antenna Tx0) */
        for(j = 0; j < numRxAnt; j++)
        {
            *dstPtrAzim++ = *srcPtrAzim++;
        }
        
        /* transfer data corresponding to azimuth virtual antennas (corresponding to chirp of antenna Tx1) */
        if (numVirtualAntAzim > numRxAnt)
        {
            float Cos,Sin;

            dopplerCompensationIdx = MmwDemo_calcCompIdx(dopplerIdx, numDopplerBins, numTxAnt);
            
            Cos = cos(2*PI_*dopplerCompensationIdx/numDopplerBins);
            Sin = sin(2*PI_*dopplerCompensationIdx/numDopplerBins);
            
            for(j = numRxAnt; j < numVirtualAntAzim; j++)
            {
                MmwDemo_dopplerComp((cmplx16ImRe_t *)srcPtrAzim++, (cmplx16ImRe_t *)dstPtrAzim++, Cos, Sin);
            }

            if (numVirtualAntElev > 0)
            {
                /* transfer data corresponding to elevation virtual antennas, (corresponding to chirp of antenna Tx2) */
                float Cos2, Sin2;
                /* Doppler phase shift is 2/3 */
                Cos2 = Cos * Cos - Sin * Sin;
                Sin2 = 2 * Cos * Sin;
                for(j = 0; j < numVirtualAntElev; j++)
                {
                    MmwDemo_dopplerComp((cmplx16ImRe_t *)srcPtrElev++, (cmplx16ImRe_t *)dstPtrElev++, Cos2, Sin2);
                }
            }
        }
        objOut++;
    }
}            

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
)
{
    uint32_t   index;
    uint32_t   j;
    float      dopplerCompensationIdx;
    float      Cos,Sin;
    float      Cos2, Sin2;

    dopplerCompensationIdx = MmwDemo_calcCompIdx(dopplerIdx, numDopplerBins, numTxAnt);

    Cos = cos(2*PI_*dopplerCompensationIdx/numDopplerBins);
    Sin = sin(2*PI_*dopplerCompensationIdx/numDopplerBins);
    
    /* Doppler phase shift is 2/3 */
    Cos2 = Cos * Cos - Sin * Sin;
    Sin2 = 2 * Cos * Sin;
    
    for(index = 0; index < numRangeBins; index ++)
    {
        /* transfer data corresponding to azimuth virtual antennas (corresponding to chirp of antenna Tx0) */
        for(j = 0; j < numRxAnt; j++)
        {
            /* Skip data, no compensation is needed */
            azimuthHeapMap++;
        }
        
        /* transfer data corresponding to azimuth virtual antennas (corresponding to chirp of antenna Tx1) */
        for(j = numRxAnt; j < numVirtualAntAzim; j++)
        {
            MmwDemo_dopplerComp(azimuthHeapMap, azimuthHeapMap, Cos, Sin);
            azimuthHeapMap++;
        }
        for(j = 0; j < numVirtualAntElev; j++)
        {
            MmwDemo_dopplerComp(azimuthHeapMap, azimuthHeapMap,       Cos2, Sin2);
            azimuthHeapMap++;
        }
    }
}            


void MmwDemo_XYZestimation(uint16_t *azimFFTPtr,
                           uint16_t *elevFFTPtr,
						   MmwDemo_DataPathObj *obj,
						   uint32_t objOutIndx,
						   uint32_t FFTIndx)
{
    uint32_t maxIdx;
	int32_t sMaxIdx;
    float temp;
    float Wx, Wz;
    float range;
    float x, y, z;
    float peakAzimRe, peakAzimIm, peakElevRe, peakElevIm;
	MmwDemo_detectedObj *objOut = &obj->objOut[0];
	uint32_t numAngleBins = obj->numAngleBins;
    uint32_t xyzOutputQFormat = obj->xyzOutputQFormat;
#define ONE_QFORMAT (1 << xyzOutputQFormat)

	maxIdx = obj->detObj2dAzimIdx[objOutIndx];
	
    if (obj->numVirtualAntElev > 0)
    {
        peakAzimIm = (float) ((int16_t) (*(azimFFTPtr + FFTIndx*2*numAngleBins + maxIdx*2)));
        peakAzimRe = (float) ((int16_t) (*(azimFFTPtr + FFTIndx*2*numAngleBins + maxIdx*2+1)));
        peakElevIm = (float) ((int16_t) (*(elevFFTPtr + FFTIndx*2*numAngleBins + maxIdx*2)));
        peakElevRe = (float) ((int16_t) (*(elevFFTPtr + FFTIndx*2*numAngleBins + maxIdx*2+1)));
    }

#ifdef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
    if (obj->rangeResolution > 0)
    {
        range = objOut[objOutIndx].rangeIdx * obj->rangeResolution;
    }
    else
    {
        objOut[objOutIndx].rangeIdx = (uint16_t) ((int32_t) obj->numRangeBins - (int32_t) objOut[objOutIndx].rangeIdx);
        range = objOut[objOutIndx].rangeIdx * -obj->rangeResolution;
    }
#else
    range = objOut[objOutIndx].rangeIdx * obj->rangeResolution;
#endif

    if(maxIdx > (numAngleBins/2 -1))
    {
        sMaxIdx = maxIdx - numAngleBins;
    }
    else
    {
        sMaxIdx = maxIdx;
    }

    Wx = 2 * (float) sMaxIdx / numAngleBins;
    x = range * Wx;

    if (obj->numVirtualAntElev > 0)
    {
        Wz = atan2(peakAzimIm * peakElevRe - peakAzimRe * peakElevIm,
                   peakAzimRe * peakElevRe + peakAzimIm * peakElevIm)/PI_ + (2 * Wx);
        if (Wz > 1)
        {
        	Wz = Wz - 2;
        }
        else if (Wz < -1)
        {
        	Wz = Wz + 2;
        }
        z = range * Wz;
        /*record wz for debugging/testing*/
        if(objOutIndx < MMW_MAX_ELEV_OBJ_DEBUG)
        {
            obj->detObjElevationAngle[objOutIndx] = Wz;
        }
        
    }
    else
    {
        z = 0;
    }

    temp = range*range -x*x -z*z;
    if (temp > 0)
    {
        y = sqrt(temp);
    }
    else
    {
        y = 0;
    }

    if(x < 0)
    {
        objOut[objOutIndx].x = (int16_t) (x * ONE_QFORMAT - 0.5);
    }
    else
    {
        objOut[objOutIndx].x = (int16_t) (x * ONE_QFORMAT + 0.5);
    }
    if(y < 0)
    {
        objOut[objOutIndx].y = (int16_t) (y * ONE_QFORMAT - 0.5);
    }
    else
    {
        objOut[objOutIndx].y = (int16_t) (y * ONE_QFORMAT + 0.5);
    }
    if(z < 0)
    {
        objOut[objOutIndx].z = (int16_t) (z * ONE_QFORMAT - 0.5);
    }
    else
    {
        objOut[objOutIndx].z = (int16_t) (z * ONE_QFORMAT + 0.5);
    }
}							

					
void angleEstimationAzimElev(uint16_t *azimFFTAbsPtr,
                            uint16_t *azimFFTPtr,
                            uint16_t *elevFFTPtr,
							MmwDemo_DataPathObj *obj)
{
    uint32_t i, j, maxVal = 0,maxIdx = 0, tempVal;
	MmwDemo_detectedObj *objOut = &obj->objOut[0];
	uint32_t numAngleBins = obj->numAngleBins;
	uint16_t *origAzimFFTAbsPtr;
    uint16_t maxNumObj = MMW_MAX_OBJ_OUT;
    
    if(obj->numVirtualAntElev > 0)
    {
        maxNumObj = MMW_MAX_OBJ_ELEVATION;
    }  
    
	
	/*Initialize the number of objects to the value received as input.
	  The value received as input will change if more than one object 
	  per azimuth is detected*/
	uint32_t numObjOut = obj->numObjOut;

    for(i=0; i < numObjOut; i++)
    {

        maxVal = 0;
		/* For this detected object, save starting point of "FFT absolute array",  
		   to be used later for the detection of the second azimuth on the same
		   range and doppler*/
		origAzimFFTAbsPtr = azimFFTAbsPtr;
        for(j=0; j < numAngleBins; j++)
        {
            tempVal = *azimFFTAbsPtr++;
            if(tempVal > maxVal)
            {
                maxVal = tempVal;
                maxIdx = j;
            }
        }

        objOut[i].peakVal = maxVal;
		obj->detObj2dAzimIdx[i] = maxIdx;
		
		/* Estimate x,y,z */
		MmwDemo_XYZestimation(azimFFTPtr, elevFFTPtr, obj, i, i);
						
        /* Multi peak azimuzth search?*/						
        if (obj->cliCfg->multiObjBeamFormingCfg.enabled)
        {
            uint32_t leftSearchIdx;
            uint32_t rightSearchIdx;
            uint32_t secondSearchLen;
            uint32_t iModAzimLen;
            uint32_t maxVal2;
            int32_t k;
			uint32_t t;
			uint32_t secondPeakObjIndex;
			uint16_t azimIdx=obj->detObj2dAzimIdx[i];
			uint16_t* azimuthMag = origAzimFFTAbsPtr;   
        
            /* Find right edge of the first peak */
            t = azimIdx;
            leftSearchIdx = (t + 1) & (numAngleBins-1);
            k = numAngleBins;
            while ((azimuthMag[t] >= azimuthMag[leftSearchIdx]) && (k > 0))
            {
                t = (t + 1) & (numAngleBins-1);
                leftSearchIdx = (leftSearchIdx + 1) & (numAngleBins-1);
                k--;
            }
        
            /* Find left edge of the first peak */
            t = azimIdx;
            rightSearchIdx = (t - 1) & (numAngleBins-1);
            k = numAngleBins;
            while ((azimuthMag[t] >= azimuthMag[rightSearchIdx]) && (k > 0))
            {
                t = (t - 1) & (numAngleBins-1);
                rightSearchIdx = (rightSearchIdx - 1) & (numAngleBins-1);
                k--;
            }
        
            secondSearchLen = ((rightSearchIdx - leftSearchIdx) & (numAngleBins-1)) + 1;
            /* Find second peak */
            maxVal2 = azimuthMag[leftSearchIdx];
            azimIdx = leftSearchIdx;
            for (t = leftSearchIdx; t < (leftSearchIdx + secondSearchLen); t++)
            {
                iModAzimLen = t & (numAngleBins-1);
                if (azimuthMag[iModAzimLen] > maxVal2)
                {
                    azimIdx = iModAzimLen;
                    maxVal2 = azimuthMag[iModAzimLen];
                }
            }
            /* Is second peak greater than threshold? */
            if ( (maxVal2 >( ((uint32_t)(maxVal * obj->cliCfg->multiObjBeamFormingCfg.multiPeakThrsScal)))) && (obj->numObjOut < maxNumObj) )
            {
                /* Second peak detected! Add it to the end of the list */
				secondPeakObjIndex = obj->numObjOut;
                obj->numObjOut++;

				/* Update second peak with same range and doppler as first peak*/
                objOut[secondPeakObjIndex].dopplerIdx = objOut[i].dopplerIdx;
                objOut[secondPeakObjIndex].rangeIdx   = objOut[i].rangeIdx;
                /* Update second peak with peak value*/
                objOut[secondPeakObjIndex].peakVal = maxVal2;
				
                /* Save azimuth index */
	            obj->detObj2dAzimIdx[secondPeakObjIndex] = azimIdx;
				
		        /* Estimate x,y,z for second peak */
		        MmwDemo_XYZestimation(azimFFTPtr, elevFFTPtr, obj,
		        					  secondPeakObjIndex, i/*FFTIndx for second peak is the same one used for the first peak*/);        
            }
        }		
    }
}
