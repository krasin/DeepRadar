/**
 *   @file  data_path.c
 *
 *   @brief
 *      Implements Data path processing functionality.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

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
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/HeapMin.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#define DebugP_ASSERT_ENABLED 1
#include <ti/drivers/osal/DebugP.h>
#include <assert.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/demo/utils/rx_ch_bias_measure.h>

#include "config_edma_util.h"
#include "config_hwa_util.h"
#include "post_processing.h"

#include "data_path.h"
#include "mmw.h"


int32_t MmwDemo_config2D_EDMA(MmwDemo_DataPathObj *obj);
void MmwDemo_config2D_HWA(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathTrigger2D(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathWait2D(MmwDemo_DataPathObj *obj);

int32_t MmwDemo_config1D_EDMA(MmwDemo_DataPathObj *obj);
int32_t MmwDemo_configCFAR_EDMA(MmwDemo_DataPathObj *obj);
void MmwDemo_configCFAR_HWA(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathTriggerCFAR(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathWaitCFAR(MmwDemo_DataPathObj *obj);

void MmwDemo_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode);

void MmwDemo_dataPathHwaDoneIsrCallback(void * arg);

#pragma DATA_SECTION(gMmwHwaMemBuf, ".hwaBufs");
/*! HWA memory buffer to produce the M0,M1,M2,M3 partition addresses
    and link to the HWA section in the linker command file. */
mmwHwaBuf_t gMmwHwaMemBuf[MMW_HWA_NUM_MEM_BUFS];


/**
 *  @b Description
 *  @n
 *      Function to generate a single FFT window sample.
 *
 *  @param[out] win Pointer to output calculated window sample.
 *  @param[in]  winIndx Index of window to generate sample at.
 *  @param[in]  phi Pre-calculated constant by caller as (2*pi/(window length - 1)).
 *  @param[in]  winType Type of window, one of @ref MMW_WIN_BLACKMAN, @ref MMW_WIN_HANNING,
 *              or @ref MMW_WIN_RECT.
 *  @retval none.
 */
static inline MmwDemo_genWindow(uint32_t *win, uint32_t winIndx, float phi, uint32_t winType)
{
    if(winType == MMW_WIN_BLACKMAN)
    {
        //Blackman window
        float a0 = 0.42;
        float a1 = 0.5;
        float a2 = 0.08;
        *win = (uint32_t) ((ONE_Q17 * (a0 - a1*cos(phi * winIndx) +
            a2*cos(2 * phi * winIndx))) + 0.5); //in Q17
        if(*win >= ONE_Q17)
        {
            *win = ONE_Q17 - 1;
        }
    }
    else if(winType == MMW_WIN_HANNING)
    {
        //Hanning window
        *win = (uint32_t) ((ONE_Q17 * 0.5* (1 - cos(phi * winIndx))) + 0.5); //in Q17
        if(*win >= ONE_Q17)
        {
            *win = ONE_Q17 - 1;
        }
    }
    else if(winType == MMW_WIN_RECT)
    {
        //Rectangular window
        *win = (uint32_t) (ONE_Q17/16); //in Q17
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA transfer completion call back function as per EDMA API.
 *      Depending on the programmed transfer completion codes,
 *      posts the corresponding done/completion semaphore.
 *      Per current design, a single semaphore could have been used as the
 *      1D, 2D and CFAR stages are sequential, this code gives some flexibility
 *      if some design change in future.
 */
void MmwDemo_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode)
{
    MmwDemo_DataPathObj *obj = (MmwDemo_DataPathObj *)arg;

    switch (transferCompletionCode)
    {
        case MMWDEMO_EDMA_TRANSFER_COMPLETION_CODE_1D_DONE:
            Semaphore_post(obj->EDMA_1Ddone_semHandle);
        break;

        case MMWDEMO_EDMA_TRANSFER_COMPLETION_CODE_2D_DONE:
            Semaphore_post(obj->EDMA_2Ddone_semHandle);
        break;

        case MMWDEMO_EDMA_TRANSFER_COMPLETION_CODE_CFAR_DONE:
            Semaphore_post(obj->EDMA_CFARdone_semHandle);
        break;

        default:
            MmwDemo_debugAssert(0);
        break;
    }
}

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function as per HWA API.
 *      Depending on the programmed transfer completion codes,
 *      posts HWA done semaphore.
 */
void MmwDemo_dataPathHwaDoneIsrCallback(void * arg)
{
    Semaphore_Handle       semHandle;

    if (arg != NULL) {
        semHandle = (Semaphore_Handle)arg;
        Semaphore_post(semHandle);
    }
}

/**
 *  @b Description
 *  @n
 *      Configures all 2D processing related EDMA configuration.
 *
 *  @param[in] obj Pointer to data path object
 *  @retval EDMA error code, see EDMA API.
 */
int32_t MmwDemo_config2D_EDMA(MmwDemo_DataPathObj *obj)
{
    int32_t errorCode = EDMA_NO_ERROR;
    EDMA_Handle handle = obj->edmaHandle;
    HWA_SrcDMAConfig dmaConfig;
    uint32_t sizeOfAbsTransfer = obj->numDopplerBins * obj->numRangeBinsPerTransfer;
    uint32_t sizeOfAbsTransferBytes = sizeOfAbsTransfer * sizeof(uint16_t);

    uint32_t sizeOfComplexTransfer = obj->numDopplerBins * obj->numVirtualAntennas * obj->numRangeBinsPerTransfer;
    uint32_t sizeOfComplexTransferBytes = sizeOfComplexTransfer * sizeof(uint32_t);

    /**************PROGRAM DMA'S FOR PING**************************************/

    if(obj->datapathChainSel == DATA_PATH_CHAIN_SEPARATE_LOGMAG)
    {
          /*************************************************************************
          *  PROGRAM DMA channel  to transfer 2D FFT data from accelerator output
             buffer (ping) to L3
          *************************************************************************/
        errorCode = EDMAutil_configHwaContiguous(handle,
            MMW_EDMA_2D_PING_CH_ID,  //chId,
            true, //isEventTriggered
            MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID1, //linkChId,
            MMW_EDMA_2D_PING_CHAIN_CH_ID1,       //chainChId,
            (uint32_t*)SOC_translateAddress((uint32_t)MMW_HWA_2D_OUT_PING,SOC_TranslateAddr_Dir_TO_EDMA,NULL),      //*pSrcAddress,
            (uint32_t*)SOC_translateAddress((uint32_t)obj->radarCube,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //*pDestAddress,
            sizeOfComplexTransferBytes,                   // numBytes,
            obj->numRangeBins /(2*obj->numRangeBinsPerTransfer), //numBlocks,
            0,                                   //srcIncrBytes,
            sizeOfComplexTransferBytes * 2,   //dstIncrBytes,
            true,   //isIntermediateChainingEnabled,
            true,   //isFinalChainingEnabled,
            false, //isTransferCompletionEnabled
            NULL,  //transferCompletionCallbackFxn
            NULL);

        if (errorCode != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /***************************************************************************
         *  PROGRAM DMA channel  to transfer 2D abs data from accelerator output
         *  buffer (ping) to L3
         **************************************************************************/
        errorCode = EDMAutil_configHwaContiguous(handle,
            MMW_EDMA_2D_PING_CHAIN_CH_ID1,        //chId,
            false, //isEventTriggered
            MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID2,  //linkChId,
            MMW_EDMA_2D_PING_CHAIN_CH_ID2,        //chainChId,
            (uint32_t*)(SOC_translateAddress((uint32_t)MMW_HWA_2D_OUT_PING,
                                               SOC_TranslateAddr_Dir_TO_EDMA,NULL)+ sizeOfComplexTransferBytes),       //*pSrcAddress,
            (uint32_t*) SOC_translateAddress((uint32_t)obj->rangeDopplerLogMagMatrix,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //*pDestAddress,
            sizeOfAbsTransferBytes,       // numBytes,
            obj->numRangeBins/(2*obj->numRangeBinsPerTransfer), //numBlocks,
            0,                             //srcIncrBytes,
            sizeOfAbsTransferBytes * 2,    //dstIncrBytes,
            true,   //isIntermediateChainingEnabled,
            false,  //isFinalChainingEnabled,
            false, //isTransferCompletionEnabled
            NULL,  //transferCompletionCallbackFxn
            NULL);

        if (errorCode != EDMA_NO_ERROR)
        {
            goto exit;
        }
    }
    else
    {
        /***************************************************************************
         *  PROGRAM DMA channel  to transfer 2D abs data from accelerator output
         *  buffer (ping) to L3
         **************************************************************************/
        errorCode = EDMAutil_configHwaContiguous(handle,
            MMW_EDMA_2D_PING_CH_ID,        //chId,
            true, //isEventTriggered
            MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID1,  //linkChId,
            MMW_EDMA_2D_PING_CHAIN_CH_ID2,        //chainChId,
            (uint32_t*)(SOC_translateAddress((uint32_t)MMW_HWA_2D_OUT_PING,
                                               SOC_TranslateAddr_Dir_TO_EDMA,NULL)+ sizeOfComplexTransferBytes),       //*pSrcAddress,
            (uint32_t*) SOC_translateAddress((uint32_t)obj->rangeDopplerLogMagMatrix,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //*pDestAddress,
            sizeOfAbsTransferBytes,       // numBytes,
            obj->numRangeBins/(2*obj->numRangeBinsPerTransfer), //numBlocks,
            0,                             //srcIncrBytes,
            sizeOfAbsTransferBytes * 2,    //dstIncrBytes,
            true,   //isIntermediateChainingEnabled,
            false,  //isFinalChainingEnabled,
            false, //isTransferCompletionEnabled
            NULL,  //transferCompletionCallbackFxn
            NULL);

        if (errorCode != EDMA_NO_ERROR)
        {
            goto exit;
        }
    }

      /******************************************************************************************
      *  PROGRAM DMA channel  to transfer data from L3 to accelerator input buffer (ping)
      ******************************************************************************************/
    errorCode = EDMAutil_configHwaContiguous(handle,
        MMW_EDMA_2D_PING_CHAIN_CH_ID2,         //chId,
        false, //isEventTriggered
        MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID3,   //linkChId,
        MMW_EDMA_2D_PING_CHAIN_CH_ID3,         //chainChId,
        (uint32_t*)SOC_translateAddress((uint32_t)obj->radarCube,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //*pSrcAddress,
        (uint32_t*)SOC_translateAddress((uint32_t)MMW_HWA_2D_INP_PING,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //*pDestAddress,
        sizeOfComplexTransferBytes,   // numBytes,
        obj->numRangeBins/(2*obj->numRangeBinsPerTransfer), //numBlocks,
        sizeOfComplexTransferBytes * 2, //srcIncrBytes,
        0,                              //dstIncrBytes,
        true,  //isIntermediateChainingEnabled,
        true,  //isFinalChainingEnabled,
        false, //isTransferCompletionEnabled;
        NULL, //transferCompletionCallbackFxn
        NULL);

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

    HWA_getDMAconfig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SOURCE_2D_PING, &dmaConfig);

    errorCode = EDMAutil_configHwaOneHotSignature(handle,
        MMW_EDMA_2D_PING_CHAIN_CH_ID3, //chId,
        false, //isEventTriggered
        (uint32_t*)SOC_translateAddress((uint32_t)dmaConfig.srcAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //pSrcAddress
        (uint32_t*)SOC_translateAddress((uint32_t)dmaConfig.destAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pDestAddress
        dmaConfig.aCnt,
        dmaConfig.bCnt,
        dmaConfig.cCnt,
        MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID4); //linkChId

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /**************PROGRAM DMA'S FOR PONG********************************************************/

    if(obj->datapathChainSel == DATA_PATH_CHAIN_SEPARATE_LOGMAG)
    {
          /******************************************************************************************
          *  PROGRAM DMA channel  to transfer 2D FFT data from accelerator output buffer (pong) to L3
          ******************************************************************************************/

        errorCode = EDMAutil_configHwaContiguous(handle,
            MMW_EDMA_2D_PONG_CH_ID,               //chId,
            true, //isEventTriggered
            MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID1,  //linkChId,
            MMW_EDMA_2D_PONG_CHAIN_CH_ID1,        //chainChId,
            (uint32_t*)(SOC_translateAddress((uint32_t)MMW_HWA_2D_OUT_PONG,SOC_TranslateAddr_Dir_TO_EDMA,NULL)), //*pSrcAddress,
            (uint32_t*)SOC_translateAddress((uint32_t)(obj->radarCube + sizeOfComplexTransfer),SOC_TranslateAddr_Dir_TO_EDMA,NULL),
            sizeOfComplexTransferBytes,                   // numBytes,
            obj->numRangeBins/(2*obj->numRangeBinsPerTransfer),  //numBlocks,
            0,                                 //srcIncrBytes,
            sizeOfComplexTransferBytes * 2, //dstIncrBytes,
            true,  //isIntermediateChainingEnabled,
            true,  //isFinalChainingEnabled,
            false, //isTransferCompletionEnabled;
            NULL,  //transferCompletionCallbackFxn
            NULL);

        if (errorCode != EDMA_NO_ERROR)
        {
            goto exit;
        }

          /******************************************************************************************
          *  PROGRAM DMA channel  to transfer 2D abs data from accelerator output buffer (pong) to L3
          ******************************************************************************************/
        errorCode = EDMAutil_configHwaContiguous(handle,
            MMW_EDMA_2D_PONG_CHAIN_CH_ID1,       //chId,
            false, //isEventTriggered
            MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID2, //linkChId,
            MMW_EDMA_2D_PONG_CHAIN_CH_ID2,       //chainChId,
            (uint32_t*)(SOC_translateAddress((uint32_t)MMW_HWA_2D_OUT_PONG,
                                               SOC_TranslateAddr_Dir_TO_EDMA,NULL)+ sizeOfComplexTransferBytes),//*pSrcAddress,
            (uint32_t*)(SOC_translateAddress((uint32_t)(obj->rangeDopplerLogMagMatrix + sizeOfAbsTransfer),
                                             SOC_TranslateAddr_Dir_TO_EDMA,NULL)),    //*pDestAddress,
            sizeOfAbsTransferBytes,                      //numBytes,
            obj->numRangeBins/(2*obj->numRangeBinsPerTransfer), //numBlocks,
            0,                             //srcIncrBytes,
            sizeOfAbsTransferBytes * 2, //dstIncrBytes,
            true,  //isIntermediateChainingEnabled,
            false, //isFinalChainingEnabled,
            true,  //isTransferCompletionEnabled;
            MmwDemo_EDMA_transferCompletionCallbackFxn, //transferCompletionCallbackFxn
            (uintptr_t)obj);

        if (errorCode != EDMA_NO_ERROR)
        {
            goto exit;
        }
    }
    else
    {
          /******************************************************************************************
          *  PROGRAM DMA channel  to transfer 2D abs data from accelerator output buffer (pong) to L3
          ******************************************************************************************/
        errorCode = EDMAutil_configHwaContiguous(handle,
            MMW_EDMA_2D_PONG_CH_ID,       //chId,
            true, //isEventTriggered
            MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID1, //linkChId,
            MMW_EDMA_2D_PONG_CHAIN_CH_ID2,       //chainChId,
            (uint32_t*)(SOC_translateAddress((uint32_t)MMW_HWA_2D_OUT_PONG,
                                               SOC_TranslateAddr_Dir_TO_EDMA,NULL)+ sizeOfComplexTransferBytes),//*pSrcAddress,
            (uint32_t*)(SOC_translateAddress((uint32_t)(obj->rangeDopplerLogMagMatrix + sizeOfAbsTransfer),
                                             SOC_TranslateAddr_Dir_TO_EDMA,NULL)),    //*pDestAddress,
            sizeOfAbsTransferBytes,                      //numBytes,
            obj->numRangeBins/(2*obj->numRangeBinsPerTransfer), //numBlocks,
            0,                             //srcIncrBytes,
            sizeOfAbsTransferBytes * 2, //dstIncrBytes,
            true,  //isIntermediateChainingEnabled,
            false, //isFinalChainingEnabled,
            true,  //isTransferCompletionEnabled;
            MmwDemo_EDMA_transferCompletionCallbackFxn, //transferCompletionCallbackFxn
            (uintptr_t)obj);

        if (errorCode != EDMA_NO_ERROR)
        {
            goto exit;
        }
    }
    /******************************************************************************************
     *  PROGRAM DMA channel  to transfer data from L3 to accelerator input buffer (pong)
     ******************************************************************************************/
    errorCode = EDMAutil_configHwaContiguous(handle,
        MMW_EDMA_2D_PONG_CHAIN_CH_ID2,       //chId,
        false, //isEventTriggered
        MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID3, //linkChId,
        MMW_EDMA_2D_PONG_CHAIN_CH_ID3,       //chainChId,
        (uint32_t*)SOC_translateAddress((uint32_t)(obj->radarCube + sizeOfComplexTransfer),SOC_TranslateAddr_Dir_TO_EDMA,NULL), //*pSrcAddress,
        (uint32_t*)SOC_translateAddress((uint32_t)MMW_HWA_2D_INP_PONG,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //*pDestAddress,
        sizeOfComplexTransferBytes,                  //numBytes,
        obj->numRangeBins/(2*obj->numRangeBinsPerTransfer), //numBlocks,
        sizeOfComplexTransferBytes * 2, //srcIncrBytes,
        0,                                 //dstIncrBytes,
        true,  //isIntermediateChainingEnabled,
        true,  //isFinalChainingEnabled,
        false, //isTransferCompletionEnabled;
        NULL,  //transferCompletionCallbackFxn
        NULL);

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

    HWA_getDMAconfig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SOURCE_2D_PONG, &dmaConfig);

    errorCode = EDMAutil_configHwaOneHotSignature(handle,
        MMW_EDMA_2D_PONG_CHAIN_CH_ID3, //chId,
        false, //isEventTriggered
        (uint32_t*)SOC_translateAddress(dmaConfig.srcAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //pSrcAddress
        (uint32_t*)SOC_translateAddress(dmaConfig.destAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pDestAddress
        dmaConfig.aCnt,
        dmaConfig.bCnt,
        dmaConfig.cCnt,
        MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID4); //linkChId

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      Common configuration of data path processing, required to be done only
 *      once. Configures all of EDMA and window RAM of HWA.
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_dataPathConfigCommon(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;
    uint32_t fftWindow;
    uint32_t winLen, winIndx, hwaRamOffset;
    float phi;
    HWA_CommonConfig hwaCommonConfig;

    MmwDemo_config1D_EDMA(obj);
    MmwDemo_config2D_EDMA(obj);
    MmwDemo_configCFAR_EDMA(obj);

    /**********************************************/
    /* Disable HWA and reset to known state       */
    /**********************************************/

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0);
    if (errCode != 0)
    {
        //System_printf("Error: MmwDemo_dataPathConfigCommon:HWA_enable(0) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /* Reset the internal state of the HWA */
    errCode = HWA_reset(obj->hwaHandle);
    if (errCode != 0)
    {
        //System_printf("Error: MmwDemo_dataPathConfigCommon:HWA_reset returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /***********************/
    /* CONFIG WINDOW RAM   */
    /***********************/
    /* if windowing is enabled, load the window coefficients in RAM */
    //1D-FFT window
    hwaRamOffset = 0;
    winLen = obj->numAdcSamples;
    phi = 2 * PI_ / ((float) winLen - 1);
    for(winIndx = 0; winIndx < winLen; winIndx++)
    {
        MmwDemo_genWindow(&fftWindow, winIndx, phi, MMW_WIN_BLACKMAN);
        errCode = HWA_configRam(obj->hwaHandle,
                                HWA_RAM_TYPE_WINDOW_RAM,
                                (uint8_t *)&fftWindow,
                                sizeof(uint32_t),   //size in bytes
                                hwaRamOffset); //offset in bytes
        if (errCode != 0)
        {
            //System_printf("Error: HWA_configRam returned %d\n",errCode);
            MmwDemo_debugAssert (0);
            return;
        }
        hwaRamOffset += sizeof(uint32_t);
    }

    winLen = obj->numDopplerBins;
    phi = 2 * PI_ / ((float) winLen - 1);
    for(winIndx = 0; winIndx < winLen; winIndx++)
    {
        MmwDemo_genWindow(&fftWindow, winIndx, phi, MMW_WIN_HANNING);
        errCode = HWA_configRam(obj->hwaHandle,
                                HWA_RAM_TYPE_WINDOW_RAM,
                                (uint8_t *)&fftWindow,
                                sizeof(uint32_t), //size in bytes
                                hwaRamOffset); //offset in bytes
        if (errCode != 0)
        {
            //System_printf("Error: HWA_configRam returned %d\n",errCode);
            MmwDemo_debugAssert (0);
            return;
        }
        hwaRamOffset += sizeof(uint32_t);
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    errCode = HWA_enableDoneInterrupt(obj->hwaHandle,
        MmwDemo_dataPathHwaDoneIsrCallback,
        obj->HWA_done_semHandle);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enableDoneInterrupt returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /***********************************************/
    /* ENABLE FFT Twiddle coefficient dithering    */
    /***********************************************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                                 HWA_COMMONCONFIG_MASK_LFSRSEED;
    hwaCommonConfig.fftConfig.twidDitherEnable = HWA_FEATURE_BIT_ENABLE;
    hwaCommonConfig.fftConfig.lfsrSeed = 0x1234567; /*Some non-zero value*/

    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configCommon returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

}

/**
 *  @b Description
 *  @n
 *      Init HWA.
 */
void MmwDemo_hwaInit(MmwDemo_DataPathObj *obj)
{
    /* Initialize the HWA */
    HWA_init();
}

/**
 *  @b Description
 *  @n
 *      Init EDMA.
 */
void MmwDemo_edmaInit(MmwDemo_DataPathObj *obj)
{
    uint8_t edmaNumInstances, inst;
    Semaphore_Params       semParams;
    int32_t errorCode;

    edmaNumInstances = EDMA_getNumInstances();
    for (inst = 0; inst < edmaNumInstances; inst++)
    {
        errorCode = EDMA_init(inst);
        if (errorCode != EDMA_NO_ERROR)
        {
            //System_printf ("Debug: EDMA instance %d initialization returned error %d\n", errorCode);
            MmwDemo_debugAssert (0);
            return;
        }
        //System_printf ("Debug: EDMA instance %d has been initialized\n", inst);
    }

    memset(&obj->EDMA_errorInfo, 0, sizeof(obj->EDMA_errorInfo));
    memset(&obj->EDMA_transferControllerErrorInfo, 0, sizeof(obj->EDMA_transferControllerErrorInfo));

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    obj->EDMA_1Ddone_semHandle = Semaphore_create(0, &semParams, NULL);

    /* Enable Done Interrupt */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    obj->HWA_done_semHandle = Semaphore_create(0, &semParams, NULL);

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    obj->EDMA_2Ddone_semHandle = Semaphore_create(0, &semParams, NULL);

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    obj->EDMA_CFARdone_semHandle = Semaphore_create(0, &semParams, NULL);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA CC (Channel controller) error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void MmwDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    MmwDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void MmwDemo_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo)
{
    MmwDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Open HWA instance.
 */
void MmwDemo_hwaOpen(MmwDemo_DataPathObj *obj, SOC_Handle socHandle)
{
    int32_t             errCode;

    /* Open the HWA Instance */
    obj->hwaHandle = HWA_open(0, socHandle, &errCode);
    if (obj->hwaHandle == NULL)
    {
        //System_printf("Error: Unable to open the HWA Instance err:%d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: HWA Instance %p has been opened successfully\n", obj->hwaHandle);
}

/**
 *  @b Description
 *  @n
 *      Close HWA instance.
 */
void MmwDemo_hwaClose(MmwDemo_DataPathObj *obj)
{
    int32_t             errCode;

    /* Close the HWA Instance */
    errCode = HWA_close(obj->hwaHandle);
    if (errCode != 0)
    {
        //System_printf("Error: Unable to close the HWA Instance err:%d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: HWA Instance %p has been closed successfully\n", obj->hwaHandle);
}

/**
 *  @b Description
 *  @n
 *      Open EDMA.
 */
void MmwDemo_edmaOpen(MmwDemo_DataPathObj *obj)
{
    int32_t             errCode;
    EDMA_instanceInfo_t edmaInstanceInfo;
    EDMA_errorConfig_t  errorConfig;

    obj->edmaHandle = EDMA_open(0, &errCode, &edmaInstanceInfo);

    if (obj->edmaHandle == NULL)
    {
        //System_printf("Error: Unable to open the EDMA Instance err:%d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: EDMA Instance %p has been opened successfully\n", obj->edmaHandle);

    errorConfig.isConfigAllEventQueues = true;
    errorConfig.isConfigAllTransferControllers = true;
    errorConfig.isEventQueueThresholdingEnabled = true;
    errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
    errorConfig.isEnableAllTransferControllerErrors = true;
    errorConfig.callbackFxn = MmwDemo_EDMA_errorCallbackFxn;
    errorConfig.transferControllerCallbackFxn = MmwDemo_EDMA_transferControllerErrorCallbackFxn;
    if ((errCode = EDMA_configErrorMonitoring(obj->edmaHandle, &errorConfig)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }

}

/**
 *  @b Description
 *  @n
 *      Close EDMA.
 */
void MmwDemo_edmaClose(MmwDemo_DataPathObj *obj)
{
    EDMA_close(obj->edmaHandle);
}

/**
 *  @b Description
 *  @n
 *      Configure HWA for 2D processing.
 *
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_config2D_HWA(MmwDemo_DataPathObj *obj)
{
    int32_t                 errCode;
    HWA_CommonConfig hwaCommonConfig;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /***************************************/
    /* Doppler FFT chain                   */
    /***************************************/
    HWAutil_configDopplerFFT(obj->hwaHandle,
        HWAUTIL_NUM_PARAM_SETS_1D,              // paramSetStartIdx,
        obj->numDopplerBins,               // dopplerFftSize,
        obj->numVirtualAntennas,        // numVirtualAnt,
        obj->numRangeBinsPerTransfer,    // numRangeBinsPerIter,
        obj->numAdcSamples + MMW_HWA_WINDOWRAM_1D_OFFSET,  // windowOffsetBytes

        MMW_HWA_DMA_TRIGGER_SOURCE_2D_PING,
        MMW_HWA_DMA_TRIGGER_SOURCE_2D_PONG,
        MMW_HWA_DMA_DEST_CHANNEL_2D_PING,
        MMW_HWA_DMA_DEST_CHANNEL_2D_PONG,

        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_2D_INP_PING), //hwaMemSourcePingOffset
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_2D_INP_PONG), //hwaMemSourcePongOffset
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_2D_OUT_PING), //hwaMemDestPingOffset
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_2D_OUT_PONG),
        obj->datapathChainSel); //hwaMemDestPongOffset


    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;


    hwaCommonConfig.numLoops = obj->numRangeBins/(2*obj->numRangeBinsPerTransfer);
    hwaCommonConfig.paramStartIdx = HWAUTIL_NUM_PARAM_SETS_1D; //TODO //only one paramset used here so start and stop index are the same
    hwaCommonConfig.paramStopIdx = HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_2D-1;//TODO
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configCommon returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Trigger 2D processing.
 *
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_dataPathTrigger2D(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;

    /* Enable the HWA */
    errCode = HWA_enable(obj->hwaHandle,1); // set 1 to enable
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    EDMA_startTransfer(obj->edmaHandle, MMW_EDMA_2D_PING_CHAIN_CH_ID2, EDMA3_CHANNEL_TYPE_DMA);
    EDMA_startTransfer(obj->edmaHandle, MMW_EDMA_2D_PONG_CHAIN_CH_ID2, EDMA3_CHANNEL_TYPE_DMA);
}

/**
 *  @b Description
 *  @n
 *      Waits for 2D processing to finish. This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait2D(MmwDemo_DataPathObj *obj)
{
    Bool       status;

    /* wait for the all paramSets done interrupt */
    status = Semaphore_pend(obj->HWA_done_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //System_printf("Error: Semaphore_pend returned %d\n",status);
        MmwDemo_debugAssert (0);
        return;
    }

    status = Semaphore_pend(obj->EDMA_2Ddone_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //System_printf("Error: Semaphore_pend returned %d\n",status);
        MmwDemo_debugAssert (0);
        return;
    }
}

#define SOC_MAX_NUM_RX_ANTENNAS 4
#define SOC_MAX_NUM_TX_ANTENNAS 3

/**
 *  @b Description
 *  @n
 *      Initializes data path object with supplied cli config pointer. The cli config
 *      points to permanent storage outside of data path object that data path object can refer
 *      to anytime during the lifetime of data path object.
 *      Data path default values that are not required to come through CLI commands are also
 *      set in this function.
 */
void MmwDemo_dataPathObjInit(MmwDemo_DataPathObj *obj,
                             MmwDemo_CliCfg_t *cliCfg,
                             MmwDemo_CliCommonCfg_t *cliCommonCfg)
{
    memset(obj, 0, sizeof(MmwDemo_DataPathObj));
    obj->cliCfg = cliCfg;
    obj->cliCommonCfg = cliCommonCfg;

    /* Default datapath chain selection */
    obj->datapathChainSel = DATA_PATH_CHAIN_COMBINED_LOGMAG;

}

/**
 *  @b Description
 *  @n
 *      Allocate buffers needed for data path.
 */
void MmwDemo_dataPathCfgBuffers(MmwDemo_DataPathObj *obj, MmwDemoMemPool_t *pool)
{
    MmwDemo_memPoolReset(pool);

    obj->radarCube = (uint32_t *) MmwDemo_memPoolAlloc(pool, obj->numRangeBins *
            obj->numDopplerBins * obj->numVirtualAntennas * sizeof(uint32_t));

    if (obj->radarCube == NULL)
    {
        MmwDemo_debugAssert(0);
    }

    obj->rangeDopplerLogMagMatrix = (uint16_t *) MmwDemo_memPoolAlloc(pool, obj->numRangeBins *
        obj->numDopplerBins * sizeof(uint16_t));

    if (obj->rangeDopplerLogMagMatrix == NULL)
    {
        MmwDemo_debugAssert(0);
    }

    obj->cfarDetectionOut = (cfarDetOutput_t *) MmwDemo_memPoolAlloc(pool,
        MMW_MAX_OBJ_OUT * sizeof(cfarDetOutput_t));

    if (obj->cfarDetectionOut == NULL)
    {
        MmwDemo_debugAssert(0);
    }

    if (obj->cliCfg->guiMonSel.rangeAzimuthHeatMap)
    {
        obj->azimuthStaticHeatMap = (cmplx16ImRe_t *) MmwDemo_memPoolAlloc(pool,
                   obj->numRangeBins * obj->numVirtualAntAzim * sizeof(cmplx16ImRe_t));

        if (obj->azimuthStaticHeatMap == NULL)
        {
            MmwDemo_debugAssert(0);
        }
    }
    
    if (obj->dcRangeSigMean == NULL)
    {
        obj->dcRangeSigMean = (cmplx32ImRe_t *) Memory_alloc (NULL,
              DC_RANGE_SIGNATURE_COMP_MAX_BIN_SIZE * SOC_MAX_NUM_TX_ANTENNAS * SOC_MAX_NUM_RX_ANTENNAS * sizeof(cmplx32ImRe_t),
              (uint8_t) MMWDEMO_MEMORY_ALLOC_DOUBLE_WORD_ALIGN,
              NULL);
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool structure.
 *
 *  @retval
 *      none.
 */
void MmwDemo_memPoolReset(MmwDemoMemPool_t *pool)
{
    pool->indx = 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function for allocating from a static memory pool.
 *
 *  @pre It is assumed that no allocation from l3 will need better than 32-bit
 *      alignment (structures with 64-bit prohibited) and so for simplicity,
 *      size is assumed to be multiple of 4 bytes
 *
 *  @param[in]  pool Handle to pool structure.
 *  @param[in]  size Size in bytes to be allocated.
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could notice
 *      allocate.
 */
uint8_t *MmwDemo_memPoolAlloc(MmwDemoMemPool_t *pool, uint32_t size)
{
    uint8_t *ptr = NULL;

    if ((size % 4) != 0)
    {
        return(ptr);
    }

    if ((pool->indx + size) <= pool->size)
    {
        ptr = pool->base + pool->indx;
        pool->indx += size;
    }

    return(ptr);
}

/**
 *  @b Description
 *  @n
 *      Configures all 1D processing related EDMA configuration.
 *
 *  @param[in] obj Pointer to data path object
 *  @retval EDMA error code, see EDMA API.
 */
int32_t MmwDemo_config1D_EDMA(MmwDemo_DataPathObj *obj)
{
    int32_t errorCode = EDMA_NO_ERROR;
    EDMA_Handle handle = obj->edmaHandle;
    HWA_SrcDMAConfig dmaConfig;

    /* Ping configuration to transfer 1D FFT output from HWA to L3 RAM transposed */
    errorCode = EDMAutil_configHwaTranspose(handle,
        MMW_EDMA_1D_PING_CH_ID,       //chId,
        MMW_EDMA_1D_PING_SHADOW_LINK_CH_ID,  //linkChId,
        MMW_EDMA_1D_PING_CHAIN_CH_ID, //chainChId,
        (uint32_t*)SOC_translateAddress((uint32_t)MMW_HWA_1D_OUT_PING,SOC_TranslateAddr_Dir_TO_EDMA,NULL),   //*pSrcAddress,
        (uint32_t*)SOC_translateAddress((uint32_t)obj->radarCube,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //*pDestAddress,
        obj->numRxAntennas,            //numAnt,
        obj->numRangeBins,        //numRangeBins,
        obj->numChirpsPerFrame,  //numChirpsPerFrame,
        true,   //isIntermediateChainingEnabled,
        true,   //isFinalChainingEnabled,
        false, //isTransferCompletionEnabled
        NULL,  //transferCompletionCallbackFxn
		NULL);

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /* Pong configuration to transfer 1D FFT output from HWA to L3 RAM transposed */
    errorCode = EDMAutil_configHwaTranspose(handle,
        MMW_EDMA_1D_PONG_CH_ID,      //chId,
        MMW_EDMA_1D_PONG_SHADOW_LINK_CH_ID, //linkChId,
        MMW_EDMA_1D_PONG_CHAIN_CH_ID, //SR_DBT_xxx //chainChId,
        (uint32_t*)SOC_translateAddress((uint32_t)MMW_HWA_1D_OUT_PONG,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //*pSrcAddress,
        (uint32_t*) SOC_translateAddress((uint32_t)(obj->radarCube + obj->numRxAntennas),SOC_TranslateAddr_Dir_TO_EDMA,NULL), //*pDestAddress,
         obj->numRxAntennas,           //numAnt,
         obj->numRangeBins,       //numRangeBins,
         obj->numChirpsPerFrame, //numChirpsPerFrame,
         true, //isIntermediateChainingEnabled,
         true, //isFinalChainingEnabled,
         true, //isTransferCompletionEnabled
         MmwDemo_EDMA_transferCompletionCallbackFxn, //transferCompletionCallbackFxn
		 (uintptr_t)obj);

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

     /**************************************************************************
      *  PROGRAM EDMA2_ACC_CHAN0 (resp. EDMA2_ACC_CHAN1) to communicate completion
         of DMA CHAN EDMA_TPCC0_REQ_HWACC_0 (resp. EDMA_TPCC0_REQ_HWACC_1)
      *************************************************************************/

    HWA_getDMAconfig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SOURCE_1D_PING, &dmaConfig);

    errorCode = EDMAutil_configHwaOneHotSignature(handle,
        MMW_EDMA_1D_PING_CHAIN_CH_ID, //chId,
        false, //isEventTriggered
        (uint32_t*)SOC_translateAddress(dmaConfig.srcAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pSrcAddress
        (uint32_t*)SOC_translateAddress(dmaConfig.destAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pDestAddress
        dmaConfig.aCnt,
        dmaConfig.bCnt,
        dmaConfig.cCnt,
        MMW_EDMA_1D_PING_ONE_HOT_SHADOW_LINK_CH_ID); //linkChId

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

    HWA_getDMAconfig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SOURCE_1D_PONG, &dmaConfig);

    errorCode = EDMAutil_configHwaOneHotSignature(handle,
        MMW_EDMA_1D_PONG_CHAIN_CH_ID, //chId,
        false, //isEventTriggered
        (uint32_t*)SOC_translateAddress(dmaConfig.srcAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //pSrcAddress
        (uint32_t*)SOC_translateAddress(dmaConfig.destAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pDestAddress
        dmaConfig.aCnt,
        dmaConfig.bCnt,
        dmaConfig.cCnt,
        MMW_EDMA_1D_PONG_ONE_HOT_SHADOW_LINK_CH_ID); //linkChId

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

exit:
    return(errorCode);
}


/**
 *  @b Description
 *  @n
 *      Configures all CFAR processing related EDMA configuration.
 *
 *  @param[in] obj Pointer to data path object
 *  @retval EDMA error code, see EDMA API.
 */
int32_t MmwDemo_configCFAR_EDMA(MmwDemo_DataPathObj *obj)
{
    int32_t errorCode = EDMA_NO_ERROR;
    EDMA_Handle handle = obj->edmaHandle;
    HWA_SrcDMAConfig dmaConfig;

//program DMA to MOVE DATA FROM L3_DETECTION TO ACCEL, THIS SHOULD BE LINKED TO DMA2ACC
    errorCode = EDMAutil_configHwaContiguous(handle,
        MMW_EDMA_CFAR_INP_CH_ID,              //chId,
        false, //isEventTriggered
        MMW_EDMA_CFAR_INP_SHADOW_LINK_CH_ID1, //linkChId,
        MMW_EDMA_CFAR_INP_CHAIN_CH_ID,       //chainChId,
        (uint32_t*)SOC_translateAddress((uint32_t)obj->rangeDopplerLogMagMatrix,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //*pSrcAddress,
        (uint32_t*)SOC_translateAddress((uint32_t)MMW_HWA_CFAR_INP,SOC_TranslateAddr_Dir_TO_EDMA,NULL),      //*pDestAddress,
        obj->numRangeBins * obj->numDopplerBins * sizeof(uint16_t), //numBytes,
        1, //numBlocks,
        0, //srcIncrBytes,
        0, //dstIncrBytes,
        false,  //isIntermediateChainingEnabled,
        true,   //isFinalChainingEnabled,
        false, //isTransferCompletionEnabled;
        NULL,  //transferCompletionCallbackFxn
		NULL);

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

    HWA_getDMAconfig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SOURCE_CFAR, &dmaConfig);

    errorCode = EDMAutil_configHwaOneHotSignature(handle,
        MMW_EDMA_CFAR_INP_CHAIN_CH_ID, //chId,
        false, //isEventTriggered
        (uint32_t*)SOC_translateAddress(dmaConfig.srcAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //pSrcAddress
        (uint32_t*)SOC_translateAddress(dmaConfig.destAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pDestAddress
        dmaConfig.aCnt,
        dmaConfig.bCnt,
        dmaConfig.cCnt,
        MMW_EDMA_CFAR_INP_SHADOW_LINK_CH_ID2); //linkChId

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

//another DMA to move data from ACC to L3_OBJECT LIN
    errorCode = EDMAutil_configHwaContiguous(handle,
        MMW_EDMA_CFAR_OUT_CH_ID,              //chId,
        true, //isEventTriggered
        MMW_EDMA_CFAR_OUT_SHADOW_LINK_CH_ID1, //linkChId,
        MMW_EDMA_CFAR_OUT_CHAIN_CH_ID,       //chainChId,
        (uint32_t*)SOC_translateAddress((uint32_t)MMW_HWA_CFAR_OUT,SOC_TranslateAddr_Dir_TO_EDMA,NULL),      //*pSrcAddress,
        (uint32_t*)SOC_translateAddress((uint32_t)obj->cfarDetectionOut,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //*pDestAddress,
        MMW_MAX_OBJ_OUT * sizeof(cfarDetOutput_t), //numBytes
        1,     //numBlocks,
        0, //srcIncrBytes,
        0, //dstIncrBytes,
        false, //isIntermediateChainingEnabled,
        false, //isFinalChainingEnabled,
        true, //isTransferCompletionEnabled;
        MmwDemo_EDMA_transferCompletionCallbackFxn,//transferCompletionCallbackFxn)
		(uintptr_t)obj);

exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      Configures EDMA to prepare for single range bin 2D FFT. It copies a signle bin 1D FFT symbols from
 *  radarCube to HWA memory. Then triggers 2D FFT  in HWA.
 *
 *  @param[in]  obj Pointer to data path object
 *  @param[in]  srcBuffPtr Source buffer address for Single bin 2D FFT operation
 *  @param[in]  dstBuffPtr HWA memory offset to single bin 2D FFT

 *  @retval EDMA error code, see EDMA API.
 */
int32_t MmwDemo_config2DSingleBin_EDMA
(
    MmwDemo_DataPathObj *obj,
    uint32_t srcBuffPtr,
    uint32_t dstBuffPtr
)
{
    int32_t     errorCode = EDMA_NO_ERROR;
    volatile bool       edmaTransComplete = false ;
    HWA_SrcDMAConfig    dmaConfig;

    /* Copy one Range bin to HWA memory */
    errorCode = EDMAutil_configHwaContiguous(
                obj->edmaHandle,
                MMW_EDMA_2DFFT_SINGLERBIN_CH_ID,
                false,
                MMW_EDMA_2DFFT_SINGLERBIN_SHADOW_LINK_CH_ID,
                MMW_EDMA_2DFFT_SINGLERBIN_CHAIN_CH_ID,
                (uint32_t*)SOC_translateAddress(srcBuffPtr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pSrcAddress
                (uint32_t*)SOC_translateAddress(dstBuffPtr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pDestAddress
                obj->numVirtualAntennas * sizeof(uint32_t) * obj->numDopplerBins,
                1,
                0,
                0,
                true,
                true,
                false,
                NULL,
                NULL);


    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /* Copy one hot signature to trigger HWA 2D FFT */
    HWA_getDMAconfig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SINGLEBIN_2DFFT, &dmaConfig);

    errorCode = EDMAutil_configHwaOneHotSignature(obj->edmaHandle,
        MMW_EDMA_2DFFT_SINGLERBIN_CHAIN_CH_ID, //chId,
        false, //isEventTriggered
        (uint32_t*)SOC_translateAddress((uint32_t)dmaConfig.srcAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL),  //pSrcAddress
        (uint32_t*)SOC_translateAddress((uint32_t)dmaConfig.destAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL), //pDestAddress
        dmaConfig.aCnt,
        dmaConfig.bCnt,
        dmaConfig.cCnt,
        MMW_EDMA_2DFFT_SINGLERBIN_SHADOW_LINK_CH_ID2); //linkChId

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }
exit:
    return(errorCode);
}


/**
 *  @b Description
 *  @n
 *      Trigger Single range bin 2D FFT processing.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathTrigger2DSingleBin(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;

    /* Enable the HWA */
    errCode = HWA_enable(obj->hwaHandle, 1); // set 1 to enable
    if (errCode != 0)
    {
        MmwDemo_debugAssert (0);
        return;
    }

    /* Trigger EDMA */
    EDMA_startDmaTransfer(obj->edmaHandle, MMW_EDMA_2DFFT_SINGLERBIN_CH_ID);
}

/**
 *  @b Description
 *  @n
 *      Configure HWA for 1D processing.
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_config1D_HWA(MmwDemo_DataPathObj *obj)
{
    int32_t             errCode;
    HWA_CommonConfig    hwaCommonConfig;
    uint8_t             hwaTriggerMode;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0); // set 1 to enable
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    if(obj->dataPathMode == DATA_PATH_STANDALONE)
    {
        /* trigger manually and immediately */
        hwaTriggerMode = HWA_TRIG_MODE_SOFTWARE;
    }
    else
    {
        /* trigger done by ADC buffer */
        hwaTriggerMode = HWA_TRIG_MODE_DFE;
    }

    HWAutil_configRangeFFT(obj->hwaHandle,
        MMW_HWA_START_POS_PARAMSETS_1D,
        obj->numAdcSamples,
        obj->numRangeBins,
        obj->numRxAntennas,
        MMW_HWA_WINDOWRAM_1D_OFFSET,
        MMW_HWA_DMA_TRIGGER_SOURCE_1D_PING,
        MMW_HWA_DMA_TRIGGER_SOURCE_1D_PONG,
        MMW_HWA_DMA_DEST_CHANNEL_1D_PING,
        MMW_HWA_DMA_DEST_CHANNEL_1D_PONG,
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_ADCBUF_INP),
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_OUT_PING),
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_OUT_PONG),
        hwaTriggerMode
    );

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;
    hwaCommonConfig.numLoops = obj->numChirpsPerFrame/2;
    hwaCommonConfig.paramStartIdx = MMW_HWA_START_POS_PARAMSETS_1D;
    hwaCommonConfig.paramStopIdx = MMW_HWA_START_POS_PARAMSETS_1D + HWAUTIL_NUM_PARAM_SETS_1D - 1;
    if(obj->dataPathMode == DATA_PATH_STANDALONE)
    {
	    /* HWA will input data from M0 memory*/
        hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    }
    else
    {
	    /* HWA will input data from ADC buffer memory*/
        hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_ENABLE;
    }
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configCommon returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Trigger HWA for 1D processing.
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_dataPathTrigger1D(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;

    /* Enable the HWA */
    errCode = HWA_enable(obj->hwaHandle,1); // set 1 to enable
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /* trigger the HWA since triggerMode is set to DMA */
    errCode = HWA_setDMA2ACCManualTrig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SOURCE_1D_PING);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_setDMA2ACCManualTrig(0) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    errCode = HWA_setDMA2ACCManualTrig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SOURCE_1D_PONG);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_setDMA2ACCManualTrig(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Waits for 1D processing to finish. This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait1D(MmwDemo_DataPathObj *obj)
{
    Bool       status;
//    Semaphore_Handle       paramSetSem;

    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    /* wait for the all paramSets done interrupt */
    status = Semaphore_pend(obj->HWA_done_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //System_printf("Error: Semaphore_pend returned %d\n",status);
        MmwDemo_debugAssert (0);
    }

    /**********************************************/
    /* WAIT FOR EDMA NUMLOOPS INTERRUPT            */
    /**********************************************/
    status = Semaphore_pend(obj->EDMA_1Ddone_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //System_printf("Error: Semaphore_pend returned %d\n",status);
        MmwDemo_debugAssert (0);
    }
}

/**
 *  @b Description
 *  @n
 *      Configures HWA for CFAR processing.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_configCFAR_HWA(MmwDemo_DataPathObj *obj)
{
    int32_t                 errCode;
    HWA_CommonConfig hwaCommonConfig;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0); // set 1 to enable
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /**************************************/
    /* CFAR PROCESSING                    */
    /**************************************/
    HWAutil_configCFAR(obj->hwaHandle,
        HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_2D,
        obj->numRangeBins,
        obj->numDopplerBins,
        obj->cliCfg->cfarCfg.winLen,
        obj->cliCfg->cfarCfg.guardLen,
        obj->cliCfg->cfarCfg.noiseDivShift,
        obj->cliCfg->peakGroupingCfg.inRangeDirectionEn,
        obj->cliCfg->cfarCfg.cyclicMode,
        obj->cliCfg->cfarCfg.averageMode,
        MMW_HWA_MAX_CFAR_DET_OBJ_LIST_SIZE,
        MMW_HWA_DMA_TRIGGER_SOURCE_CFAR,
        MMW_HWA_DMA_DEST_CHANNEL_CFAR,
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_CFAR_INP), //hwaMemSourceOffset
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_CFAR_OUT)  //hwaMemDestOffset
    );


    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
                               HWA_COMMONCONFIG_MASK_CFARTHRESHOLDSCALE;


    hwaCommonConfig.numLoops = 1;
    hwaCommonConfig.paramStartIdx = HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_2D;
    hwaCommonConfig.paramStopIdx =  HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_2D;
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    hwaCommonConfig.cfarConfig.cfarThresholdScale = obj->cliCfg->cfarCfg.thresholdScale;

    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configCommon returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Trigger CFAR processing.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathTriggerCFAR(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;

    /* Enable the HWA */
    errCode = HWA_enable(obj->hwaHandle,1); // set 1 to enable
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    EDMA_startTransfer(obj->edmaHandle, MMW_EDMA_CFAR_INP_CH_ID, EDMA3_CHANNEL_TYPE_DMA);
}

/**
 *  @b Description
 *  @n
 *      Waits for CFAR processing to finish. This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWaitCFAR(MmwDemo_DataPathObj *obj)
{
    Bool       status;

    /* then wait for the all paramSets done interrupt */
    status = Semaphore_pend(obj->HWA_done_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //System_printf("Error: Semaphore_pend returned %d\n",status);
        MmwDemo_debugAssert (0);
        return;
    }

    /* wait for EDMA done */
    status = Semaphore_pend(obj->EDMA_CFARdone_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //System_printf("Error: Semaphore_pend returned %d\n",status);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Configures HWA for angle (azimuth) estimation on top of detected objects.
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] dataInPtr    HWA memory offset for input data of single range bin 2DFFT
 *  @param[in] dataOutPtr   HWA memory offset for out data of single range bin 2DFFT
 *
 *  @retval
 *      NONE
 */
void MmwDemo_configDetectedObj2DFFT_HWA
(
    MmwDemo_DataPathObj *obj,
    uint32_t *dataInPtr,
    uint32_t *dataOutPtr
)
{
    int32_t errCode = 0;
    HWA_CommonConfig hwaCommonConfig;
    uint32_t     paramIndex;


    paramIndex = HWAUTIL_NUM_PARAM_SETS_1D +
                HWAUTIL_NUM_PARAM_SETS_2D +
                HWAUTIL_NUM_PARAM_SETS_CFAR +
                HWAUTIL_NUM_PARAM_SETS_ANGLE;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle, 0); // set 1 to enable
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        return;
    }

    /* Perform 2D FFT for detected Objects */
    HWAutil_configDopplerFFTSingleRangeBin(obj->hwaHandle,
            paramIndex,
            obj->numDopplerBins,
            obj->numVirtualAntennas,
            obj->numAdcSamples + MMW_HWA_WINDOWRAM_1D_OFFSET,  // windowOffsetBytes
            MMW_HWA_DMA_TRIGGER_SINGLEBIN_2DFFT,
            ADDR_TRANSLATE_CPU_TO_HWA(dataInPtr),
            ADDR_TRANSLATE_CPU_TO_HWA(dataOutPtr));

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;


    hwaCommonConfig.numLoops = 1;
    hwaCommonConfig.paramStartIdx = paramIndex;
    hwaCommonConfig.paramStopIdx = paramIndex;

    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configCommon returned %d\n",errCode);
        return;
    }

}

/**
 *  @b Description
 *  @n
 *      Configures HWA for angle (azimuth) estimation.
 *
 *  @param[in] obj  Pointer to data path object
 *  @param[in] numObjOut  Number of detected objects
 *
 *  @retval
 *      NONE
 */
void MmwDemo_configAngleEstimation_HWA(MmwDemo_DataPathObj *obj, uint32_t numObjOut)
{
    int32_t errCode;
    HWA_CommonConfig hwaCommonConfig;
    int32_t numHwaParamSets3D;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0); // set 1 to enable
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    if(obj->numVirtualAntElev == 0)
    {
        HWAutil_configAngleEstAzimuth(obj->hwaHandle,
            HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_2D + HWAUTIL_NUM_PARAM_SETS_CFAR,
            obj->numVirtualAntennas,
            obj->numAngleBins,
            numObjOut,
            ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_ANGLE_AZIM_INP), //hwaMemSourceOffset
            ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_ANGLE_AZIM_ABS_OUT));  //hwaMemDestOffset
        numHwaParamSets3D = 1;
    }
    else
    {
        HWAutil_configAngleEstAzimuthElevation(obj->hwaHandle,
            HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_2D + HWAUTIL_NUM_PARAM_SETS_CFAR,
            obj->numVirtualAntAzim,
            obj->numVirtualAntElev,
            obj->numAngleBins,
            numObjOut,
            ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_ANGLE_AZIM_INP),
            ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_ANGLE_ELEV_INP),
            ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_ANGLE_AZIM_ABS_OUT),
            ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_ANGLE_AZIM_CPLX_OUT),
            ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_ANGLE_ELEV_CPLX_OUT));
        numHwaParamSets3D = 3;
    }

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;


    hwaCommonConfig.numLoops = 1;
    hwaCommonConfig.paramStartIdx = HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_2D +
        HWAUTIL_NUM_PARAM_SETS_CFAR;
    hwaCommonConfig.paramStopIdx =  HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_2D +
        HWAUTIL_NUM_PARAM_SETS_CFAR + (numHwaParamSets3D - 1);
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configCommon returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Wait for angle estimation to complete. This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait2DFFTDetObj(MmwDemo_DataPathObj *obj)
{
    Bool       status;

    status = Semaphore_pend(obj->HWA_done_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: Semaphore_pend returned %d\n",status);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Trigger Angle estimation processing.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathTriggerAngleEstimation(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;

    /* Enable the HWA */
    errCode = HWA_enable(obj->hwaHandle,1); // set 1 to enable
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Wait for angle estimation to complete. This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWaitAngleEstimation(MmwDemo_DataPathObj *obj)
{
    Bool       status;

    status = Semaphore_pend(obj->HWA_done_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //System_printf("Error: Semaphore_pend returned %d\n",status);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *    Compensation of DC range antenna signature
 *
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dcRangeSignatureCompensation(MmwDemo_DataPathObj *obj)
{
    uint32_t antIdx, rngBinIdx;
    uint32_t ind;
    int32_t dcRangeSigMeanSize;
    uint32_t doppIdx;
    cmplx16ImRe_t *fftOut1D = (cmplx16ImRe_t *) obj->radarCube;
    MmwDemo_CalibDcRangeSigCfg *calibDc = &obj->cliCfg->calibDcRangeSigCfg;
    int32_t rngOffset = obj->numVirtualAntennas * obj->numDopplerBins;
    int32_t numVirtAnt = obj->numVirtualAntennas;
    int32_t negativeBinIdx = calibDc->negativeBinIdx;
    int32_t positiveBinIdx = calibDc->positiveBinIdx;
    cmplx32ImRe_t *dcRangeSigMean = obj->dcRangeSigMean;
    int32_t numRangeBins = obj->numRangeBins;
    int32_t log2NumAvgChirps = obj->log2NumAvgChirps;
    int32_t numDopplerBins = obj->numDopplerBins;

    dcRangeSigMeanSize = numVirtAnt * (positiveBinIdx - negativeBinIdx + 1);
    if (obj->dcRangeSigCalibCntr == 0)
    {
        memset(dcRangeSigMean, 0, dcRangeSigMeanSize * sizeof(cmplx32ImRe_t));
    }

    /* Calibration */
    if (obj->dcRangeSigCalibCntr < calibDc->numAvgChirps)
    {
        for (doppIdx = 0; doppIdx < numDopplerBins; doppIdx++)
        {
            /* Accumulate */
            ind = 0;
            for (rngBinIdx = 0; rngBinIdx <= positiveBinIdx; rngBinIdx++)
            {
                for (antIdx  = 0; antIdx < numVirtAnt; antIdx++)
                {
                    int32_t binIdx = rngBinIdx * rngOffset + doppIdx * numVirtAnt + antIdx;
                    dcRangeSigMean[ind].real += fftOut1D[binIdx].real;
                    dcRangeSigMean[ind].imag += fftOut1D[binIdx].imag;
                    ind++;
                }
            }
            for (rngBinIdx = 0; rngBinIdx < -negativeBinIdx; rngBinIdx++)
            {
                for (antIdx  = 0; antIdx < numVirtAnt; antIdx++)
                {
                    int32_t binIdx = (numRangeBins + negativeBinIdx + rngBinIdx) * rngOffset +
                                      doppIdx * numVirtAnt + antIdx;
                    dcRangeSigMean[ind].real += fftOut1D[binIdx].real;
                    dcRangeSigMean[ind].imag += fftOut1D[binIdx].imag;
                    ind++;
                }
            }
            obj->dcRangeSigCalibCntr++;
        }

        if (obj->dcRangeSigCalibCntr == calibDc->numAvgChirps)
        {
            /* Divide */
            for (ind  = 0; ind < dcRangeSigMeanSize; ind++)
            {
                dcRangeSigMean[ind].real = dcRangeSigMean[ind].real >> log2NumAvgChirps;
                dcRangeSigMean[ind].imag = dcRangeSigMean[ind].imag >> log2NumAvgChirps;
            }
        }
    }
    else
    {
        /* fftOut1D -= dcRangeSigMean */
        for (doppIdx = 0; doppIdx < numDopplerBins; doppIdx++)
        {
            ind = 0;
            for (rngBinIdx = 0; rngBinIdx <= positiveBinIdx; rngBinIdx++)
            {
                for (antIdx  = 0; antIdx < numVirtAnt; antIdx++)
                {
                    int32_t binIdx = rngBinIdx * rngOffset + doppIdx * numVirtAnt + antIdx;
                    fftOut1D[binIdx].real -= dcRangeSigMean[ind].real;
                    fftOut1D[binIdx].imag -= dcRangeSigMean[ind].imag;
                    ind++;
                }
            }
            for (rngBinIdx = 0; rngBinIdx < -negativeBinIdx; rngBinIdx++)
            {
                for (antIdx  = 0; antIdx < numVirtAnt; antIdx++)
                {
                    int32_t binIdx = (numRangeBins + negativeBinIdx + rngBinIdx) * rngOffset +
                                     doppIdx * numVirtAnt + antIdx;
                    fftOut1D[binIdx].real -= dcRangeSigMean[ind].real;
                    fftOut1D[binIdx].imag -= dcRangeSigMean[ind].imag;
                    ind++;
                }
            }
        }
     }
}

/*!*****************************************************************************************************************
 * \brief
 * Function Name       :    MmwDemo_rxChanPhaseBiasCompensation
 *
 * \par
 * <b>Description</b>  : Compensation of rx channel phase bias
 *
 * @param[in]      obj Data path object
 *
 * @param[in]      numObj Number of detected points
 *
 * @param[inout]   azimuthSymbols Symbols of Azimuth antennas
 *
 * @param[inout]   elevationSymbols Symbols of Elevation antennas
 *
 * @return         void
 *
 *******************************************************************************************************************
 */
void MmwDemo_rxChanPhaseBiasCompensation(MmwDemo_DataPathObj *obj,
                                         uint32_t numObj,
                                         cmplx16ImRe_t *azimuthSymbols,
                                         cmplx16ImRe_t *elevationSymbols)
{
    cmplx16ImRe_t *rxChComp = (cmplx16ImRe_t *) obj->compRxChanCfg.rxChPhaseComp;
    int32_t Re, Im;
    uint32_t i, j;
    uint32_t antIndx;
    uint32_t objIdx;
    uint32_t numAzimAnt = obj->numVirtualAntAzim;
    uint32_t numElevAnt = obj->numVirtualAntElev;


    for (objIdx = 0; objIdx < numObj; objIdx++)
    {
        j = 0;
        /* Compensation of azimuth antennas */
        for (antIndx = 0; antIndx < numAzimAnt; antIndx++)
        {
            i = objIdx*numAzimAnt + antIndx;
            Re = (int32_t) azimuthSymbols[i].real * (int32_t) rxChComp[j].real -
                 (int32_t) azimuthSymbols[i].imag * (int32_t) rxChComp[j].imag;

            Im = (int32_t) azimuthSymbols[i].real * (int32_t) rxChComp[j].imag +
                 (int32_t) azimuthSymbols[i].imag * (int32_t) rxChComp[j].real;
            Re = (Re + 0x4000) >> 15;
            Im = (Im + 0x4000) >> 15;
            Re = MMWDEMO_SATURATE_HIGH(Re);
            Re = MMWDEMO_SATURATE_LOW(Re);
            Im = MMWDEMO_SATURATE_HIGH(Im);
            Im = MMWDEMO_SATURATE_LOW(Im);

            azimuthSymbols[i].real = (int16_t) Re;
            azimuthSymbols[i].imag = (int16_t) Im;
            j++;
        }
        /* Compensation of elevation antennas */
        for (antIndx = 0; antIndx < numElevAnt; antIndx++)
        {
            if(elevationSymbols == NULL)
            {
                break;
            }
            else
            {
                i = objIdx*numElevAnt + antIndx;
                Re = (int32_t) elevationSymbols[i].real * (int32_t) rxChComp[j].real -
                     (int32_t) elevationSymbols[i].imag * (int32_t) rxChComp[j].imag;

                Im = (int32_t) elevationSymbols[i].real * (int32_t) rxChComp[j].imag +
                     (int32_t) elevationSymbols[i].imag * (int32_t) rxChComp[j].real;
                Re = (Re + 0x4000) >> 15;
                Im = (Im + 0x4000) >> 15;
                Re = MMWDEMO_SATURATE_HIGH(Re);
                Re = MMWDEMO_SATURATE_LOW(Re);
                Im = MMWDEMO_SATURATE_HIGH(Im);
                Im = MMWDEMO_SATURATE_LOW(Im);
                elevationSymbols[i].real = (int16_t) Re;
                elevationSymbols[i].imag = (int16_t) Im;
                j++;
            }
        }
    }
}


/**
 *  @b Description
 *  @n
 *      2D process chain.
 */
void MmwDemo_process2D(MmwDemo_DataPathObj *obj)
{
        MmwDemo_config2D_HWA(obj);
        MmwDemo_dataPathTrigger2D(obj);
        MmwDemo_dataPathWait2D(obj);
}

/**
 *  @b Description
 *  @n
 *      CFAR process chain.
 */
void MmwDemo_processCfar(MmwDemo_DataPathObj *obj, uint16_t *numDetectedObjects)
{
        MmwDemo_configCFAR_HWA(obj);
        MmwDemo_dataPathTriggerCFAR(obj);
        MmwDemo_dataPathWaitCFAR(obj);

        HWA_readCFARPeakCountReg(obj->hwaHandle,
            (uint8_t *) numDetectedObjects, sizeof(uint16_t));
        obj->numHwaCfarDetections = *numDetectedObjects;
}

/**
 *  @b Description
 *  @n
 *      Angle estimation process chain.
 */
void MmwDemo_processAngle(MmwDemo_DataPathObj *obj)
{
    uint32_t    numObjOut;
    uint32_t    idx;
    float       range;
    uint32_t    oneQFormat;

    /* Pre-processing before Angle estimation
       1. Peak Grouping if enabled
       2. Do Singlebin 2D FFT if DATA_PATH_CHAIN_COMBINED_LOGMAG is selected
       3. Doppler compensation if there is detected objects
       4. Doppler index conversion if there is detected objects
     */
    if(obj->datapathChainSel == DATA_PATH_CHAIN_SEPARATE_LOGMAG)
    {
        /* Save Azimuth heap in L3 RAM */
        uint32_t skip = obj->numChirpsPerFrame * obj->numRxAntennas;
        uint32_t copySize = obj->numVirtualAntAzim * sizeof(uint32_t);

        for (idx = 0; idx < obj->numRangeBins; idx++)
        {
            memcpy( (void *)&obj->azimuthStaticHeatMap[idx * obj->numVirtualAntAzim],
                    (void *)&obj->radarCube[idx * skip],
                    copySize);

        }
   
        /* Peak grouping */
        numObjOut = MmwDemo_peakGrouping(obj->numHwaCfarDetections,
                                 obj->rangeDopplerLogMagMatrix,
                                 (cfarDetOutput_t* ) MMW_HWA_CFAR_OUT,
                                 obj->radarCube,
                                 &obj->objOut[0],
                                 (uint32_t*) MMW_HWA_ANGLE_AZIM_INP,
                                 (uint32_t*) MMW_HWA_ANGLE_ELEV_INP,
                                 obj->numVirtualAntAzim,
                                 obj->numVirtualAntElev,
                                 obj->numDopplerBins,
                                 obj->cliCfg->peakGroupingCfg.minRangeIndex,
                                 obj->cliCfg->peakGroupingCfg.maxRangeIndex,
                                 obj->cliCfg->peakGroupingCfg.inDopplerDirectionEn,
                                 0);//peak grouping in range direction done in HWA
    }
    else if (obj->cliCfg->guiMonSel.rangeAzimuthHeatMap)
    {
        /* Extra steps: Single Bin 2DFFT on detected Objects  */
        uint32_t    RangeBinSize;
        RangeBinSize = obj->numDopplerBins * obj->numVirtualAntennas;
        uint32_t copySize = obj->numVirtualAntAzim * sizeof(uint32_t);
        
        for(idx = 0; idx < obj->numRangeBins; idx++)
        {
            /* Config HWA for single bin 2D FFT */
            MmwDemo_configDetectedObj2DFFT_HWA( obj,
                                            (uint32_t *)MMW_HWA_2DFFT_SINGLERBIN_INP,
                                            (uint32_t *)MMW_HWA_2DFFT_SINGLERBIN_OUT
                                            );

            /* Config EDMA to copy 1D FFT symbol to HWA memory */
            MmwDemo_config2DSingleBin_EDMA(obj,
                                      (uint32_t)(&obj->radarCube[RangeBinSize * idx]),
                                      (uint32_t)(MMW_HWA_2DFFT_SINGLERBIN_INP)
                                      );
            /* Trigger single 2D FFT */
            MmwDemo_dataPathTrigger2DSingleBin(obj);

            /* Wait until HWA done with FFT */
            MmwDemo_dataPathWait2DFFTDetObj(obj);
            
            memcpy( (void *)&obj->azimuthStaticHeatMap[idx * obj->numVirtualAntAzim],
                    (void *)MMW_HWA_2DFFT_SINGLERBIN_OUT,
                    copySize);
               
            memcpy( (void *)&obj->radarCube[RangeBinSize * idx],
                    (void *)MMW_HWA_2DFFT_SINGLERBIN_OUT,
                    RangeBinSize);
        }
   
        /* Peak grouping */
        numObjOut = MmwDemo_peakGrouping(obj->numHwaCfarDetections,
                                 obj->rangeDopplerLogMagMatrix,
                                 obj->cfarDetectionOut,
                                 obj->radarCube,
                                 &obj->objOut[0],
                                 (uint32_t*) MMW_HWA_ANGLE_AZIM_INP,
                                 (uint32_t*) MMW_HWA_ANGLE_ELEV_INP,
                                 obj->numVirtualAntAzim,
                                 obj->numVirtualAntElev,
                                 obj->numDopplerBins,
                                 obj->cliCfg->peakGroupingCfg.minRangeIndex,
                                 obj->cliCfg->peakGroupingCfg.maxRangeIndex,
                                 obj->cliCfg->peakGroupingCfg.inDopplerDirectionEn,
                                 0);//peak grouping in range direction done in HWA
        
    }
    else
    {
        numObjOut = MmwDemo_peakGrouping(obj->numHwaCfarDetections,
                                 obj->rangeDopplerLogMagMatrix,
                                 obj->cfarDetectionOut,
                                 (uint32_t*)NULL,
                                 &obj->objOut[0],
                                 (uint32_t*)NULL,
                                 (uint32_t*)NULL,
                                 obj->numVirtualAntAzim,
                                 obj->numVirtualAntElev,
                                 obj->numDopplerBins,
                                 obj->cliCfg->peakGroupingCfg.minRangeIndex,
                                 obj->cliCfg->peakGroupingCfg.maxRangeIndex,
                                 obj->cliCfg->peakGroupingCfg.inDopplerDirectionEn,
                                 0);//peak grouping in range direction done in HWA

        if (( numObjOut > 0) && (obj->numVirtualAntAzim != 1))
        {
            /* Extra steps: Single Bin 2DFFT on detected Objects  */
            uint32_t    dstAddrAzim;
            uint32_t    srcAddrElev;
            uint32_t    dstAddrElev;

            uint32_t    RangeBinSize;
            uint32_t    dopplerBinSize;
            uint32_t    previousIdx = obj->numRangeBins;
            RangeBinSize = obj->numDopplerBins * obj->numVirtualAntennas;
            dopplerBinSize = sizeof(uint32_t) * obj->numVirtualAntennas;

            /* Azimuth & Elevation buffer */
            dstAddrAzim = (uint32_t)MMW_HWA_ANGLE_AZIM_INP;
            dstAddrElev = (uint32_t)MMW_HWA_ANGLE_ELEV_INP;

            srcAddrElev = (uint32_t)MMW_HWA_2DFFT_SINGLERBIN_OUT + sizeof(uint32_t) * obj->numVirtualAntAzim;
            for(idx = 0;idx < numObjOut; idx++)
            {

                if (previousIdx != obj->objOut[idx].rangeIdx)
                {
                    /* Config HWA for single bin 2D FFT */
                    MmwDemo_configDetectedObj2DFFT_HWA( obj,
                                                    (uint32_t *)MMW_HWA_2DFFT_SINGLERBIN_INP,
                                                    (uint32_t *)MMW_HWA_2DFFT_SINGLERBIN_OUT
                                                    );

                    /* Config EDMA to copy 1D FFT symbol to HWA memory */
                    MmwDemo_config2DSingleBin_EDMA(obj,
                                              (uint32_t)(&obj->radarCube[RangeBinSize * obj->objOut[idx].rangeIdx]),
                                              (uint32_t)MMW_HWA_2DFFT_SINGLERBIN_INP
                                              );
                    /* Trigger single 2D FFT */
                    MmwDemo_dataPathTrigger2DSingleBin(obj);

                    /* Wait until HWA done with FFT */
                    MmwDemo_dataPathWait2DFFTDetObj(obj);
                }

                /* Save the output from one RangeBin 2D FFT result */
                memcpy((void *)dstAddrAzim,
                        (void *)((uint32_t)MMW_HWA_2DFFT_SINGLERBIN_OUT + obj->objOut[idx].dopplerIdx * dopplerBinSize),

                        sizeof(uint32_t) * obj->numVirtualAntAzim);
                dstAddrAzim += obj->numVirtualAntAzim * sizeof(uint32_t);

                if(obj->numVirtualAntElev > 0)
                {
                    memcpy((void *)dstAddrElev,
                            (void *)(srcAddrElev + obj->objOut[idx].dopplerIdx * dopplerBinSize) ,

                            sizeof(uint32_t) * obj->numVirtualAntElev);

                    dstAddrElev += obj->numVirtualAntElev * sizeof(uint32_t);
                }

                previousIdx = obj->objOut[idx].rangeIdx;

            }
        }
    }
    
    /* Azimuth heap hep doppler compensation */
    if (obj->cliCfg->guiMonSel.rangeAzimuthHeatMap)
    {
        /* Rx channel gain/phase offset compensation */
        MmwDemo_rxChanPhaseBiasCompensation(obj,
                                            obj->numRangeBins,
                                            obj->azimuthStaticHeatMap,
                                            (cmplx16ImRe_t *)NULL);
    
        /* Doppler compensation for Azimuth heat map */
        MmwDemo_azimHeapMapDopplerCompensation(
                                        obj->azimuthStaticHeatMap,
                                        0, /* dopplerIdx = 0 */
                                        obj->numTxAntennas,
                                        obj->numRxAntennas,
                                        obj->numVirtualAntAzim,
                                        obj->numVirtualAntElev,
                                        obj->numDopplerBins,
                                        obj->numRangeBins);
    }
    
    obj->numObjOut = numObjOut;

    /* Angle estimation */
    if(obj->numVirtualAntAzim == 1)
    {
        oneQFormat = 1 << obj->xyzOutputQFormat;
       /*If there is only one virtual antenna, there is no
         need of azimuth FFT as azimuth can not be estimated.*/
        for(idx=0; idx < numObjOut; idx++)
        {
            range = obj->objOut[idx].rangeIdx * obj->rangeResolution;
            if(range < 0)
            {
                obj->objOut[idx].y = (int16_t) (range * oneQFormat - 0.5);
            }
            else
            {
                obj->objOut[idx].y = (int16_t) (range * oneQFormat + 0.5);
            }
            obj->objOut[idx].x = 0;
            obj->objOut[idx].z = 0;
            obj->objOut[idx].peakVal = obj->rangeDopplerLogMagMatrix[obj->objOut[idx].rangeIdx*obj->numDopplerBins +
                obj->objOut[idx].dopplerIdx];

            obj->objOut[idx].dopplerIdx = DOPPLER_IDX_TO_SIGNED(obj->objOut[idx].dopplerIdx , obj->numDopplerBins);
        }
    }
    else if (numObjOut > 0)
    {

        /* Rx channel gain/phase offset compensation */
        MmwDemo_rxChanPhaseBiasCompensation(obj,
                                            numObjOut,
                                            (cmplx16ImRe_t *) MMW_HWA_ANGLE_AZIM_INP,
                                            (cmplx16ImRe_t *) MMW_HWA_ANGLE_ELEV_INP);

        /* Doppler Compensation */
        if (obj->numVirtualAntAzim > obj->numRxAntennas)
        {
            /* For detected objects : doppler compensation is done before Angle estimation */
            MmwDemo_dopplerCompensation(numObjOut,
                                            (uint32_t*) MMW_HWA_ANGLE_AZIM_INP,
                                            (uint32_t*) MMW_HWA_ANGLE_ELEV_INP,
                                            obj->objOut,
                                            (uint32_t *)MMW_HWA_ANGLE_AZIM_INP,
                                            (uint32_t *)MMW_HWA_ANGLE_ELEV_INP,
                                            obj->numTxAntennas,
                                            obj->numRxAntennas,
                                            obj->numVirtualAntAzim,
                                            obj->numVirtualAntElev,
                                            obj->numDopplerBins);
        }


        /* Convert doppler index to signed index */
        for(idx=0; idx < numObjOut; idx++)
        {
            obj->objOut[idx].dopplerIdx = DOPPLER_IDX_TO_SIGNED(obj->objOut[idx].dopplerIdx , obj->numDopplerBins);
        }

        /* Angle estimation - Azimuth FFT */
        MmwDemo_configAngleEstimation_HWA(obj, numObjOut);
        /* Ensure R4F writes to HWA memory is flushed out */
        MEM_BARRIER();
        MmwDemo_dataPathTriggerAngleEstimation(obj);
        MmwDemo_dataPathWaitAngleEstimation(obj);
        /* Azimuth FFT done!*/

        angleEstimationAzimElev((uint16_t *)MMW_HWA_ANGLE_AZIM_ABS_OUT,
                                (uint16_t *)MMW_HWA_ANGLE_AZIM_CPLX_OUT,
                                (uint16_t *)MMW_HWA_ANGLE_ELEV_CPLX_OUT,
                                obj);
    }
}

/**
 *  @b Description
 *  @n
 *      Power of 2 round up function.
 */
uint32_t MmwDemo_pow2roundup (uint32_t x)
{
    uint32_t result = 1;
    while(x > result)
    {
        result <<= 1;
    }
    return result;
}

/**
 *  @b Description
 *  @n
 *      Delete semaphores.
 */
void MmwDemo_dataPathDeleteSemaphore(MmwDemo_DataPathObj *obj)
{
    Semaphore_delete(&obj->EDMA_1Ddone_semHandle);
    Semaphore_delete(&obj->EDMA_2Ddone_semHandle);
    Semaphore_delete(&obj->EDMA_CFARdone_semHandle);
    Semaphore_delete(&obj->HWA_done_semHandle);
}
