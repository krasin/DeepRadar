/**
 *   @file  mmw_lvds_stream.c
 *
 *   @brief
 *      Implements LVDS stream functionality.
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

 
/*LVDS feature is unverified code. Conflicts with data path processing and should not be used.*/ 
 
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/utils/Load.h>

/* MMWSDK Include Files. */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/cbuff/cbuff.h>
#include <ti/utils/hsiheader/hsiheader.h>

/* MMWAVE Demo Include Files */
#include <ti/demo/xwr14xx/mmw/mmw.h>
#include <ti/demo/xwr14xx/mmw/data_path.h>

extern MmwDemo_MCB    gMmwMCB; 

 /**
 *  @b Description
 *  @n
 *      This function initializes/configures the LVDS
 *      streaming EDMA resources.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void MmwDemo_LVDSStream_EDMAInit (void)
{
    gMmwMCB.lvdsStream.hwSessionEDMAChannelAllocatorIndex = 0;
    gMmwMCB.lvdsStream.swSessionEDMAChannelAllocatorIndex = 0;
 
    /* Populate the LVDS Stream HW Session EDMA Channel Table: */              
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[0].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_0;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[0].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_0;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[1].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_1;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[1].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_1;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[2].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_2;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[2].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_2;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[3].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_3;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[3].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_3;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[4].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_4;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[4].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_4;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[5].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_5;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[5].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_5;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[6].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_6;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[6].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_6;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[7].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_7;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[7].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_7;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[8].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_8;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[8].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_8;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[9].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_9;
    gMmwMCB.lvdsStream.hwSessionEDMAChannelTable[9].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_9;

    /* Populate the LVDS Stream SW Session EDMA Channel Table: */
    gMmwMCB.lvdsStream.swSessionEDMAChannelTable[0].chainChannelsId       = MMW_LVDS_STREAM_SW_SESSION_EDMA_CH_0;
    gMmwMCB.lvdsStream.swSessionEDMAChannelTable[0].shadowLinkChannelsId  = MMW_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0;
    gMmwMCB.lvdsStream.swSessionEDMAChannelTable[1].chainChannelsId       = MMW_LVDS_STREAM_SW_SESSION_EDMA_CH_1;
    gMmwMCB.lvdsStream.swSessionEDMAChannelTable[1].shadowLinkChannelsId  = MMW_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_1;
} 
 
 /**
 *  @b Description
 *  @n
 *      This is the LVDS streaming init function. 
 *      It initializes the necessary modules
 *      that implement the streaming.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_LVDSStreamInit (void)
{
    CBUFF_InitCfg           initCfg;
    int32_t                 retVal = MINUS_ONE;
    int32_t                 errCode;

    /*************************************************************************************
     * Open the CBUFF Driver:
     *************************************************************************************/
    memset ((void *)&initCfg, 0, sizeof(CBUFF_InitCfg));

    /* Populate the configuration: */
    initCfg.socHandle                 = gMmwMCB.socHandle;
    initCfg.enableECC                 = 0U;
    initCfg.crcEnable                 = 1U;
    /* Up to 1 SW session + 1 HW session can be configured for each frame. Therefore max session is 2. */
    initCfg.maxSessions               = 2U;
    initCfg.enableDebugMode           = false;
    initCfg.interface                 = CBUFF_Interface_LVDS;
    initCfg.outputDataFmt             = CBUFF_OutputDataFmt_16bit;
    initCfg.u.lvdsCfg.crcEnable       = 0U;
    initCfg.u.lvdsCfg.msbFirst        = 1U;
    /* Enable all lanes available on the platform*/
    initCfg.u.lvdsCfg.lvdsLaneEnable  = 0xFU;
    initCfg.u.lvdsCfg.ddrClockMode    = 1U;
    initCfg.u.lvdsCfg.ddrClockModeMux = 1U;

    /* Initialize the CBUFF Driver: */
    gMmwMCB.lvdsStream.cbuffHandle = CBUFF_init (&initCfg, &errCode);
    if (gMmwMCB.lvdsStream.cbuffHandle == NULL)
    {
        /* Error: Unable to initialize the CBUFF Driver */
        //System_printf("Error: CBUFF_init failed with [Error=%d]\n", errCode);
        MmwDemo_debugAssert (0);
        goto exit;
    }

    /* Initialize the HSI Header Module: */
    if (HSIHeader_init (&initCfg, &errCode) < 0)
    {
        /* Error: Unable to initialize the HSI Header Module */
        //System_printf("Error: HSIHeader_init failed with [Error=%d]\n", errCode);
        MmwDemo_debugAssert (0);
        goto exit;
    }

    /* Populate EDMA resources */
    MmwDemo_LVDSStream_EDMAInit();
    
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function that allocates CBUFF-EDMA channel
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 */
static void MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    if(ptrEDMAInfo->dmaNum == 0)
    {
        ptrEDMACfg->chainChannelsId      = MMW_LVDS_STREAM_CBUFF_EDMA_CH_0;
        ptrEDMACfg->shadowLinkChannelsId = MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0;        
    }
    else if(ptrEDMAInfo->dmaNum == 1)
    {
        ptrEDMACfg->chainChannelsId      = MMW_LVDS_STREAM_CBUFF_EDMA_CH_1;
        ptrEDMACfg->shadowLinkChannelsId = MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_1;        
    }    
    else
    {
        /* Max of 2 CBUFF sessions can be configured*/
        MmwDemo_debugAssert (0);
    }
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel allocation function
 *      which allocates EDMA channels for CBUFF HW Session
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_LVDSStream_EDMAAllocateCBUFFHwChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    int32_t         retVal = MINUS_ONE;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMCB.lvdsStream;

    if(ptrEDMAInfo->isFirstEDMAChannel)
    {
        MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel(ptrEDMAInfo, ptrEDMACfg);
        retVal = 0;
    }
    else
    {

        /* Sanity Check: Are there sufficient EDMA channels? */
        if (streamMCBPtr->hwSessionEDMAChannelAllocatorIndex >= MMWDEMO_LVDS_STREAM_HW_SESSION_MAX_EDMA_CHANNEL)
        {
            /* Error: All the EDMA channels are allocated */
            //System_printf ("Error: MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel failed. HW channel index=%d\n", streamMCBPtr->hwSessionEDMAChannelAllocatorIndex);
            MmwDemo_debugAssert (0);
            goto exit;
        }

        /* Copy over the allocated EDMA configuration. */
        memcpy ((void *)ptrEDMACfg,
                (void*)&streamMCBPtr->hwSessionEDMAChannelTable[streamMCBPtr->hwSessionEDMAChannelAllocatorIndex],
                sizeof(CBUFF_EDMAChannelCfg));

        /* Increment the allocator index: */
        streamMCBPtr->hwSessionEDMAChannelAllocatorIndex++;

        /* EDMA Channel allocated successfully */
        retVal = 0;
    }    

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel allocation function
 *      which allocates EDMA channels for CBUFF SW Session
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_LVDSStream_EDMAAllocateCBUFFSwChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    int32_t         retVal = MINUS_ONE;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMCB.lvdsStream;

    if(ptrEDMAInfo->isFirstEDMAChannel)
    {
        MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel(ptrEDMAInfo,ptrEDMACfg);
        retVal = 0;
    }
    else
    {
        /* Sanity Check: Are there sufficient EDMA channels? */
        if (streamMCBPtr->swSessionEDMAChannelAllocatorIndex >= MMWDEMO_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL)
        {
            /* Error: All the EDMA channels are allocated */
            //System_printf ("Error: MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel failed. SW channel index=%d\n", streamMCBPtr->swSessionEDMAChannelAllocatorIndex);
            MmwDemo_debugAssert (0);                
            goto exit;
        }
        
        /* Copy over the allocated EDMA configuration. */
        memcpy ((void *)ptrEDMACfg,
                (void*)&streamMCBPtr->swSessionEDMAChannelTable[streamMCBPtr->swSessionEDMAChannelAllocatorIndex],
                sizeof(CBUFF_EDMAChannelCfg));
        
        /* Increment the allocator index: */
        streamMCBPtr->swSessionEDMAChannelAllocatorIndex++;
        
        /* EDMA Channel allocated successfully */
        retVal = 0;
    }    

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel free function which frees EDMA channels
 *      which had been allocated for use by a CBUFF HW Session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_EDMAFreeCBUFFHwChannel (CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    uint8_t    index;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMCB.lvdsStream;

    if((ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_0) ||
       (ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_1))
    {
        /*This is the CBUFF trigger channel. It is not part of the resource table so
          nothing needs to be done*/
        goto exit;  
    }

    for (index = 0U; index < MMWDEMO_LVDS_STREAM_HW_SESSION_MAX_EDMA_CHANNEL; index++) 
    {
        /* Do we have a match? */
        if (memcmp ((void*)ptrEDMACfg,
                    (void*)&streamMCBPtr->hwSessionEDMAChannelTable[index],
                    sizeof(CBUFF_EDMAChannelCfg)) == 0)
        {
            /* Yes: Decrement the HW Session index */
            streamMCBPtr->hwSessionEDMAChannelAllocatorIndex--;
            goto exit;
        }
    }

    /* Sanity Check: We should have had a match. An assertion is thrown to indicate that the EDMA channel
     * being cleaned up does not belong to the table*/
    MmwDemo_debugAssert (0);

exit:
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel free function which frees EDMA channels
 *      which had been allocated for use by a CBUFF SW Session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_EDMAFreeCBUFFSwChannel (CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    uint8_t    index;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMCB.lvdsStream;

    if((ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_0) ||
       (ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_1))
    {
        /*This is the CBUFF trigger channel. It is not part of the resource table so
          nothing needs to be done*/
        goto exit;  
    }

    for (index = 0U; index < MMWDEMO_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL; index++)
    {
        /* Do we have a match? */
        if (memcmp ((void*)ptrEDMACfg,
                    (void*)&streamMCBPtr->swSessionEDMAChannelTable[index],
                    sizeof(CBUFF_EDMAChannelCfg)) == 0)
        {
            /* Yes: Decrement the SW Session index */
            streamMCBPtr->swSessionEDMAChannelAllocatorIndex--;
            goto exit;
        }
    }

    /* Sanity Check: We should have had a match. An assertion is thrown to indicate that the EDMA channel
     * being cleaned up does not belong to the table*/
    MmwDemo_debugAssert (0);

exit:
    return;
}


/**
 *  @b Description
 *  @n
 *      This function deletes the hardware session and any HSI
 *      header associated with it. 
 *
 *  @param[in]  sessionHandle
 *      Handle to the session
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_LVDSStreamDeleteHwSession (CBUFF_SessionHandle sessionHandle)
{
    int32_t     errCode;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMCB.lvdsStream;
    
    /* Delete session*/
    if (CBUFF_deleteSession (sessionHandle, &errCode) < 0)
    {
        /* Error: Unable to delete the session. */
        MmwDemo_debugAssert(0);
        return;
    }
    
    gMmwMCB.lvdsStream.hwSessionHandle = NULL;
    
    /* Did we stream out with the HSI Header? */
    if (gMmwMCB.cliCfg.lvdsStreamCfg.isHeaderEnabled)
    {
        /* Delete the HSI Header: */
        if (HSIHeader_deleteHeader (&streamMcb->hwSessionHSIHeader, &errCode) < 0)
        {
            /* Error: Unable to delete the HSI Header */
            MmwDemo_debugAssert(0);
            return;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function deletes the SW session and any HSI
 *      header associated with it. 
 *
 *  @param[in]  sessionHandle
 *      Handle to the session
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_LVDSStreamDeleteSwSession (CBUFF_SessionHandle sessionHandle)
{
    int32_t     errCode;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMCB.lvdsStream;
    
    /* Delete session*/
    if (CBUFF_deleteSession (sessionHandle, &errCode) < 0)
    {
        /* Error: Unable to delete the session. */
        MmwDemo_debugAssert(0);
        return;
    }
    
    gMmwMCB.lvdsStream.swSessionHandle = NULL;
    
    /* Did we stream out with the HSI Header? */
    if (gMmwMCB.cliCfg.lvdsStreamCfg.isHeaderEnabled)
    {
        /* Delete the HSI Header: */
        if (HSIHeader_deleteHeader (&streamMcb->swSessionHSIHeader, &errCode) < 0)
        {
            /* Error: Unable to delete the HSI Header */
            MmwDemo_debugAssert(0);
            return;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This is the registered callback function which is invoked after the
 *      frame done interrupt is received for the hardware session.
 *
 *  @param[in]  sessionHandle
 *      Handle to the session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_HwTriggerFrameDone (CBUFF_SessionHandle sessionHandle)
{
    int32_t     errCode;
    
    /* Increment stats*/
    gMmwMCB.lvdsStream.hwFrameDoneCount++;
    
    if(sessionHandle != NULL)
    {
        /* If there is a software session configured, we need to 
           deactivate the HW session here.*/
        if(gMmwMCB.cliCfg.lvdsStreamCfg.isSwEnabled == 1)
        {
            if(CBUFF_deactivateSession (sessionHandle, &errCode) < 0)
            {
                /* Error: Unable to deactivate the session. */
                DebugP_assert(0);
                return;
            }
        }    
    }
    else
    {
        DebugP_assert(0);
    }    
    
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the registered callback function which is invoked after the
 *      frame done interrupt is received for the SW session.
 *
 *  @param[in]  sessionHandle
 *      Handle to the session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_SwTriggerFrameDone (CBUFF_SessionHandle sessionHandle)
{    
    int32_t     errCode;

    /* Increment stats*/
    gMmwMCB.lvdsStream.swFrameDoneCount++;
    
    if(sessionHandle != NULL)
    {
        if(CBUFF_deactivateSession (sessionHandle, &errCode) < 0)
        {
            /* Error: Unable to deactivate the session. */
            DebugP_assert(0);
            return;
        }
        
        /*If a HW session has been configured, we need to
          enable it here. */
        if(gMmwMCB.lvdsStream.hwSessionHandle != NULL)  
        {        
            if(CBUFF_activateSession (gMmwMCB.lvdsStream.hwSessionHandle, &errCode) < 0)
            {
                DebugP_assert(0);
            }
        }        
    }
    else
    {
        DebugP_assert(0);
    }    
    
    return;    
}

/**
 *  @b Description
 *  @n
 *      This is the LVDS streaming config function. 
 *      It configures the sessions for the LVDS streaming.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_LVDSStreamHwConfig (MmwDemo_DataPathObj *dataPathObj)
{
    CBUFF_SessionCfg          sessionCfg;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMCB.lvdsStream;
    int32_t                   errCode;
    int32_t                   retVal = MINUS_ONE;

    memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));
    
    /* Populate the configuration: */
    sessionCfg.executionMode          = CBUFF_SessionExecuteMode_HW;
    sessionCfg.edmaHandle             = dataPathObj->edmaHandle; 
    sessionCfg.allocateEDMAChannelFxn = MmwDemo_LVDSStream_EDMAAllocateCBUFFHwChannel;
    sessionCfg.freeEDMAChannelFxn     = MmwDemo_LVDSStream_EDMAFreeCBUFFHwChannel;
    sessionCfg.frameDoneCallbackFxn   = MmwDemo_LVDSStream_HwTriggerFrameDone;
    sessionCfg.dataType               = CBUFF_DataType_COMPLEX;
    sessionCfg.u.hwCfg.dataMode       = (CBUFF_DataMode)gMmwMCB.cliCfg.adcBufCfg.chInterleave;
    
    /* Populate the HW Session configuration: */
    sessionCfg.u.hwCfg.adcBufHandle      = dataPathObj->adcbufHandle;
    sessionCfg.u.hwCfg.numADCSamples     = dataPathObj->numAdcSamples;
    sessionCfg.u.hwCfg.numChirpsPerFrame = dataPathObj->numChirpsPerFrame;
    sessionCfg.u.hwCfg.chirpMode         = dataPathObj->cliCfg->adcBufCfg.chirpThreshold;
    sessionCfg.u.hwCfg.opMode            = CBUFF_OperationalMode_CHIRP;
    
    switch(gMmwMCB.cliCfg.lvdsStreamCfg.dataFmt)
    {
        case 1:
            sessionCfg.u.hwCfg.dataFormat = CBUFF_DataFmt_ADC_DATA;
        break;
        case 2:
            sessionCfg.u.hwCfg.dataFormat = CBUFF_DataFmt_CP_ADC;
        break;
        case 3:
            sessionCfg.u.hwCfg.dataFormat = CBUFF_DataFmt_ADC_CP;
        break;
        case 4:
            sessionCfg.u.hwCfg.dataFormat = CBUFF_DataFmt_CP_ADC_CQ;
            sessionCfg.u.hwCfg.cqSize[0] = 0;
            sessionCfg.u.hwCfg.cqSize[1] = HSIHeader_toCBUFFUnits(dataPathObj->datapathCQ.sigImgMonTotalSize);
            sessionCfg.u.hwCfg.cqSize[2] = HSIHeader_toCBUFFUnits(dataPathObj->datapathCQ.satMonTotalSize);
        break;
        default:
            //System_printf ("Error: lvdsStreamCfg dataFmt %d is invalid\n", dataPathObj->cliCfg->lvdsStreamCfg.dataFmt);
            MmwDemo_debugAssert(0);
        break;
    }    
        
    if(dataPathObj->cliCfg->lvdsStreamCfg.isHeaderEnabled) 
    {    
        /* Create the HSI Header to be used for the HW Session: */ 
        if (HSIHeader_createHeader (&sessionCfg, false, &(streamMcb->hwSessionHSIHeader), &errCode) < 0)
        {
            /* Error: Unable to create the HSI Header; report the error */
            System_printf("Error: MmwDemo_LVDSStream_config unable to create HW HSI header with [Error=%d]\n", errCode);
            goto exit;
        }
        
        /* Setup the header in the CBUFF session configuration: */
        sessionCfg.header.size    = HSIHeader_getHeaderSize(&streamMcb->hwSessionHSIHeader);
        sessionCfg.header.address = (uint32_t)&(streamMcb->hwSessionHSIHeader);
    }    
       
    /* Create the HW Session: */
    streamMcb->hwSessionHandle = CBUFF_createSession (gMmwMCB.lvdsStream.cbuffHandle, &sessionCfg, &errCode);
                                                      
    if (streamMcb->hwSessionHandle == NULL)
    {
        /* Error: Unable to create the CBUFF hardware session */
        System_printf("Error: MmwDemo_LVDSStream_config unable to create the CBUFF hardware session with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Control comes here implies that the LVDS Stream has been configured successfully */
    retVal = 0;

exit:
    return retVal;
}

 /**
 *  @b Description
 *  @n
 *      This is the LVDS streaming config function. 
 *      It configures the sessions for the LVDS streaming.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_LVDSStreamSwConfig (MmwDemo_DataPathObj *dataPathObj)
{
    CBUFF_SessionCfg          sessionCfg;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMCB.lvdsStream;
    int32_t                   errCode;
    int32_t                   retVal = MINUS_ONE;

    memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));
    
    /* Populate the configuration: */
    sessionCfg.executionMode                     = CBUFF_SessionExecuteMode_SW;
    sessionCfg.edmaHandle                        = dataPathObj->edmaHandle; 
    sessionCfg.allocateEDMAChannelFxn            = MmwDemo_LVDSStream_EDMAAllocateCBUFFSwChannel;
    sessionCfg.freeEDMAChannelFxn                = MmwDemo_LVDSStream_EDMAFreeCBUFFSwChannel;
    sessionCfg.frameDoneCallbackFxn              = MmwDemo_LVDSStream_SwTriggerFrameDone;
    sessionCfg.dataType                          = CBUFF_DataType_COMPLEX; 
    sessionCfg.u.swCfg.userBufferInfo[0].size    = HSIHeader_toCBUFFUnits(sizeof(MmwDemo_LVDSUserDataHeader_t));
    sessionCfg.u.swCfg.userBufferInfo[0].address = (uint32_t)&(streamMcb->userDataHeader);
    if(dataPathObj->numObjOut != 0)
    {
        sessionCfg.u.swCfg.userBufferInfo[1].size    = HSIHeader_toCBUFFUnits((dataPathObj->numObjOut) * sizeof(MmwDemo_detectedObj));
        sessionCfg.u.swCfg.userBufferInfo[1].address = (uint32_t)&dataPathObj->objOut[0];
    }    
    else
    {
        sessionCfg.u.swCfg.userBufferInfo[1].size    = 0;
        sessionCfg.u.swCfg.userBufferInfo[1].address = 0;
    }    
    
    /* Do we need to enable the header? */
    if(dataPathObj->cliCfg->lvdsStreamCfg.isHeaderEnabled) 
    {    
        /* Create the HSI Header to be used for the SW Session: */ 
        if (HSIHeader_createHeader (&sessionCfg, true, &(streamMcb->swSessionHSIHeader), &errCode) < 0)
        {
            /* Error: Unable to create the HSI Header; report the error */
            System_printf("Error: MmwDemo_LVDSStream_config unable to create HW HSI header with [Error=%d]\n", errCode);
            goto exit;
        }
        
        /* Setup the header in the CBUFF session configuration: */
        sessionCfg.header.size    = HSIHeader_getHeaderSize(&streamMcb->swSessionHSIHeader);
        sessionCfg.header.address = (uint32_t)&(streamMcb->swSessionHSIHeader);
    }    

    /* Create the SW Session. */
    streamMcb->swSessionHandle = CBUFF_createSession (gMmwMCB.lvdsStream.cbuffHandle, &sessionCfg, &errCode);
    
    if (streamMcb->swSessionHandle == NULL)
    {
        /* Error: Unable to create the CBUFF SW session */
        System_printf("Error: MmwDemo_LVDSStream_config unable to create the CBUFF SW session with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Control comes here implies that the LVDS Stream has been configured successfully */
    retVal = 0;

exit:
    return retVal;
}

 /**
 *  @b Description
 *  @n
 *      Function that deletes all active LVDS stream
 *      sessions.
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_LVDSStreamDelete (void)
{
    int32_t errCode;
    
    /* Delete any active streaming session */
    if(gMmwMCB.lvdsStream.hwSessionHandle != NULL)
    {
        CBUFF_deactivateSession (gMmwMCB.lvdsStream.hwSessionHandle, &errCode);
        MmwDemo_LVDSStreamDeleteHwSession(gMmwMCB.lvdsStream.hwSessionHandle);
    }
    
    if(gMmwMCB.lvdsStream.swSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSwSession(gMmwMCB.lvdsStream.swSessionHandle);
    }
}                
