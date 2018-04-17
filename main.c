/**
 *   @file  main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
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
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>
#include <ti/sysbios/utils/Load.h>


/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/demo/io_interface/mmw_output.h>

#include "config_edma_util.h"
#include "config_hwa_util.h"
#include "post_processing.h"

/* Demo Include Files */
#include "mmw.h"
#include "data_path.h"
#include <ti/demo/io_interface/mmw_config.h>
#include <ti/demo/utils/mmwDemo_monitor.h>

/* These address offsets are in bytes, when configure address offset in hardware,
   these values will be converted to number of 128bits */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET        0U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET         0x400U

extern mmwHwaBuf_t gMmwHwaMemBuf[MMW_HWA_NUM_MEM_BUFS];

extern uint32_t log2Approx(uint32_t x);

/*! L3 RAM buffer */
uint8_t gMmwL3[SOC_XWR14XX_MSS_L3RAM_SIZE];
#pragma DATA_SECTION(gMmwL3, ".l3ram");

/* Data memory for CQ:Rx Saturation - 16 bit CQ  format */
rlRfRxSaturationCqData_t gCQRxSatMonMemory;

/* Data memory for CQ:Signal & Image band monitor  - 16 bit CQ format */
rlRfSigImgPowerCqData_t gCQRxSigImgMemory;

/*! L3 heap for convenience of partitioning L3 RAM */
MmwDemoMemPool_t gMmwL3heap =
{
    &gMmwL3[0],
    SOC_XWR14XX_MSS_L3RAM_SIZE,
    0
};

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB    gMmwMCB;


/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern void MmwDemo_CLIInit (void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);

void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathConfig(void);
void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathStop (MmwDemo_DataPathObj *obj);

void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                    MmwDemo_DataPathObj *obj);

void MmwDemo_initTask(UArg arg0, UArg arg1);
void MmwDemo_dataPathTask(UArg arg0, UArg arg1);
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Get a handle for ADCBuf.
 */
void MmwDemo_ADCBufOpen(MmwDemo_DataPathObj *obj)
{
    ADCBuf_Params       ADCBufparams;
    /*****************************************************************************
     * Start ADCBUF driver:
     *****************************************************************************/
    /* ADCBUF Params initialize */
    ADCBuf_Params_init(&ADCBufparams);
    ADCBufparams.chirpThreshold = 1;
    ADCBufparams.continousMode  = 0;

    /* Open ADCBUF driver */
    obj->adcbufHandle = ADCBuf_open(0, &ADCBufparams);
    if (obj->adcbufHandle == NULL)
    {
        //System_printf("Error: Unable to open the ADCBUF driver\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: ADCBUF Instance(0) %p has been opened successfully\n", obj->adcbufHandle);
}



/**
 *  @b Description
 *  @n
 *      Configures ADCBuf and returns the number of RxAntennas
 */
int32_t MmwDemo_ADCBufConfig(MmwDemo_DataPathObj *dataPathObj)
{

    ADCBuf_dataFormat   dataFormat;
    ADCBuf_RxChanConf   rxChanConf;
    uint8_t             channel;
    int32_t             retVal = 0;
    uint8_t             numBytePerSample = 0;
    MmwDemo_ADCBufCfg*  ptrAdcbufCfg;
    uint32_t            chirpThreshold;
    uint32_t            rxChanMask = 0xF;

    ptrAdcbufCfg = &dataPathObj->cliCfg->adcBufCfg;

    /* Check if ADC configuration is supported:*/
    /* ADC out bits: must be 16 Bits */
    MmwDemo_debugAssert(gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcBits == 2);
    
    /* ADC data format: must be complex */
    /*adcCfg command*/
    if((gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcOutFmt != 1) &&
       (gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcOutFmt != 2))
    {
        MmwDemo_debugAssert(0);
    }    
    /*adcbufCfg command*/
    MmwDemo_debugAssert(ptrAdcbufCfg->adcFmt == 0);
    
    /* ADC channel interleave mode: must be interleaved */
    MmwDemo_debugAssert(ptrAdcbufCfg->chInterleave == 0);

    
    /*****************************************************************************
     * Disable all ADCBuf channels
     *****************************************************************************/
    if ((retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
    {
       //System_printf("Error: Disable ADCBuf channels failed with [Error=%d]\n", retVal);
       MmwDemo_debugAssert (0);
       goto exit;
    }

    /* Complex dataFormat has 4 bytes */
    numBytePerSample =  4;

    /* Configure ADC buffer data format */
    dataFormat.adcOutFormat       = ptrAdcbufCfg->adcFmt;
    dataFormat.sampleInterleave   = ptrAdcbufCfg->iqSwapSel;
    dataFormat.channelInterleave  = ptrAdcbufCfg->chInterleave;

    /* Debug Message: */
    /*System_printf("Debug: Start ADCBuf driver dataFormat=%d, sampleSwap=%d, interleave=%d, chirpThreshold=%d\n",
                   dataFormat.adcOutFormat, dataFormat.sampleInterleave, dataFormat.channelInterleave,
                   ptrAdcbufCfg->chirpThreshold);*/

    retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)&dataFormat);
    if (retVal < 0)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }

    memset((void*)&rxChanConf, 0, sizeof(ADCBuf_RxChanConf));

    /* Enable Rx Channel indicated in channel configuration */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(gMmwMCB.cfg.openCfg.chCfg.rxChannelEn & (0x1<<channel))
        {
            /* Populate the receive channel configuration: */
            rxChanConf.channel = channel;
            retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
            if (retVal < 0)
            {
                MmwDemo_debugAssert (0);
                goto exit;
            }
            rxChanConf.offset  += dataPathObj->numAdcSamples * numBytePerSample;
        }
    }

    chirpThreshold = ptrAdcbufCfg->chirpThreshold;

    /* Set the chirp threshold: */
    retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_SET_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        MmwDemo_debugAssert (0);
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      EDMA transfer completion callback for CQ transfers
 *
 *  @param[in] arg                      Transfer completion callback argument
 *  @param[in] transferCompletionCode   Transfer completion code
 *
 *  @retval
 *      None
 */
void MmwDemo_EDMA_CQTransferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode)
{
    MmwDemo_DataPathObj *obj = (MmwDemo_DataPathObj *)arg;

    switch (transferCompletionCode)
    {
        case MMW_EDMA_RXSAT_TRANSFER_COMPLETION:
            /* Chained EDMA channels complete on 2nd EDMA channel */
            if(obj->datapathCQ.anaMonCfg->sigImgMonEn)
            {
                obj->datapathCQ.sigImgBandEdmaCnt++;
                MmwDemo_debugAssert(*(uint8_t *)obj->datapathCQ.sigImgData <= obj->datapathCQ.sigImgMonCfg->numSlices);
            }
            if(obj->datapathCQ.anaMonCfg->rxSatMonEn)
            {
                obj->datapathCQ.rxSatEdmaCnt++;
                MmwDemo_debugAssert(*(uint8_t *)obj->datapathCQ.rxSatData <= obj->datapathCQ.rxSatMonCfg->numSlices);
            }
            
        break;
        
        default:
            MmwDemo_debugAssert(0);
        break;
    }
}


/**
 *  @b Description
 *  @n
 *      Configures all CQ EDMA channels and param sets used in data path processing
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      -1 if error, 0 for no error
 */
int32_t MmwDemo_dataPathConfigCQEdma(MmwDemo_DataPathObj *obj)
{
    uint32_t    eventQueue;
    int32_t     retVal = 0;
    uint16_t    aCnt;

    /*****************************************************
     * EDMA configuration for getting CQ data from CQ buffer
     * to Datapath CQ storage
     *****************************************************/
    /* Use event queue 1 for CQ because when event queue 0 is used for CQ 
       and LVDS streaming is enabled (LVDS streaming uses event queue 0)
       then there is data corruption on the saved CQ data that is EDMAed below.
       Problem being investigated.*/       
    eventQueue = 1U;

    /*
        EDMA configurations for 2 chained EDMA channels:
        sigImgMonEn = 1, rxSatMonEn = 1,
            acnt: sigImgMonTotalSize, satMonTotalSize
        sigImgMonEn = 1, rxSatMonEn = 0,
            acnt: sigImgMonTotalSize, 0
        sigImgMonEn = 0, rxSatMonEn = 1,
            acnt: 0, satMonTotalSize     
    */    
    if(obj->datapathCQ.anaMonCfg->sigImgMonEn)
    {
        aCnt = obj->datapathCQ.sigImgMonTotalSize;
    }
    else
    {
        /* This becomes a dummy paRAM set */
        aCnt = 0;
    }

    /* Configure EDMA channel for sigImg monitor */
    retVal = EDMAutil_configSyncAwithChaining(obj->edmaHandle,
        (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.sigImgMonAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.sigImgData,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        EDMA_TPCC0_REQ_DFE_CHIRP_AVAIL, 
        MMW_EDMA_CH_RX_SATURATION_MON,
        true,
        EDMA_TPCC0_REQ_DFE_CHIRP_AVAIL,
        aCnt,
        1,
        0,
        0,
        eventQueue,
        false, /*isFinalTransferInterruptEnabled */
        true,  /* isFinalChainingEnabled */
        NULL,
        (uintptr_t) NULL);
        
    if (retVal != EDMA_NO_ERROR)
    {
        return -1;
    }

    if(obj->datapathCQ.anaMonCfg->rxSatMonEn)
    {
        aCnt = obj->datapathCQ.satMonTotalSize;
    }
    else
    {
        /* This becomes a dummy paRAM set */
        aCnt = 0;
    }
    
    /* Configure EDMA channel for RX saturation monitor */
    retVal = EDMAutil_configSyncAwithChaining(obj->edmaHandle,
        (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.satMonAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.rxSatData,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        MMW_EDMA_CH_RX_SATURATION_MON, 
        MMW_EDMA_CH_RX_SATURATION_MON,
        false,
        MMW_EDMA_CH_RX_SATURATION_MON,
        aCnt,
        1,
        0,
        0,
        eventQueue,
        true, /*isFinalTransferInterruptEnabled */
        false,  /* isFinalChainingEnabled */
        MmwDemo_EDMA_CQTransferCompletionCallbackFxn,
        (uintptr_t) obj);
    if (retVal != EDMA_NO_ERROR)
    {
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to configure CQ.
 *  @param[in] ptrDataPathObj Pointer to data path object.
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
int32_t MmwDemo_dataPathConfigCQ(MmwDemo_DataPathObj * ptrDataPathObj)
{
    ADCBuf_CQConf               cqConfig;
    rlRxSatMonConf_t*           ptrSatMonCfg;
    rlSigImgMonConf_t*          ptrSigImgMonCfg;
    MmwDemo_AnaMonitorCfg*      ptrAnaMonitorCfg;
    int32_t                     retVal;

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &ptrDataPathObj->cliCommonCfg->anaMonCfg;

    /* Config mmwaveLink to enable Saturation monitor - CQ2 */
    ptrSatMonCfg = &ptrDataPathObj->cliCommonCfg->cqSatMonCfg[ptrDataPathObj->validProfileIdx];

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        retVal = mmwDemo_cfgRxSaturationMonitor(ptrSatMonCfg);
        if(retVal < 0)
        {
            System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d\n", retVal);

            MmwDemo_debugAssert(0);
        }
    }

    /* Config mmwaveLink to enable Signal Image band monitor - CQ1 */
    ptrSigImgMonCfg = &ptrDataPathObj->cliCommonCfg->cqSigImgMonCfg[ptrDataPathObj->validProfileIdx];

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        retVal = mmwDemo_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if(retVal < 0)
        {
            System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d\n", retVal);

            MmwDemo_debugAssert(0);
        }
    }

    /* Config CQ */
    if ((ptrAnaMonitorCfg->rxSatMonEn) || (ptrAnaMonitorCfg->sigImgMonEn))
    {
        /* CQ driver config */
        memset((void *)&cqConfig, 0, sizeof(ADCBuf_CQConf));
        cqConfig.cqDataWidth = 0; /* 16bit for mmw demo */

        /* CQ1 starts from the beginning of the buffer, address should be 16 bytes aligned */
        cqConfig.cq1AddrOffset = MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET;  
        cqConfig.cq2AddrOffset = MMW_DEMO_CQ_RXSAT_ADDR_OFFSET;  /* address should be 16 bytes aligned. */

        retVal = ADCBuf_control(ptrDataPathObj->adcbufHandle, ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if (retVal < 0)
        {
            System_printf ("Error: Unable to configure CQ, errorCode[%d]\n", retVal);
            return -1;
        }
    }

    /* Save config pointer */
    ptrDataPathObj->datapathCQ.rxSatMonCfg = ptrSatMonCfg;
    ptrDataPathObj->datapathCQ.sigImgMonCfg = ptrSigImgMonCfg;
    ptrDataPathObj->datapathCQ.anaMonCfg= ptrAnaMonitorCfg;

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* Save CQ-Signal & Image band energy info in datapath object */
        ptrDataPathObj->datapathCQ.sigImgMonAddr = ADCBUF_MMWave_getCQBufAddr(ptrDataPathObj->adcbufHandle,
                                                                                   ADCBufMMWave_CQType_CQ1,
                                                                                   &retVal);
        MmwDemo_debugAssert (ptrDataPathObj->datapathCQ.sigImgMonAddr != NULL);

        /* This is for 16bit format in mmw demo */
        ptrDataPathObj->datapathCQ.sigImgMonTotalSize = (ptrSigImgMonCfg->numSlices + 1U) * 2U;

        /* Allocate data memory */
        ptrDataPathObj->datapathCQ.sigImgData = &gCQRxSigImgMemory;

    }
    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        /* Save CQ-Rx Saturation info in datapath object */
        ptrDataPathObj->datapathCQ.satMonAddr =
                    ADCBUF_MMWave_getCQBufAddr(ptrDataPathObj->adcbufHandle,
                                                   ADCBufMMWave_CQType_CQ2,
                                                   &retVal);
        MmwDemo_debugAssert    (ptrDataPathObj->datapathCQ.satMonAddr != NULL);

        /* This is for 16bit format in mmw demo */
        ptrDataPathObj->datapathCQ.satMonTotalSize = ptrSatMonCfg->numSlices + 1U;

        /* Allocate data memory */
        ptrDataPathObj->datapathCQ.rxSatData = &gCQRxSatMonMemory;
    }

    /* EDMA configuration for CQ */
    if ((ptrAnaMonitorCfg->rxSatMonEn) || (ptrAnaMonitorCfg->sigImgMonEn))
    {
        retVal = MmwDemo_dataPathConfigCQEdma(ptrDataPathObj);
        if(retVal < 0)
        {
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      parses Profile, Chirp and Frame config and extracts parameters
 *      needed for processing chain configuration
 */
bool MmwDemo_parseProfileAndChirpConfig(MmwDemo_DataPathObj *dataPathObj)
{
    uint16_t    frameChirpStartIdx;
    uint16_t    frameChirpEndIdx;
    int16_t     frameTotalChirps;
    int32_t     errCode;
    uint32_t    profileLoopIdx, chirpLoopIdx;
    bool        foundValidProfile = false;
    uint16_t    channelTxEn = gMmwMCB.cfg.openCfg.chCfg.txChannelEn;
    uint8_t     channel;
    uint8_t     numRxAntennas = 0;

    /* Find number of enabled channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(gMmwMCB.cfg.openCfg.chCfg.rxChannelEn & (0x1<<channel))
        {
            /* Track the number of receive channels: */
            numRxAntennas++;
        }
    }
    dataPathObj->numRxAntennas = numRxAntennas;

    /* read frameCfg chirp start/stop*/
    frameChirpStartIdx = gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpStartIdx;
    frameChirpEndIdx = gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpEndIdx;
    frameTotalChirps = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* loop for profiles and find if it has valid chirps */
    /* we support only one profile in this processing chain */
    for (profileLoopIdx=0;
        ((profileLoopIdx<MMWAVE_MAX_PROFILE)&&(foundValidProfile==false));
        profileLoopIdx++)
    {
        uint32_t    mmWaveNumChirps = 0;
        bool        validProfileHasElevation=false;
        bool        validProfileHasOneTxPerChirp=false;
        uint16_t    validProfileTxEn = 0;
        uint16_t    validChirpTxEnBits[32]={0};
        MMWave_ProfileHandle profileHandle;

        profileHandle = gMmwMCB.cfg.ctrlCfg.u.frameCfg.profileHandle[profileLoopIdx];
        if (profileHandle == NULL)
            continue; /* skip this profile */

        /* get numChirps for this profile; skip error checking */
        MMWave_getNumChirps(profileHandle,&mmWaveNumChirps,&errCode);
        /* loop for chirps and find if it has valid chirps for the frame
           looping around for all chirps in a profile, in case
           there are duplicate chirps
         */
        for (chirpLoopIdx=1;chirpLoopIdx<=mmWaveNumChirps;chirpLoopIdx++)
        {
            MMWave_ChirpHandle chirpHandle;
            /* get handle and read ChirpCfg */
            if (MMWave_getChirpHandle(profileHandle,chirpLoopIdx,&chirpHandle,&errCode)==0)
            {
                rlChirpCfg_t chirpCfg;
                if (MMWave_getChirpCfg(chirpHandle,&chirpCfg,&errCode)==0)
                {
                    uint16_t chirpTxEn = chirpCfg.txEnable;
                    /* do chirps fall in range and has valid antenna enabled */
                    if ((chirpCfg.chirpStartIdx >= frameChirpStartIdx) &&
                        (chirpCfg.chirpEndIdx <= frameChirpEndIdx) &&
                        ((chirpTxEn & channelTxEn) > 0))
                    {
                        uint16_t idx = 0;
                        for (idx=(chirpCfg.chirpStartIdx-frameChirpStartIdx);idx<=(chirpCfg.chirpEndIdx-frameChirpStartIdx);idx++)
                        {
                            validChirpTxEnBits[idx] = chirpTxEn;
                            foundValidProfile = true;
                        }

                    }
                }
            }
        }
        /* now loop through unique chirps and check if we found all of the ones
           needed for the frame and then determine the azimuth/elevation antenna
           configuration
         */
        if (foundValidProfile) {
            int16_t nonElevFirstChirpIdx = -1;
            for (chirpLoopIdx=0;chirpLoopIdx<frameTotalChirps;chirpLoopIdx++)
            {
                bool validChirpHasElevation=false;
                bool validChirpHasOneTxPerChirp=false;
                uint16_t chirpTxEn = validChirpTxEnBits[chirpLoopIdx];
                if (chirpTxEn == 0) {
                    /* this profile doesnt have all the needed chirps */
                    foundValidProfile = false;
                    break;
                }
                /* check if this is an elevation TX chirp */
                validChirpHasElevation = (chirpTxEn==0x2);
                validProfileHasElevation |= validChirpHasElevation;
                /* if not, then check the MIMO config */
                if (!validChirpHasElevation)
                {
                    validChirpHasOneTxPerChirp = ((chirpTxEn==0x1) || (chirpTxEn==0x4));
                    /* if this is the first chirp without elevation, record the chirp's
                       MIMO config as profile's MIMO config. We dont handle intermix
                       at this point */
                    if (nonElevFirstChirpIdx==-1) {
                        validProfileHasOneTxPerChirp = validChirpHasOneTxPerChirp;
                        nonElevFirstChirpIdx = chirpLoopIdx;
                    }
                    /* check the chirp's MIMO config against Profile's MIMO config */
                    if (validChirpHasOneTxPerChirp != validProfileHasOneTxPerChirp)
                    {
                        /* this profile doesnt have all chirps with same MIMO config */
                        foundValidProfile = false;
                        break;
                    }
                }
                /* save the antennas actually enabled in this profile */
                validProfileTxEn |= chirpTxEn;
            }
        }

        /* found valid chirps for the frame; mark this profile valid */
        if (foundValidProfile==true) {
            rlProfileCfg_t  profileCfg;
            uint32_t        numTxAntAzim = 0;
            uint32_t        numTxAntElev = 0;

            dataPathObj->validProfileIdx = profileLoopIdx;
            dataPathObj->numTxAntennas = 0;
            if (validProfileHasElevation)
            {
                numTxAntElev = 1;
            }
            if (!validProfileHasOneTxPerChirp)
            {
                numTxAntAzim=1;
            }
            else
            {
                if (validProfileTxEn & 0x1)
                {
                    numTxAntAzim++;
                }
                if (validProfileTxEn & 0x4)
                {
                    numTxAntAzim++;
                }
            }
            /*System_printf("Azimuth Tx: %d (MIMO:%d), Elev Tx:%d\n",
                            numTxAntAzim,validProfileHasMIMO,numTxAntElev);*/
            dataPathObj->numTxAntennas = numTxAntAzim + numTxAntElev;
            dataPathObj->numVirtualAntAzim = numTxAntAzim * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntElev = numTxAntElev * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntennas = dataPathObj->numVirtualAntAzim + dataPathObj->numVirtualAntElev;

            /* Sanity Check: Ensure that the number of antennas is within system limits */
            MmwDemo_debugAssert (dataPathObj->numVirtualAntennas > 0);
            MmwDemo_debugAssert (dataPathObj->numVirtualAntennas <= (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL));

            /* Get the profile configuration: */
            if (MMWave_getProfileCfg(profileHandle,&profileCfg, &errCode) < 0)
            {
                MmwDemo_debugAssert(0);
                return false;
            }

#ifndef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
            /* Check frequency slope */
            if (profileCfg.freqSlopeConst < 0)
            {
                System_printf("Frequency slope must be positive\n");
                MmwDemo_debugAssert(0);
            }
#endif

            dataPathObj->numAdcSamples = profileCfg.numAdcSamples;
            dataPathObj->numRangeBins = MmwDemo_pow2roundup(dataPathObj->numAdcSamples);
            dataPathObj->numChirpsPerFrame = frameTotalChirps *
                                              gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numLoops;

            dataPathObj->numAngleBins = MMW_NUM_ANGLE_BINS;
            dataPathObj->numDopplerBins = dataPathObj->numChirpsPerFrame/dataPathObj->numTxAntennas;
            dataPathObj->numRangeBinsPerTransfer = MMW_NUM_RANGE_BINS_PER_TRANSFER;
            dataPathObj->rangeResolution = MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC * profileCfg.digOutSampleRate * 1e3 /
                    (2 * profileCfg.freqSlopeConst * ((3.6*1e3*900)/(1U << 26)) * 1e12 * dataPathObj->numRangeBins);

            dataPathObj->xyzOutputQFormat    = (uint32_t) ceil(log10(16./fabs(dataPathObj->rangeResolution))/log10(2));
            dataPathObj->dataPathMode = DATA_PATH_WITH_ADCBUF;
            dataPathObj->frameStartIntCounter = 0;
            dataPathObj->interFrameProcToken = 0;
        }
    }
    return foundValidProfile;
}

/** @brief Transmits detection data over UART
*
*    The following data is transmitted:
*    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
*       and icluding the number of TLV items
*    TLV Items:
*    3. If logMagRange flag is set,  rangeProfile,
*       size = number of range bins * sizeof(uint16_t)
*    7. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
*       range cubed matrix, size = number of Rx Azimuth virtual antennas *
*       number of chirps per frame * sizeof(uint32_t)
*    8. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
*       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
*   @param[in] uartHandle   UART driver handle
*   @param[in] obj          Pointer data path object MmwDemo_DataPathObj
*/

void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                    MmwDemo_DataPathObj *obj)
{
    MmwDemo_output_message_header header;
    MmwDemo_GuiMonSel   *pGuiMonSel;
    uint32_t tlvIdx = 0;
    uint32_t i;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];

    MmwDemo_output_message_tl   tl[MMWDEMO_OUTPUT_MSG_MAX];

    /* Get Gui Monitor configuration */
    pGuiMonSel = &gMmwMCB.cliCfg.guiMonSel;

    /* Clear message header */
    memset((void *)&header, 0, sizeof(MmwDemo_output_message_header));
    /* Header: */
    header.platform = 0xA1443;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.numDetectedObj = 0;
    header.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                        (MMWAVE_SDK_VERSION_MINOR << 16) |
                        (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(MmwDemo_output_message_header);
    if (pGuiMonSel->logMagRange)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * obj->numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeAzimuthHeatMap)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP;
        tl[tlvIdx].length = obj->numRangeBins * obj->numVirtualAntAzim * sizeof(uint32_t);
        packetLen += sizeof(MmwDemo_output_message_tl) +  tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeDopplerHeatMap)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP;
        tl[tlvIdx].length = obj->numRangeBins * obj->numDopplerBins * sizeof(uint16_t);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }

    header.numTLVs = tlvIdx;
    /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
    header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
            ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles =  Pmu_getCount(0);
    header.frameNumber = obj->frameStartIntCounter;


    UART_writePolling (uartHandle,
                       (uint8_t*)&header,
                       sizeof(MmwDemo_output_message_header));

    tlvIdx = 0;
    /* Send Range profile */
    if (pGuiMonSel->logMagRange)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        for(i = 0; i < obj->numRangeBins; i++)
        {
            UART_writePolling (uartHandle,
                    (uint8_t*)&obj->rangeDopplerLogMagMatrix[i*obj->numDopplerBins],
                    sizeof(uint16_t));
        }
        tlvIdx++;
    }

    /* Send data for static azimuth heatmap */
    if (pGuiMonSel->rangeAzimuthHeatMap)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        UART_writePolling (uartHandle,
                (uint8_t *) obj->azimuthStaticHeatMap,
                obj->numRangeBins * obj->numVirtualAntAzim * sizeof(uint32_t));

        tlvIdx++;
    }

    /* Send data for range/Doppler heatmap */
    if (pGuiMonSel->rangeDopplerHeatMap == 1)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        UART_writePolling (uartHandle,
                (uint8_t*)obj->rangeDopplerLogMagMatrix,
                tl[tlvIdx].length);
        tlvIdx++;
    }

    /* Send  radarCube */
    {
      /* numRangeBins * numDopplerBins * numTxAntennas * numRxAntennas * 4 */
      size_t radarCubeSize = obj->numRangeBins * obj->numDopplerBins * obj->numTxAntennas * obj->numRxAntennas * 4;
      size_t transmitSize = radarCubeSize;
      UART_writePolling(uartHandle,
			(uint8_t*)obj->radarCube,
			transmitSize);
    }

    /* Send padding bytes */
    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
    if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling (uartHandle,
                            (uint8_t*)padding,
                            numPaddingBytes);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to trigger the Front end to start generating chirps.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dataPathStart (void)
{
    MMWave_CalibrationCfg   calibrationCfg;
    int32_t                 errCode = 0;
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;

    dataPathObj->frameStartIntCounter = 0;
    dataPathObj->interFrameProcToken = 0;

    /* Initialize the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start (gMmwMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        /* Error: Unable to start the mmWave control */
        //System_printf ("Error: mmWave Control Start failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
    }
    else
    {
        /* Update data path stop status */
        dataPathObj->datapathStopped = false;
    }

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the data path based on the chirp profile.
 *      After this function is executed, the data path processing will ready to go
 *      when the ADC buffer starts receiving samples corresponding to the chirps.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathConfig (void)
{
    int32_t    retVal = 0;
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;

    /* Configure ADCBuf Config and get the valid number of RX antennas
       do this first as we need the numRxAntennas in MmwDemo_parseProfileAndChirpConfig
       to get the Virtual Antennas */
    /* Parse the profile and chirp configs and get the valid number of TX Antennas */
    if (MmwDemo_parseProfileAndChirpConfig(dataPathObj) == true)
    {

        retVal = mmwDemo_cfgAnalogMonitor(&dataPathObj->cliCommonCfg->anaMonCfg);
        if (retVal != 0)
        {
            System_printf ("Error: rlRfAnaMonConfig returns error = %d\n", retVal);

            MmwDemo_debugAssert(0);
        }

        if (MmwDemo_ADCBufConfig(dataPathObj) < 0)
        {
            //System_printf("Error: ADCBuf config failed \n");
            MmwDemo_debugAssert (0);
        }

        /* Configure CQ */
        MmwDemo_dataPathConfigCQ(dataPathObj);

        /* Now we are ready to allocate and config the data buffers */
        MmwDemo_dataPathCfgBuffers(dataPathObj, &gMmwL3heap);
        /* Configure one-time EDMA and HWA parameters */
        MmwDemo_dataPathConfigCommon(dataPathObj);

        /* Config HWA for 1D processing and keep it ready for immediate processingh
           as soon as Front End starts generating chirps */
        MmwDemo_config1D_HWA(dataPathObj);
        MmwDemo_dataPathTrigger1D(dataPathObj);
    }
    else
    {
        /* no valid profile found - assert! */
        MmwDemo_debugAssert(0);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *  This function is called at the init time from @ref MmwDemo_initTask.
 *  It initializes drivers: ADCBUF, HWA, EDMA, and semaphores used
 *  by  @ref MmwDemo_dataPathTask
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj)
{
    MmwDemo_dataPathObjInit(obj, &gMmwMCB.cliCfg, &gMmwMCB.cliCommonCfg);

    /* Initialize the ADCBUF */
    ADCBuf_init();

    /* Initialize HWA */
    MmwDemo_hwaInit(obj);

    /* Initialize EDMA */
    MmwDemo_edmaInit(obj);
}

void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj)
{
    /*****************************************************************************
     * Start HWA, EDMA and ADCBUF drivers:
     *****************************************************************************/
    MmwDemo_hwaOpen(obj, gMmwMCB.socHandle);
    MmwDemo_edmaOpen(obj);
    MmwDemo_ADCBufOpen(obj);
}

/**
 *  @b Description
 *  @n
 *      The function is used to Stop data path.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathStop (MmwDemo_DataPathObj *obj)
{
    obj->datapathStopped = true;
    if(obj->interFrameProcToken == 0)
    {
        MmwDemo_notifyDathPathStop();
    }
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwMCB.ctrlHandle, &errCode) < 0)
        {
            //System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert (0);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Registered event function to mmwave which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0
 */
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitomplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitomplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0xFFFU;

                    /* Display the calibration status: */
                    CLI_write ("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    /*Received Frame Stop async event from BSS. Post event to sensor management task.*/
                    MmwDemo_notifyBssSensorStop();
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The task is used for data path processing and to transmit the
 *      detected objects through the UART output port.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathTask(UArg arg0, UArg arg1)
{
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;
    uint16_t numDetectedObjects;
    while(1)
    {
        Semaphore_pend(dataPathObj->frameStart_semHandle, BIOS_WAIT_FOREVER);

        Load_update();

        MmwDemo_dataPathWait1D(dataPathObj);
        /* 1st Dimension FFT done! */

        Load_update();

        if(dataPathObj->cliCfg->calibDcRangeSigCfg.enabled)
        {
             if (dataPathObj->cliCfg->calibDcRangeSigCfg.numAvgChirps <  dataPathObj->numDopplerBins)
             {
                 dataPathObj->cliCfg->calibDcRangeSigCfg.enabled = 0;
                 dataPathObj->dcRangeForcedDisableCntr++;
             }
             else
             {
                 MmwDemo_dcRangeSignatureCompensation(dataPathObj);
             }
        }

        MmwDemo_process2D(dataPathObj);
        /* 2nd Dimension FFT done! */

        MmwDemo_processCfar(dataPathObj, &numDetectedObjects);
        /* CFAR done! */

        /* Postprocessing/angle estimation */
        dataPathObj->numHwaCfarDetections = numDetectedObjects;

        MmwDemo_transmitProcessedOutput(gMmwMCB.loggingUartHandle,
                                        dataPathObj);

        dataPathObj->interFrameProcToken--;
        if(dataPathObj->datapathStopped == true)
        {
            MmwDemo_notifyDathPathStop();
        }
        else
        {
            /* Prepare for next frame */
            MmwDemo_config1D_HWA(dataPathObj);
            MmwDemo_dataPathTrigger1D(dataPathObj);
        }
    }
}


/**
 *  @b Description
 *  @n
 *      Frame start interrupt handler
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_frameStartIntHandler(uintptr_t arg)
{
    MmwDemo_DataPathObj * dpObj = &gMmwMCB.dataPathObj;

    /* Increment interrupt counter for debugging purpose */
    dpObj->frameStartIntCounter++;

    /* Check if previous chirp processing has completed */
    MmwDemo_debugAssert(dpObj->interFrameProcToken == 0);
    dpObj->interFrameProcToken++;

    Semaphore_post(dpObj->frameStart_semHandle);

}


/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_initTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;

    /* Debug Message: */
    System_printf("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/

    /* Initialize the UART */
    UART_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Initialize the GPIO */
    GPIO_init ();

    /* Initialize the Data Path: */
    MmwDemo_dataPathInit(&gMmwMCB.dataPathObj);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN6_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINN6_PADBE, SOC_XWR14XX_PINN6_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN5_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINN5_PADBD, SOC_XWR14XX_PINN5_PADBD_MSS_UARTA_RX);

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone   = 1;

    /* Open the UART Instance */
    gMmwMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMCB.commandUartHandle == NULL)
    {
        //System_printf("Error: Unable to open the Command UART Instance\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: UART Instance %p has been opened successfully\n", gMmwMCB.commandUartHandle);

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 0;

    /* Open the Logging UART Instance: */
    gMmwMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMCB.loggingUartHandle == NULL)
    {
        //System_printf("Error: Unable to open the Logging UART Instance\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: UART Instance %p has been opened successfully\n", gMmwMCB.loggingUartHandle);

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* Populate the init configuration: */
    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.socHandle               = gMmwMCB.socHandle;
    initCfg.eventFxn                = MmwDemo_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_Channel_CH1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;

    /* Initialize and setup the mmWave Control module */
    gMmwMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        //System_printf ("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf ("Debug: mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    if (MMWave_sync (gMmwMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to synchronize the mmWave control module */
        //System_printf ("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf ("Debug: mmWave Control Synchronization was successful\n");

    MmwDemo_dataPathOpen(&gMmwMCB.dataPathObj);

    /* Configure banchmark counter */
    Pmu_configureCounter(0, 0x11, FALSE);
    Pmu_startCounter(0);

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = 5;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialize the CLI Module:
     *****************************************************************************/
    MmwDemo_CLIInit();

    /*****************************************************************************
     * Initialize the Sensor Management Module:
     *****************************************************************************/
    if (MmwDemo_sensorMgmtInit() < 0)
        return;

    /* Register Frame start interrupt handler */
    {
        SOC_SysIntListenerCfg  socIntCfg;
        int32_t errCode;

        Semaphore_Params       semParams;

        /* Register frame start interrupt listener */
        socIntCfg.systemInterrupt  = SOC_XWR14XX_DSS_FRAME_START_IRQ;
        socIntCfg.listenerFxn      = MmwDemo_frameStartIntHandler;
        socIntCfg.arg              = (uintptr_t)NULL;
        if (SOC_registerSysIntListener(gMmwMCB.socHandle, &socIntCfg, &errCode) == NULL)
        {
            //System_printf("Error: Unable to register frame start interrupt listener , error = %d\n", errCode);
            MmwDemo_debugAssert (0);
            return;
        }

        Semaphore_Params_init(&semParams);
        semParams.mode = Semaphore_Mode_BINARY;
        gMmwMCB.dataPathObj.frameStart_semHandle = Semaphore_create(0, &semParams, NULL);
    }

    /*****************************************************************************
     * Launch the Main task
     * - The main demo task
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = 4;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_dataPathTask, &taskParams, NULL);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction.
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        //System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return -1;
    }

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMCB, 0, sizeof(MmwDemo_MCB));

    gMmwMCB.socHandle = socHandle;

    /* Initialize the DEMO configuration: */
    gMmwMCB.cfg.sysClockFrequency = (200 * 1000000);
    gMmwMCB.cfg.loggingBaudRate   = 921600;
    gMmwMCB.cfg.commandBaudRate   = 115200;

#if 0
    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the Millimeter Wave Demo\n");
    System_printf ("**********************************************\n");
#endif

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    Task_create(MmwDemo_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}


