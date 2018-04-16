/**
 *   @file  mmw.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
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
#ifndef MMW_DEMO_H
#define MMW_DEMO_H

#include <ti/common/mmwave_error.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/hwa/hwa.h>
#include <ti/drivers/edma/edma.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/drivers/osal/DebugP.h>

#include <ti/sysbios/knl/Semaphore.h>

#include "post_processing.h"
#include "data_path.h"
#include "mmw_lvds_stream.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_Stats_t
{
    /*! @brief   Counter which tracks the number of frame trigger events from BSS */
    uint64_t     frameTriggerReady;
    
    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     failedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports received
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     calibrationReports;

     /*! @brief   Counter which tracks the number of sensor stop events received
      *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     sensorStopped;
}MmwDemo_Stats;


/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_MCB_t
{
    /*! @brief   Configuration which is used to execute the demo */
    MmwDemo_Cfg                 cfg;
    
    /*! @brief CLI related configuration */
    MmwDemo_CliCfg_t           cliCfg;

    /*! @brief   CLI related configuration */
    MmwDemo_CliCommonCfg_t      cliCommonCfg;


    /*! * @brief   Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief   UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief   UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief   This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief   Data Path object */
    MmwDemo_DataPathObj         dataPathObj;

    /*! @brief   Demo Stats */
    MmwDemo_Stats               stats;
    
    /*! @brief   this structure is used to hold all the relevant information 
     for the mmw demo LVDS stream*/
    /*Unverified code. Conflicts with data path processing and should not be used.*/ 
    MmwDemo_LVDSStream_MCB_t    lvdsStream;

} MmwDemo_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern MmwDemo_MCB    gMmwMCB;
extern void MmwDemo_dataPathConfig (void);
extern int32_t MmwDemo_dataPathStart (void);
extern void MmwDemo_dataPathStop (MmwDemo_DataPathObj *obj);

/* Sensor Management Module Exported API */
extern int32_t MmwDemo_sensorMgmtInit (void);
extern int32_t MmwDemo_sensorMgmtDeinit (void);
extern void MmwDemo_notifySensorStart(bool doReconfig);
extern void MmwDemo_notifySensorStop(void);
extern void MmwDemo_waitSensorStopComplete(void);
extern int32_t MmwDemo_waitSensorStartComplete(void);
extern void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line);
extern void MmwDemo_notifyBssSensorStop(void);
extern void MmwDemo_notifyDathPathStop(void);
#define MmwDemo_debugAssert(expression) {                                      \
                                         _MmwDemo_debugAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }
                                        
#ifdef __cplusplus
}
#endif

#endif /* MMW_DEMO_H */

