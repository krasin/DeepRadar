/**
 *   @file  config_edma_util.h
 *
 *   @brief
 *      EDMA Configuration Utility API definitions.
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
#ifndef _CONFIGEDMA_H
#define _CONFIGEDMA_H

#include <ti/drivers/edma/edma.h>

/**
 *  @b Description
 *  @n
 *     Utility function for configuring a simple EDMA transfer used for transferring
 *     a specific  32-bit word (from a table of 16 words each containing a 1-hot signature)
 *     to the HWA's DMA completion register. The counts and addresses
 *     can be obtained from HWA APIs issued for configuring HWA.
 *     This is used to implement the DMA based trigger mode
 *     of the HWACC (search for "DMA based trigger" in the HWACC spec document)
 *
 *  @param[in]  handle  EDMA handle.
 *  @param[in]  chId    Channel ID
 *  @param[in]  isEventTriggered true if chId is event triggered else false
 *  @param[in]  pSrcAddress Pointer to source address.
 *  @param[in]  pDestAddress Pointer to destination address.
 *  @param[in]  aCount A count.
 *  @param[in]  bCount B count.
 *  @param[in]  cCount C count.
 *  @param[in]  linkChId Link Channel Id (or rather PaRAM Id) for shadow linking
 *             (reloading to prevent reprogramming).
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 */
int32_t EDMAutil_configHwaOneHotSignature(EDMA_Handle handle,
    uint8_t chId,
    bool isEventTriggered,
    uint32_t *pSrcAddress,
    uint32_t *pDestAddress,
    uint16_t aCount,
    uint16_t bCount,
    uint16_t cCount,
    uint16_t linkChId);

/**
 *  @b Description
 *  @n
 *     Utility function for configuring EDMA for a transpose transfer
 *    (used to write 1D-FFT output to L3 in a transpose fashion).
 *
 *  @param[in]  handle      EDMA handle.
 *  @param[in]  chId        Channel ID
 *  @param[in]  linkChId    Link Channel Id (or rather PaRAM Id)
 *                          for shadow linking (reloading to prevent reprogramming).
 *  @param[in]  chainChId   Chain Channel Id, chId will be chained to this.
 *  @param[in]  pSrcAddress Pointer to source address.
 *  @param[in]  pDestAddress Pointer to destination address.
 *  @param[in]  numAnt      Number of receive antennae.
 *  @param[in]  numRangeBins Number of range bins.
 *  @param[in]  numChirpsPerFrame Number of chirps per frame.
 *  @param[in]  isIntermediateChainingEnabled Set to 'true' if intermediate transfer chaining is to be
 *                 enabled.
 *  @param[in]  isFinalChainingEnabled Set to 'true' if final transfer chaining to be
 *                 enabled.
 *  @param[in]  isTransferCompletionEnabled Set to 'true' if final transfer completion
 *                 indication is to be enabled.
 *  @param[in]  transferCompletionCallbackFxn Transfer completion call back function.
 *  @param[in]  transferCompletionCallbackFxnArg Argument for transfer completion call back function.
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 */
int32_t EDMAutil_configHwaTranspose(EDMA_Handle handle,
    uint8_t   chId,
    uint16_t  linkChId,
    uint8_t   chainChId,
    uint32_t  *pSrcAddress,
    uint32_t  *pDestAddress,
    uint8_t   numAnt,
    uint16_t  numRangeBins,
    uint16_t  numChirpsPerFrame,
    bool      isIntermediateChainingEnabled,
    bool      isFinalChainingEnabled,
    bool      isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
	uintptr_t transferCompletionCallbackFxnArg);

/**
 *  @b Description
 *  @n
 *     Utility function for configuring EDMA for a contiguous (as opposed to transpose)
 *    transfer.
 *
 *  @param[in]  handle      EDMA handle.
 *  @param[in]  chId        Channel ID
 *  @param[in]  isEventTriggered true if chId is event triggered else false.
 *  @param[in]  linkChId    Link Channel Id (or rather PaRAM Id)
 *                          for shadow linking (reloading to prevent reprogramming).
 *  @param[in]  chainChId   Chain Channel Id, chId will be chained to this.
 *  @param[in]  pSrcAddress Pointer to source address.
 *  @param[in]  pDestAddress Pointer to destination address.
 *  @param[in]  numBytes    Number of bytes, determines aCount (the 1st dimension of transfer).
 *  @param[in]  numBlocks   Number of blocks, determines bCount (the 2nd dimension of transfer).
 *  @param[in]  srcIncrBytes Source increment bytes, determines sourceBindex
 *                           (jump in source address in bytes after every 1st
 *                            dimension (aCount related) transfer).
 *  @param[in]  dstIncrBytes Destination increment bytes, determines destinatinoBindex
 *                           (jump in destination address in bytes after every 1st
 *                            dimension (aCount related) transfer).
 *  @param[in]  isIntermediateChainingEnabled Set to 'true' if intermediate transfer chaining is to be
 *                 enabled.
 *  @param[in]  isFinalChainingEnabled Set to 'true' if final transfer chaining to be
 *                 enabled.
 *  @param[in]  isTransferCompletionEnabled Set to 'true' if final transfer completion
 *                 indication is to be enabled.
 *  @param[in]  transferCompletionCallbackFxn Transfer completion call back function.
 *  @param[in]  transferCompletionCallbackFxnArg Argument for transfer completion call back function.
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 */
int32_t EDMAutil_configHwaContiguous(EDMA_Handle handle,
    uint8_t   chId,
    bool isEventTriggered,
    uint8_t   linkChId,
    uint8_t   chainChId,
    uint32_t  *pSrcAddress,
    uint32_t  *pDestAddress,
    uint16_t  numBytes,
    uint16_t  numBlocks,
    uint16_t  srcIncrBytes,
    uint16_t  dstIncrBytes,
    bool      isIntermediateChainingEnabled,
    bool      isFinalChainingEnabled,
    bool      isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
	uintptr_t transferCompletionCallbackFxnArg);

/**
 *  @b Description
 *  @n
 *     Utility function for configuring EDMA for a Sync A transfer
 *    transfer.
 *
 *  @param[in]  handle                  EDMA handle.
 *  @param[in]  srcBuff                 Pointer to source buffer.
 *  @param[in]  dstBuff                 Pointer to destination buffer.
 *  @param[in]  chId                    Channel ID
 *  @param[in]  chainId                 Chain Channel Id, chId will be chained to this.
 *  @param[in]  isEventTriggered        true if chId is event triggered else false.
 *  @param[in]  shadowParamId           Shadow Param Id
 *  @param[in]  aCount                  A count.
 *  @param[in]  bCount                  B count.
 *  @param[in]  srcBIdx                 Source B index
 *  @param[in]  dstBIdx                 Destination B index
 *  @param[in]  eventQueueId            Event Queue Id on which to schedule the transfer.
 *  @param[in]  isFinalTransferInterruptEnabled Set to 'true' if final transfer interrupt to be enabled
 *  @param[in]  isFinalChainingEnabled  Set to 'true' if final transfer chaining to be enabled
 *  @param[in]  transferCompletionCallbackFxn Transfer completion call back function.
 *  @param[in]  transferCompletionCallbackFxnArg Transfer completion call back function argument.
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 */
int32_t EDMAutil_configSyncAwithChaining(EDMA_Handle handle,
    uint8_t *srcBuff,
    uint8_t *dstBuff,
    uint8_t chId,
    uint8_t chainId,
    bool isEventTriggered,
    uint16_t shadowParamId,
    uint16_t aCount,
    uint16_t bCount,
    int16_t srcBIdx,
    int16_t dstBIdx,
    uint8_t eventQueueId,
    bool isFinalTransferInterruptEnabled,
    bool isFinalChainingEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t transferCompletionCallbackFxnArg);
#endif
