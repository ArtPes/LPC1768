/*
 * driver.h - CC31xx/CC32xx Host Driver Implementation
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
*/

#ifndef DRIVER_INT_H_
#define DRIVER_INT_H_

#include "cc3100_simplelink.h"

#include "cc3100_protocol.h"
#include "cc3100_nonos.h"
#include "cc3100_spi.h"
#include "cc3100_netapp.h"
#include "cc3100.h"


/*****************************************************************************/
/* Macro declarations                                                        */
/*****************************************************************************/

/*  2 LSB of the N2H_SYNC_PATTERN are for sequence number
only in SPI interface
support backward sync pattern */
#define N2H_SYNC_PATTERN_SEQ_NUM_BITS            ((uint32_t)0x00000003) /* Bits 0..1    - use the 2 LBS for seq num */
#define N2H_SYNC_PATTERN_SEQ_NUM_EXISTS          ((uint32_t)0x00000004) /* Bit  2       - sign that sequence number exists in the sync pattern */
#define N2H_SYNC_PATTERN_MASK                    ((uint32_t)0xFFFFFFF8) /* Bits 3..31   - constant SYNC PATTERN */
#define N2H_SYNC_SPI_BUGS_MASK                   ((uint32_t)0x7FFF7F7F) /* Bits 7,15,31 - ignore the SPI (8,16,32 bites bus) error bits  */
#define BUF_SYNC_SPIM(pBuf)                      ((*(uint32_t *)(pBuf)) & N2H_SYNC_SPI_BUGS_MASK)
#define N2H_SYNC_SPIM                            (N2H_SYNC_PATTERN    & N2H_SYNC_SPI_BUGS_MASK)
#define N2H_SYNC_SPIM_WITH_SEQ(TxSeqNum)         ((N2H_SYNC_SPIM & N2H_SYNC_PATTERN_MASK) | N2H_SYNC_PATTERN_SEQ_NUM_EXISTS | ((TxSeqNum) & (N2H_SYNC_PATTERN_SEQ_NUM_BITS)))
#define MATCH_WOUT_SEQ_NUM(pBuf)                 ( BUF_SYNC_SPIM(pBuf) ==  N2H_SYNC_SPIM )
#define MATCH_WITH_SEQ_NUM(pBuf, TxSeqNum)       ( BUF_SYNC_SPIM(pBuf) == (N2H_SYNC_SPIM_WITH_SEQ(TxSeqNum)) )
#define N2H_SYNC_PATTERN_MATCH(pBuf, TxSeqNum) \
    ( \
    (  (*((uint32_t *)pBuf) & N2H_SYNC_PATTERN_SEQ_NUM_EXISTS) && ( MATCH_WITH_SEQ_NUM(pBuf, TxSeqNum) ) )	|| \
    ( !(*((uint32_t *)pBuf) & N2H_SYNC_PATTERN_SEQ_NUM_EXISTS) && ( MATCH_WOUT_SEQ_NUM(pBuf          ) ) )	   \
    )

#define OPCODE(_ptr)          (((_SlResponseHeader_t *)(_ptr))->GenHeader.Opcode)
#define RSP_PAYLOAD_LEN(_ptr) (((_SlResponseHeader_t *)(_ptr))->GenHeader.Len - _SL_RESP_SPEC_HDR_SIZE)
#define SD(_ptr)              (((_SocketAddrResponse_u *)(_ptr))->IpV4.sd)
/*  Actual size of Recv/Recvfrom response data  */
#define ACT_DATA_SIZE(_ptr)   (((_SocketAddrResponse_u *)(_ptr))->IpV4.statusOrLen)

#define _SL_PROTOCOL_ALIGN_SIZE(msgLen)             (((msgLen)+3) & (~3))
#define _SL_IS_PROTOCOL_ALIGNED_SIZE(msgLen)        (!((msgLen) & 3))
#define _SL_PROTOCOL_CALC_LEN(pCmdCtrl,pCmdExt)     ((pCmdExt) ? \
                                                     (_SL_PROTOCOL_ALIGN_SIZE(pCmdCtrl->TxDescLen) + _SL_PROTOCOL_ALIGN_SIZE(pCmdExt->TxPayloadLen)) : \
                                                     (_SL_PROTOCOL_ALIGN_SIZE(pCmdCtrl->TxDescLen)))
                                                     

namespace mbed_cc3100 {
	
class cc3100;
	
/*****************************************************************************/
/* Structure/Enum declarations                                               */
/*****************************************************************************/
//typedef void(*_SlSpawnEntryFunc_t)(void* pValue);

typedef struct {
    _SlOpcode_t      Opcode;
    _SlArgSize_t     TxDescLen;
    _SlArgSize_t     RxDescLen;
} _SlCmdCtrl_t;

typedef struct {
    uint16_t  TxPayloadLen;
    int16_t   RxPayloadLen;
    int16_t   ActualRxPayloadLen;
    uint8_t   *pTxPayload;
    uint8_t   *pRxPayload;
} _SlCmdExt_t;


typedef struct _SlArgsData_t {
    uint8_t	 *pArgs;
    uint8_t  *pData;
} _SlArgsData_t;


typedef struct _SlPoolObj_t {
    _SlSyncObj_t	      SyncObj;
    uint8_t               *pRespArgs;
    uint8_t			      ActionID;
    uint8_t			      AdditionalData; /* use for socketID and one bit which indicate supprt IPV6 or not (1=support, 0 otherwise) */
    uint8_t				  NextIndex;

} _SlPoolObj_t;


typedef enum {
    SOCKET_0,
    SOCKET_1,
    SOCKET_2,
    SOCKET_3,
    SOCKET_4,
    SOCKET_5,
    SOCKET_6,
    SOCKET_7,
    MAX_SOCKET_ENUM_IDX,
#ifndef SL_TINY_EXT    
    ACCEPT_ID = MAX_SOCKET_ENUM_IDX,
    CONNECT_ID,
#else
    CONNECT_ID = MAX_SOCKET_ENUM_IDX,
#endif
#ifndef SL_TINY_EXT    
	SELECT_ID,
#endif
	GETHOSYBYNAME_ID,
#ifndef SL_TINY_EXT    
	GETHOSYBYSERVICE_ID,
	PING_ID,
#endif	
    START_STOP_ID,
    RECV_ID
} _SlActionID_e;

typedef struct _SlActionLookup_t {
    uint8_t					ActionID;
    uint16_t				ActionAsyncOpcode;
    _SlSpawnEntryFunc_t		AsyncEventHandler;

} _SlActionLookup_t;


typedef struct {
    uint8_t         TxPoolCnt;
    _SlLockObj_t    TxLockObj;
    _SlSyncObj_t    TxSyncObj;
} _SlFlowContCB_t;

typedef enum {
    RECV_RESP_CLASS,
    CMD_RESP_CLASS,
    ASYNC_EVT_CLASS,
    DUMMY_MSG_CLASS
} _SlRxMsgClass_e;

typedef struct {
    uint8_t                 *pAsyncBuf;         /* place to write pointer to buffer with CmdResp's Header + Arguments */
    uint8_t					ActionIndex;
    _SlSpawnEntryFunc_t     AsyncEvtHandler;    /* place to write pointer to AsyncEvent handler (calc-ed by Opcode)   */
    _SlRxMsgClass_e         RxMsgClass;         /* type of Rx message                                                 */
} AsyncExt_t;

typedef struct {
    _SlCmdCtrl_t         *pCmdCtrl;
    uint8_t              *pTxRxDescBuff;
    _SlCmdExt_t          *pCmdExt;
    AsyncExt_t           AsyncExt;
} _SlFunctionParams_t;

typedef void (*P_INIT_CALLBACK)(uint32_t Status);

typedef struct {
    _SlFd_t                    FD;
    _SlLockObj_t               GlobalLockObj;
    _SlCommandHeader_t         TempProtocolHeader;
    P_INIT_CALLBACK            pInitCallback;

    _SlPoolObj_t               ObjPool[MAX_CONCURRENT_ACTIONS];
    uint8_t					   FreePoolIdx;
    uint8_t					   PendingPoolIdx;
    uint8_t					   ActivePoolIdx;
    uint32_t				   ActiveActionsBitmap;
    _SlLockObj_t               ProtectionLockObj;

    _SlSyncObj_t               CmdSyncObj;
    uint8_t                    IsCmdRespWaited;
    _SlFlowContCB_t            FlowContCB;
    uint8_t	                   TxSeqNum;
    uint8_t	                   RxDoneCnt;
    uint8_t	                   SocketNonBlocking;
	uint8_t	                   SocketTXFailure;
    /* for stack reduction the parameters are globals */
    _SlFunctionParams_t        FunctionParams;

    uint8_t	                   ActionIndex;
}_SlDriverCb_t;

extern volatile uint8_t	RxIrqCnt;

extern _SlDriverCb_t* g_pCB;
typedef uint8_t _SlSd_t;

class cc3100_driver
{

public:

    cc3100_driver(cc3100_nonos &nonos, cc3100_netapp &netapp, cc3100_flowcont &flowcont, cc3100_spi &spi);

    ~cc3100_driver();


    /*****************************************************************************/
    /* Function prototypes                                                       */
    /*****************************************************************************/
    typedef _SlDriverCb_t pDriver;
    
    uint8_t _SlDrvProtectAsyncRespSetting(uint8_t *pAsyncRsp, uint8_t ActionID, uint8_t SocketID);
    
    bool _SL_PENDING_RX_MSG(pDriver* pDriverCB);
    
    void _SlDrvDriverCBInit(void);

    void _SlDrvDriverCBDeinit(void);

    void _SlDrvRxIrqHandler(void *pValue);

    _SlReturnVal_t  _SlDrvCmdOp(_SlCmdCtrl_t *pCmdCtrl , void* pTxRxDescBuff , _SlCmdExt_t* pCmdExt);

    _SlReturnVal_t  _SlDrvCmdSend(_SlCmdCtrl_t *pCmdCtrl , void* pTxRxDescBuff , _SlCmdExt_t* pCmdExt);

    _SlReturnVal_t  _SlDrvDataReadOp(_SlSd_t Sd, _SlCmdCtrl_t *pCmdCtrl , void* pTxRxDescBuff , _SlCmdExt_t* pCmdExt);

    _SlReturnVal_t  _SlDrvDataWriteOp(_SlSd_t Sd, _SlCmdCtrl_t *pCmdCtrl , void* pTxRxDescBuff , _SlCmdExt_t* pCmdExt);
#ifndef SL_TINY_EXT
    int16_t  _SlDrvBasicCmd(_SlOpcode_t Opcode);
#endif
    uint8_t _SlDrvWaitForPoolObj(uint8_t ActionID, uint8_t SocketID);

    void _SlDrvReleasePoolObj(uint8_t pObj);

//    void _SlDrvObjInit(void);

    _SlReturnVal_t   _SlDrvMsgRead(void);
    
    _SlReturnVal_t   _SlDrvMsgWrite(_SlCmdCtrl_t  *pCmdCtrl,_SlCmdExt_t  *pCmdExt, uint8_t *pTxRxDescBuff);

 //   _SlReturnVal_t   _SlDrvMsgWrite(void);

    _SlReturnVal_t   _SlDrvMsgReadCmdCtx(void);

    _SlReturnVal_t   _SlDrvMsgReadSpawnCtx_(void *pValue);

    void             _SlDrvClassifyRxMsg(_SlOpcode_t Opcode );

    _SlReturnVal_t   _SlDrvRxHdrRead(uint8_t *pBuf, uint8_t *pAlignSize);

    void             _SlDrvShiftDWord(uint8_t *pBuf);

    void             _SlAsyncEventGenericHandler(void);

    void			 _SlDrvObjDeInit(void);

    void			 _SlRemoveFromList(uint8_t* ListIndex, uint8_t ItemIndex);

    _SlReturnVal_t	 _SlFindAndSetActiveObj(_SlOpcode_t  Opcode, uint8_t Sd);
    
    uint16_t _SlDrvAlignSize(uint16_t msgLen); 
    void  _SlDrvSyncObjWaitForever(_SlSyncObj_t *pSyncObj);
    void  _SlDrvSyncObjSignal(_SlSyncObj_t *pSyncObj);
    void  _SlDrvObjLock(_SlLockObj_t *pLockObj, _SlTime_t Timeout);
    void  _SlDrvObjLockWaitForever(_SlLockObj_t *pLockObj);
    void  _SlDrvProtectionObjLockWaitForever();
    void  _SlDrvObjUnLock(_SlLockObj_t *pLockObj);
    void  _SlDrvProtectionObjUnLock();
    void  _SlDrvMemZero(void* Addr, uint16_t size);
    void  _SlDrvResetCmdExt(_SlCmdExt_t* pCmdExt);


private:

    cc3100_nonos      &_nonos;
    cc3100_netapp     &_netapp;
    cc3100_flowcont   &_flowcont;
    cc3100_spi        &_spi;

};//class

}//namespace mbed_cc3100

#endif /* __DRIVER_INT_H__ */

