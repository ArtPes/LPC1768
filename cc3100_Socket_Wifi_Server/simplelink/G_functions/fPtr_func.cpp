/*
 *  - CC31xx/CC32xx Host Driver Implementation
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

#include "cc3100_simplelink.h"

#include "cc3100.h"
#include "cc3100_driver.h"
#include "fPtr_func.h"

    
using namespace mbed_cc3100;    
    
cc3100        *_cc3100_;

#ifdef __cplusplus
extern "C" {
#endif 

/* General Events handling*/
#if defined (EXT_LIB_REGISTERED_GENERAL_EVENTS)

typedef _SlEventPropogationStatus_e (*general_callback) (SlDeviceEvent_t *);

static const general_callback  general_callbacks[] =
{
#ifdef SlExtLib1GeneralEventHandler
    SlExtLib1GeneralEventHandler,
#endif

#ifdef SlExtLib2GeneralEventHandler
    SlExtLib2GeneralEventHandler,
#endif

#ifdef SlExtLib3GeneralEventHandler
    SlExtLib3GeneralEventHandler,
#endif

#ifdef SlExtLib4GeneralEventHandler
    SlExtLib4GeneralEventHandler,
#endif

#ifdef SlExtLib5GeneralEventHandler
    SlExtLib5GeneralEventHandler,
#endif
};

#undef _SlDrvHandleGeneralEvents

/********************************************************************
  _SlDrvHandleGeneralEvents
  Iterates through all the general(device) event handlers which are
  registered by   the external libs/user application.
*********************************************************************/
void _SlDrvHandleGeneralEvents(SlDeviceEvent_t *slGeneralEvent)
{
    uint8_t i;

    /* Iterate over all the extenal libs handlers */
    for ( i = 0 ; i < sizeof(general_callbacks)/sizeof(general_callbacks[0]) ; i++ )
    {
        if (EVENT_PROPAGATION_BLOCK == general_callbacks[i](slGeneralEvent) )
        {
            /* exit immediately and do not call the user specific handler as well */
            return;
        }
    }

/* At last call the Application specific handler if registered */
#ifdef sl_GeneralEvtHdlr
    sl_GeneralEvtHdlr(slGeneralEvent);
#endif

}
#endif



/* WLAN Events handling*/

#if defined (EXT_LIB_REGISTERED_WLAN_EVENTS)

typedef _SlEventPropogationStatus_e (*wlan_callback) (SlWlanEvent_t *);

static wlan_callback  wlan_callbacks[] =
{
#ifdef SlExtLib1WlanEventHandler
        SlExtLib1WlanEventHandler,
#endif

#ifdef SlExtLib2WlanEventHandler
        SlExtLib2WlanEventHandler,
#endif

#ifdef SlExtLib3WlanEventHandler
        SlExtLib3WlanEventHandler,
#endif

#ifdef SlExtLib4WlanEventHandler
        SlExtLib4WlanEventHandler,
#endif

#ifdef SlExtLib5WlanEventHandler
        SlExtLib5WlanEventHandler,
#endif
};

#undef _SlDrvHandleWlanEvents

/***********************************************************
  _SlDrvHandleWlanEvents
  Iterates through all the wlan event handlers which are
  registered by the external libs/user application.
************************************************************/
void _SlDrvHandleWlanEvents(SlWlanEvent_t *slWlanEvent)
{
    uint8_t i;

    /* Iterate over all the extenal libs handlers */
    for ( i = 0 ; i < sizeof(wlan_callbacks)/sizeof(wlan_callbacks[0]) ; i++ )
    {
        if ( EVENT_PROPAGATION_BLOCK == wlan_callbacks[i](slWlanEvent) )
        {
            /* exit immediately and do not call the user specific handler as well */
            return;
        }
    }

/* At last call the Application specific handler if registered */
#ifdef sl_WlanEvtHdlr
    sl_WlanEvtHdlr(slWlanEvent);
#endif

}
#endif


/* NetApp Events handling */
#if defined (EXT_LIB_REGISTERED_NETAPP_EVENTS)

typedef _SlEventPropogationStatus_e (*netApp_callback) (SlNetAppEvent_t *);

static const netApp_callback  netApp_callbacks[] =
{
#ifdef SlExtLib1NetAppEventHandler
     SlExtLib1NetAppEventHandler,
#endif

#ifdef SlExtLib2NetAppEventHandler
     SlExtLib2NetAppEventHandler,
#endif

#ifdef SlExtLib3NetAppEventHandler
     SlExtLib3NetAppEventHandler,
#endif

#ifdef SlExtLib4NetAppEventHandler
     SlExtLib4NetAppEventHandler,
#endif

#ifdef SlExtLib5NetAppEventHandler
     SlExtLib5NetAppEventHandler,
#endif
};

#undef _SlDrvHandleNetAppEvents

/************************************************************
  _SlDrvHandleNetAppEvents
  Iterates through all the net app event handlers which are
  registered by   the external libs/user application.
************************************************************/
void _SlDrvHandleNetAppEvents(SlNetAppEvent_t *slNetAppEvent)
{
    uint8_t i;

    /* Iterate over all the extenal libs handlers */
    for ( i = 0 ; i < sizeof(netApp_callbacks)/sizeof(netApp_callbacks[0]) ; i++ )
    {
        if (EVENT_PROPAGATION_BLOCK == netApp_callbacks[i](slNetAppEvent) )
        {
            /* exit immediately and do not call the user specific handler as well */
            return;
        }
    }

/* At last call the Application specific handler if registered */
#ifdef sl_NetAppEvtHdlr
    sl_NetAppEvtHdlr(slNetAppEvent);
#endif

}
#endif


/* Http Server Events handling */
#if defined (EXT_LIB_REGISTERED_HTTP_SERVER_EVENTS)

typedef _SlEventPropogationStatus_e (*httpServer_callback) (SlHttpServerEvent_t*, SlHttpServerResponse_t*);

static const httpServer_callback  httpServer_callbacks[] =
{
#ifdef SlExtLib1HttpServerEventHandler
        SlExtLib1HttpServerEventHandler,
#endif

#ifdef SlExtLib2HttpServerEventHandler
        SlExtLib2HttpServerEventHandler,
#endif

#ifdef SlExtLib3HttpServerEventHandler
        SlExtLib3HttpServerEventHandler,
#endif

#ifdef SlExtLib4HttpServerEventHandler
        SlExtLib4HttpServerEventHandler,
#endif

#ifdef SlExtLib5HttpServerEventHandler
      SlExtLib5HttpServerEventHandler,
#endif
};

#undef _SlDrvHandleHttpServerEvents

/*******************************************************************
  _SlDrvHandleHttpServerEvents
  Iterates through all the http server event handlers which are
  registered by the external libs/user application.
********************************************************************/
void _SlDrvHandleHttpServerEvents(SlHttpServerEvent_t *slHttpServerEvent, SlHttpServerResponse_t *slHttpServerResponse)
{
    _u8 i;

    /* Iterate over all the external libs handlers */
    for ( i = 0 ; i < sizeof(httpServer_callbacks)/sizeof(httpServer_callbacks[0]) ; i++ )
    {
        if ( EVENT_PROPAGATION_BLOCK == httpServer_callbacks[i](slHttpServerEvent, slHttpServerResponse) )
        {
            /* exit immediately and do not call the user specific handler as well */
            return;
        }
    }

/* At last call the Application specific handler if registered */
#ifdef sl_HttpServerCallback
    sl_HttpServerCallback(slHttpServerEvent, slHttpServerResponse);
#endif

}
#endif


/* Socket Events */
#if defined (EXT_LIB_REGISTERED_SOCK_EVENTS)

typedef _SlEventPropogationStatus_e (*sock_callback) (SlSockEvent_t *);

static const sock_callback  sock_callbacks[] =
{
#ifdef SlExtLib1SockEventHandler
        SlExtLib1SockEventHandler,
#endif

#ifdef SlExtLib2SockEventHandler
        SlExtLib2SockEventHandler,
#endif

#ifdef SlExtLib3SockEventHandler
        SlExtLib3SockEventHandler,
#endif

#ifdef SlExtLib4SockEventHandler
        SlExtLib4SockEventHandler,
#endif

#ifdef SlExtLib5SockEventHandler
        SlExtLib5SockEventHandler,
#endif
};

/*************************************************************
  _SlDrvHandleSockEvents
  Iterates through all the socket event handlers which are
  registered by the external libs/user application.
**************************************************************/
void _SlDrvHandleSockEvents(SlSockEvent_t *slSockEvent)
{
    uint8_t i;

    /* Iterate over all the external libs handlers */
    for ( i = 0 ; i < sizeof(sock_callbacks)/sizeof(sock_callbacks[0]) ; i++ )
    {
        if ( EVENT_PROPAGATION_BLOCK == sock_callbacks[i](slSockEvent) )
        {
            /* exit immediately and do not call the user specific handler as well */
            return;
        }
    }

/* At last call the Application specific handler if registered */
#ifdef sl_SockEvtHdlr
    sl_SockEvtHdlr(slSockEvent);
#endif

}

#endif   

/*!
    \brief This function handles ping report events

    \param[in]      pPingReport holds the ping report statistics

    \return         None

    \note

    \warning
*/
void SimpleLinkPingReport(SlPingReport_t *pPingReport)
{
    _cc3100_->SET_STATUS_BIT(g_Status, STATUS_BIT_PING_DONE);

    if(pPingReport == NULL)
        printf(" [PING REPORT] NULL Pointer Error\r\n");

    g_PingPacketsRecv = pPingReport->PacketsReceived;
}


/*******************************************************************************/
/*   _sl_HandleAsync_Accept */
/*******************************************************************************/
#ifndef SL_TINY_EXT
void _sl_HandleAsync_Accept(void *pVoidBuf)
{
    _SocketAddrResponse_u *pMsgArgs = (_SocketAddrResponse_u *)_SL_RESP_ARGS_START(pVoidBuf);

    _cc3100_->_driver._SlDrvProtectionObjLockWaitForever();

    VERIFY_PROTOCOL(( pMsgArgs->IpV4.sd & BSD_SOCKET_ID_MASK) <= SL_MAX_SOCKETS);
    VERIFY_SOCKET_CB(NULL != g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs);

    memcpy(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs, pMsgArgs,sizeof(_SocketAddrResponse_u));
    _cc3100_->_driver._SlDrvSyncObjSignal(&g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].SyncObj);

    _cc3100_->_driver._SlDrvProtectionObjUnLock();
    return;
}

/*******************************************************************************/
/*   _sl_HandleAsync_Connect */
/*******************************************************************************/
void _sl_HandleAsync_Connect(void *pVoidBuf)
{
    _SocketResponse_t *pMsgArgs = (_SocketResponse_t *)_SL_RESP_ARGS_START(pVoidBuf);

    _cc3100_->_driver._SlDrvProtectionObjLockWaitForever();

    VERIFY_PROTOCOL((pMsgArgs->sd & BSD_SOCKET_ID_MASK) <= SL_MAX_SOCKETS);
    VERIFY_SOCKET_CB(NULL != g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs);

    ((_SocketResponse_t *)(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs))->sd = pMsgArgs->sd;
    ((_SocketResponse_t *)(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs))->statusOrLen = pMsgArgs->statusOrLen;

    _cc3100_->_driver._SlDrvSyncObjSignal(&g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].SyncObj);
    _cc3100_->_driver._SlDrvProtectionObjUnLock();
    return;
}

/*******************************************************************************/
/*   _sl_HandleAsync_Select */
/*******************************************************************************/
void _sl_HandleAsync_Select(void *pVoidBuf)
{
    _SelectAsyncResponse_t     *pMsgArgs   = (_SelectAsyncResponse_t *)_SL_RESP_ARGS_START(pVoidBuf);

    _cc3100_->_driver._SlDrvProtectionObjLockWaitForever();    

    VERIFY_SOCKET_CB(NULL != g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs);

    memcpy(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs, pMsgArgs, sizeof(_SelectAsyncResponse_t));

    _cc3100_->_driver._SlDrvSyncObjSignal(&g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].SyncObj);
    _cc3100_->_driver._SlDrvProtectionObjUnLock();

    return;
}

#endif

/******************************************************************************/
/*  _sl_HandleAsync_DnsGetHostByName */
/******************************************************************************/
void _sl_HandleAsync_DnsGetHostByName(void *pVoidBuf)
{
    _GetHostByNameIPv4AsyncResponse_t *pMsgArgs = (_GetHostByNameIPv4AsyncResponse_t *)_SL_RESP_ARGS_START(pVoidBuf);

    _cc3100_->_driver._SlDrvProtectionObjLockWaitForever();

    VERIFY_SOCKET_CB(NULL != g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs);

    /*IPv6 */
    if(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].AdditionalData & SL_NETAPP_FAMILY_MASK) {
        memcpy(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs, pMsgArgs, sizeof(_GetHostByNameIPv6AsyncResponse_t));
    }
    /*IPv4 */
    else 
    {
        memcpy(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs, pMsgArgs, sizeof(_GetHostByNameIPv4AsyncResponse_t));
    }
    
    _cc3100_->_driver._SlDrvSyncObjSignal(&g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].SyncObj);
    _cc3100_->_driver._SlDrvProtectionObjUnLock();
    
    return;
}

/******************************************************************************/

/******************************************************************************
    _sl_HandleAsync_DnsGetHostByService

    CALLER          NWP - Async event on sl_DnsGetHostByService with IPv4 Family


    DESCRIPTION:

                    Async event on sl_DnsGetHostByService command with IPv4 Family.
                    Return service attributes like IP address, port and text according to service name.
                    The user sets a service name Full/Part (see example below), and should get the:
                    1. IP of the service
                    2. The port of service.
                    3. The text of service.

                    Hence it can make a connection to the specific service and use it.
                    It is similar to get host by name method.

                    It is done by a single shot query with PTR type on the service name.



                    Note:
                    1. The return's attributes are belonged to first service that is found.
                    It can be other services with the same service name will response to
                    the query. The results of these responses are saved in the peer cache of the NWP, and
                    should be read by another API.


        PARAMETERS:

                  pVoidBuf - is point to opcode of the event.
                  it contains the outputs that are given to the user

                  outputs description:

                   1.out_pAddr[]                    - output: Contain the IP address of the service.
                   2.out_pPort                      - output: Contain the port of the service.
                   3.inout_TextLen                  - Input:  Contain the max length of the text that the user wants to get.
                                                              it means that if the test of service is bigger that its value than
                                                              the text is cut to inout_TextLen value.
                                                      Output: Contain the length of the text that is returned. Can be full text or part
                                                              of the text (see above).

                   4.out_pText                      - Contain the text of the service (full or part see above- inout_TextLen description).

  *


    RETURNS:        success or fail.

******************************************************************************/
#ifndef SL_TINY_EXT
void _sl_HandleAsync_DnsGetHostByService(void *pVoidBuf)
{
    _GetHostByServiceAsyncResponse_t* Res;
    uint16_t TextLen;
    uint16_t UserTextLen;

    /*pVoidBuf - is point to opcode of the event.*/

    /*set pMsgArgs to point to the attribute of the event.*/
    _GetHostByServiceIPv4AsyncResponse_t *pMsgArgs = (_GetHostByServiceIPv4AsyncResponse_t *)_SL_RESP_ARGS_START(pVoidBuf);

    VERIFY_SOCKET_CB(NULL != g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs);

    /*IPv6*/
    if(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].AdditionalData & SL_NETAPP_FAMILY_MASK) {
        return;
    }
    /*IPv4*/
    else {
        /*************************************************************************************************

        1. Copy the attribute part of the evnt to the attribute part of the response
        memcpy(g_pCB->GetHostByServiceCB.pAsyncRsp, pMsgArgs, sizeof(_GetHostByServiceIPv4AsyncResponse_t));

        set to TextLen the text length of the service.*/
        TextLen = pMsgArgs->TextLen;

        /*Res pointed to mDNS global object struct */
        Res = (_GetHostByServiceAsyncResponse_t*)g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs;



        /*It is 4 bytes so we avoid from memcpy*/
        Res->out_pAddr[0]   = pMsgArgs->Address;
        Res->out_pPort[0]   = pMsgArgs->Port;
        Res->Status         = pMsgArgs->Status;

        /*set to TextLen the text length of the user (input fromthe user).*/
        UserTextLen = Res->inout_TextLen[0];

        /*Cut the service text if the user requested for smaller text.*/
        UserTextLen = (TextLen <= UserTextLen) ? TextLen : UserTextLen;
        Res->inout_TextLen[0] = UserTextLen ;

        /**************************************************************************************************

        2. Copy the payload part of the evnt (the text) to the payload part of the response
        the lenght of the copy is according to the text length in the attribute part. */


        memcpy(Res->out_pText          ,
                  (int8_t *)(& pMsgArgs[1]),   /* & pMsgArgs[1] -> 1st byte after the fixed header = 1st byte of variable text.*/
                  UserTextLen);


        /**************************************************************************************************/
        _cc3100_->_driver._SlDrvSyncObjSignal(&g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].SyncObj);
        return;
    }
}
#endif

/*****************************************************************************/
/*  _sl_HandleAsync_PingResponse */
/*****************************************************************************/
#ifndef SL_TINY_EXT
void _sl_HandleAsync_PingResponse(void *pVoidBuf)
{
    _PingReportResponse_t *pMsgArgs  = (_PingReportResponse_t *)_SL_RESP_ARGS_START(pVoidBuf);
    SlPingReport_t pingReport;

    if(pPingCallBackFunc) {
        _cc3100_->_netapp.CopyPingResultsToReport(pMsgArgs,&pingReport);
        pPingCallBackFunc(&pingReport);
    } else {

        _cc3100_->_driver._SlDrvProtectionObjLockWaitForever();
        VERIFY_SOCKET_CB(NULL != g_pCB->PingCB.PingAsync.pAsyncRsp);

        if (NULL != g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs) {
            memcpy(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs, pMsgArgs, sizeof(_PingReportResponse_t));
            _cc3100_->_driver._SlDrvSyncObjSignal(&g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].SyncObj);
        }
        _cc3100_->_driver._SlDrvProtectionObjUnLock();
    }
    return;
}
#endif

/* ******************************************************************************/
/*  _SlDrvMsgReadSpawnCtx                                                       */
/* ******************************************************************************/
_SlReturnVal_t _SlDrvMsgReadSpawnCtx(void *pValue)
{

#ifdef SL_POLLING_MODE_USED
    int16_t retCode = OSI_OK;
    /*  for polling based systems */
    do {
        retCode = sl_LockObjLock(&g_pCB->GlobalLockObj, 0);
        if ( OSI_OK != retCode ) {
            if (TRUE == g_pCB->IsCmdRespWaited) {
                OSI_RET_OK_CHECK( sl_SyncObjSignal(&g_pCB->CmdSyncObj) );
                return SL_RET_CODE_OK;
            }
        }

    } while (OSI_OK != retCode);

#else
    
    OSI_RET_OK_CHECK(_cc3100_->_nonos.sl_LockObjLock(&g_pCB->GlobalLockObj, NON_OS_LOCK_OBJ_UNLOCK_VALUE, NON_OS_LOCK_OBJ_LOCK_VALUE, SL_OS_WAIT_FOREVER) );
    
#endif
    
    g_pCB->FunctionParams.AsyncExt.pAsyncBuf = NULL;/*  buffer must be allocated by _SlDrvMsgRead */
    g_pCB->FunctionParams.AsyncExt.AsyncEvtHandler= NULL;
    g_pCB->FunctionParams.AsyncExt.RxMsgClass = CMD_RESP_CLASS;/* init to illegal value and verify it's overwritten with the valid one */

    /*  Messages might have been read by CmdResp context. Therefore after */
    /*  getting LockObj, check again where the Pending Rx Msg is still present. */
    if(FALSE == (_cc3100_->_driver._SL_PENDING_RX_MSG(g_pCB))) {
        OSI_RET_OK_CHECK(_cc3100_->_nonos.sl_LockObjUnlock(&g_pCB->GlobalLockObj, NON_OS_LOCK_OBJ_UNLOCK_VALUE));
        return SL_RET_CODE_OK;
    }
   
    VERIFY_RET_OK(_cc3100_->_driver._SlDrvMsgRead());

    g_pCB->RxDoneCnt++;

    switch(g_pCB->FunctionParams.AsyncExt.RxMsgClass) {
        case ASYNC_EVT_CLASS:
            /*  If got here and protected by LockObj a message is waiting  */
            /*  to be read */
            VERIFY_PROTOCOL(NULL != g_pCB->FunctionParams.AsyncExt.pAsyncBuf);

            _cc3100_->_driver._SlAsyncEventGenericHandler();

#if (SL_MEMORY_MGMT == SL_MEMORY_MGMT_STATIC)
            g_pCB->FunctionParams.AsyncExt.pAsyncBuf = NULL;
#else
            free(g_pCB->FunctionParams.AsyncExt.pAsyncBuf);
#endif
            break;
        case DUMMY_MSG_CLASS:
        case RECV_RESP_CLASS:
            /* These types are legal in this context. Do nothing */
            break;
        case CMD_RESP_CLASS:
            /* Command response is illegal in this context. */
            /* No 'break' here: Assert! */
        default:
            VERIFY_PROTOCOL(0);
    }

    OSI_RET_OK_CHECK(_cc3100_->_nonos.sl_LockObjUnlock(&g_pCB->GlobalLockObj, NON_OS_LOCK_OBJ_UNLOCK_VALUE));

    return(SL_RET_CODE_OK);   

}

/***************************************************************************
_sl_HandleAsync_Stop - handles stop signalling to
a waiting object
****************************************************************************/
void _sl_HandleAsync_Stop(void *pVoidBuf)
{
    _BasicResponse_t *pMsgArgs = (_BasicResponse_t *)_SL_RESP_ARGS_START(pVoidBuf);

    VERIFY_SOCKET_CB(NULL != g_pCB->StopCB.pAsyncRsp);

    _cc3100_->_driver._SlDrvProtectionObjLockWaitForever();

    memcpy(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs, pMsgArgs, sizeof(_BasicResponse_t));
    _cc3100_->_driver._SlDrvSyncObjSignal(&g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].SyncObj);
    _cc3100_->_driver._SlDrvProtectionObjUnLock();
    return;
}

/******************************************************************************
_SlDrvDeviceEventHandler - handles internally device async events
******************************************************************************/
void _SlDrvDeviceEventHandler(void *pArgs)
{
    _SlResponseHeader_t *pHdr = (_SlResponseHeader_t *)pArgs;
   
    switch(pHdr->GenHeader.Opcode) {
        case SL_OPCODE_DEVICE_INITCOMPLETE:
            _cc3100_->_sl_HandleAsync_InitComplete(pHdr);
            
            break;
        case SL_OPCODE_DEVICE_STOP_ASYNC_RESPONSE:
            _sl_HandleAsync_Stop(pHdr);
            
            break;
        

        case SL_OPCODE_DEVICE_ABORT:
            {
#if defined (sl_GeneralEvtHdlr) || defined(EXT_LIB_REGISTERED_GENERAL_EVENTS)
                SlDeviceEvent_t      devHandler;
                devHandler.Event = SL_DEVICE_ABORT_ERROR_EVENT; 
                devHandler.EventData.deviceReport.AbortType = *((uint32_t*)pArgs + 2);
                devHandler.EventData.deviceReport.AbortData = *((uint32_t*)pArgs + 3);
                _SlDrvHandleGeneralEvents(&devHandler);
#endif      
            }
        break;    
        case  SL_OPCODE_DEVICE_DEVICEASYNCFATALERROR:
#if defined (sl_GeneralEvtHdlr) || defined(EXT_LIB_REGISTERED_GENERAL_EVENTS)
        {
            _BasicResponse_t *pMsgArgs = (_BasicResponse_t *)_SL_RESP_ARGS_START(pHdr);
            SlDeviceEvent_t devHandler;
            devHandler.Event = SL_DEVICE_FATAL_ERROR_EVENT;
            devHandler.EventData.deviceEvent.status = pMsgArgs->status & 0xFF;
            devHandler.EventData.deviceEvent.sender = (SlErrorSender_e)((pMsgArgs->status >> 8) & 0xFF);
            _SlDrvHandleGeneralEvents(&devHandler);
        }
#endif
        break;
        default:
            //SL_ERROR_TRACE2(MSG_306, "ASSERT: _SlDrvDeviceEventHandler : invalid opcode = 0x%x = %i", pHdr->GenHeader.Opcode, pHdr->GenHeader.Opcode);
            printf("ASSERT: _SlDrvDeviceEventHandler : invalid opcode = 0x%x = %i", pHdr->GenHeader.Opcode, pHdr->GenHeader.Opcode);
            
    }
}

/*****************************************************************************/
/* _SlDrvNetAppEventHandler */
/*****************************************************************************/
void _SlDrvNetAppEventHandler(void *pArgs)
{
    _SlResponseHeader_t *pHdr = (_SlResponseHeader_t *)pArgs;
#if defined(sl_HttpServerCallback) || defined(EXT_LIB_REGISTERED_HTTP_SERVER_EVENTS)
    SlHttpServerEvent_t     httpServerEvent;    
    SlHttpServerResponse_t  httpServerResponse;
#endif
    switch(pHdr->GenHeader.Opcode) {
        case SL_OPCODE_NETAPP_DNSGETHOSTBYNAMEASYNCRESPONSE:
        case SL_OPCODE_NETAPP_DNSGETHOSTBYNAMEASYNCRESPONSE_V6:
            _sl_HandleAsync_DnsGetHostByName(pArgs);
            break;
#ifndef SL_TINY_EXT             
        case SL_OPCODE_NETAPP_MDNSGETHOSTBYSERVICEASYNCRESPONSE:
        case SL_OPCODE_NETAPP_MDNSGETHOSTBYSERVICEASYNCRESPONSE_V6:
            _sl_HandleAsync_DnsGetHostByService(pArgs);
            break;
        case SL_OPCODE_NETAPP_PINGREPORTREQUESTRESPONSE:
            _sl_HandleAsync_PingResponse(pArgs);
            break;
#endif

#if defined(sl_HttpServerCallback) || defined(EXT_LIB_REGISTERED_HTTP_SERVER_EVENTS)            
        case SL_OPCODE_NETAPP_HTTPGETTOKENVALUE: {

            uint8_t *pTokenName;
            slHttpServerData_t Token_value;
            sl_NetAppHttpServerGetToken_t *httpGetToken = (sl_NetAppHttpServerGetToken_t *)_SL_RESP_ARGS_START(pHdr);
            pTokenName = (uint8_t *)((sl_NetAppHttpServerGetToken_t *)httpGetToken + 1);

            httpServerResponse.Response = SL_NETAPP_HTTPSETTOKENVALUE;
            httpServerResponse.ResponseData.token_value.len = MAX_TOKEN_VALUE_LEN;

            /* Reuse the async buffer for getting the token value response from the user */
            httpServerResponse.ResponseData.token_value.data = (uint8_t *)_SL_RESP_ARGS_START(pHdr) + MAX_TOKEN_NAME_LEN;
            httpServerEvent.Event = SL_NETAPP_HTTPGETTOKENVALUE_EVENT;

            httpServerEvent.EventData.httpTokenName.len = httpGetToken->token_name_len;
            httpServerEvent.EventData.httpTokenName.data = pTokenName;

            Token_value.token_name =  pTokenName;

            _SlDrvHandleHttpServerEvents (&httpServerEvent, &httpServerResponse);

            Token_value.value_len = httpServerResponse.ResponseData.token_value.len;
            Token_value.name_len = httpServerEvent.EventData.httpTokenName.len;

            Token_value.token_value = httpServerResponse.ResponseData.token_value.data;


            _cc3100_->_netapp.sl_NetAppSendTokenValue(&Token_value);
#endif
        }
        break;

        case SL_OPCODE_NETAPP_HTTPPOSTTOKENVALUE: {
#ifdef sl_HttpServerCallback
            uint8_t *pPostParams;

            sl_NetAppHttpServerPostToken_t *httpPostTokenArgs = (sl_NetAppHttpServerPostToken_t *)_SL_RESP_ARGS_START(pHdr);
            pPostParams = (uint8_t *)((sl_NetAppHttpServerPostToken_t *)httpPostTokenArgs + 1);

            httpServerEvent.Event = SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT;

            httpServerEvent.EventData.httpPostData.action.len = httpPostTokenArgs->post_action_len;
            httpServerEvent.EventData.httpPostData.action.data = pPostParams;
            pPostParams+=httpPostTokenArgs->post_action_len;

            httpServerEvent.EventData.httpPostData.token_name.len = httpPostTokenArgs->token_name_len;
            httpServerEvent.EventData.httpPostData.token_name.data = pPostParams;
            pPostParams+=httpPostTokenArgs->token_name_len;

            httpServerEvent.EventData.httpPostData.token_value.len = httpPostTokenArgs->token_value_len;
            httpServerEvent.EventData.httpPostData.token_value.data = pPostParams;

            httpServerResponse.Response = SL_NETAPP_RESPONSE_NONE;


            _SlDrvHandleHttpServerEvents (&httpServerEvent, &httpServerResponse);
        }
        break;
#endif        
        default:
            SL_ERROR_TRACE2(MSG_305, "ASSERT: _SlDrvNetAppEventHandler : invalid opcode = 0x%x = %i", pHdr->GenHeader.Opcode, pHdr->GenHeader.Opcode);
            VERIFY_PROTOCOL(0);
    }
}

/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
#if (defined(sl_WlanEvtHdlr))
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    unsigned char  g_ucConnectionSSID[32+1]; //Connection SSID
    unsigned char  g_ucConnectionBSSID[6]; //Connection BSSID
    
    if(pWlanEvent == NULL)
        printf(" [WLAN EVENT] NULL Pointer Error \n\r");

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            _cc3100_->SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            
            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);
            
            printf("[WLAN EVENT] STA Connected to the AP: %s ,"
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            _cc3100_->CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            _cc3100_->CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                printf("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            } else {
                printf("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                        "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT: {
            _cc3100_->SET_STATUS_BIT(g_Status, STATUS_BIT_STA_CONNECTED);
        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT: {
            _cc3100_->CLR_STATUS_BIT(g_Status, STATUS_BIT_STA_CONNECTED);
            _cc3100_->CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_LEASED);
        }
        break;

        default: {
            printf("[WLAN EVENT] Unexpected event [0x%x]\n\r",pWlanEvent->Event);
        }
        break;
    }
}
#endif

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
#if (defined(sl_NetAppEvtHdlr))
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    
    if(pNetAppEvent == NULL){
        printf(" [NETAPP EVENT] NULL Pointer Error \n\r");
    }
    
    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;
            _cc3100_->SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);            
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;           
            g_GatewayIP = pEventData->gateway;
            
            printf("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , ""Gateway=%d.%d.%d.%d\n\r",
            _cc3100_->_netcfg.SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            _cc3100_->_netcfg.SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            _cc3100_->_netcfg.SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            _cc3100_->_netcfg.SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            _cc3100_->_netcfg.SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            _cc3100_->_netcfg.SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            _cc3100_->_netcfg.SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            _cc3100_->_netcfg.SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
            
        }
        break;

        case SL_NETAPP_IP_LEASED_EVENT: {           
            g_StationIP = pNetAppEvent->EventData.ipLeased.ip_address;          
            _cc3100_->SET_STATUS_BIT(g_Status, STATUS_BIT_IP_LEASED);
                       
        }
        break;

        default: {
            printf("[NETAPP EVENT] Unexpected event [0x%x] \n\r",pNetAppEvent->Event);
        }
        break;
    }
}
#endif 

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
#if (defined(sl_SockEvtHdlr))
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        printf(" [SOCK EVENT] NULL Pointer Error \n\r");
    
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            /*
             * TX Failed
             *
             * Information about the socket descriptor and status will be
             * available in 'SlSockEventData_t' - Applications can use it if
             * required
             *
             * SlSockEventData_t *pEventData = NULL;
             * pEventData = & pSock->EventData;
             */
            
            switch( pSock->socketAsyncEvent.SockTxFailData.status )
            {
                case SL_ECLOSE:
                    printf(" [SOCK EVENT] Close socket operation, failed to transmit all queued packets\n\r");
                    break;
                default:
                     printf("[SOCK ERROR] - TX FAILED : socket %d , reason""(%d) \n\n", pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                    break;
            }
            break;

        default:
            printf("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
            break;
    }
}
#endif

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
#if (defined(sl_HttpServerCallback))
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse)
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    printf(" [HTTP EVENT] Unexpected event \n\r");
}
#endif
/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
#if (defined(sl_GeneralEvtHdlr))
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    printf("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n", pDevEvent->EventData.deviceEvent.status, pDevEvent->EventData.deviceEvent.sender);
}
#endif

#ifdef  __cplusplus
}
#endif /*  __cplusplus */

//}//namespace
