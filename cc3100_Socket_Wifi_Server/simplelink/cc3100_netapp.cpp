/*
 * netapp.c - CC31xx/CC32xx Host Driver Implementation
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



/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "cc3100_simplelink.h"
#include "cc3100_protocol.h"
#include "cc3100_driver.h"

#include "cc3100_netapp.h"
#include "fPtr_func.h"

namespace mbed_cc3100 {

/*****************************************************************************/
/* Macro declarations                                                        */
/*****************************************************************************/
const uint32_t NETAPP_MDNS_OPTIONS_ADD_SERVICE_BIT			 =		 ((uint32_t)0x1 << 31);

#ifdef SL_TINY
const uint8_t NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH   =      63;
#else
const uint8_t NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH   =      255;
#endif
	
cc3100_netapp::cc3100_netapp(cc3100_driver &driver, cc3100_nonos &nonos)
    : _driver(driver), _nonos(nonos)
{

}

cc3100_netapp::~cc3100_netapp()
{

}


/*****************************************************************************/
/* API functions                                                             */
/*****************************************************************************/

/*****************************************************************************
 sl_NetAppStart
*****************************************************************************/
typedef union {
    _NetAppStartStopCommand_t       Cmd;
    _NetAppStartStopResponse_t   Rsp;
} _SlNetAppStartStopMsg_u;

#if _SL_INCLUDE_FUNC(sl_NetAppStart)
const _SlCmdCtrl_t _SlNetAppStartCtrl = {
    SL_OPCODE_NETAPP_START_COMMAND,
    sizeof(_NetAppStartStopCommand_t),
    sizeof(_NetAppStartStopResponse_t)
};

int16_t cc3100_netapp::sl_NetAppStart(const uint32_t AppBitMap)
{
    _SlNetAppStartStopMsg_u Msg;
    Msg.Cmd.appId = AppBitMap;
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlNetAppStartCtrl, &Msg, NULL));

    return Msg.Rsp.status;
}
#endif

/*****************************************************************************
 sl_NetAppStop
*****************************************************************************/
#if _SL_INCLUDE_FUNC(sl_NetAppStop)
const _SlCmdCtrl_t _SlNetAppStopCtrl =
{
    SL_OPCODE_NETAPP_STOP_COMMAND,
    sizeof(_NetAppStartStopCommand_t),
    sizeof(_NetAppStartStopResponse_t)
};
int16_t cc3100_netapp::sl_NetAppStop(const uint32_t AppBitMap)
{
    _SlNetAppStartStopMsg_u Msg;
    Msg.Cmd.appId = AppBitMap;
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlNetAppStopCtrl, &Msg, NULL));

    return Msg.Rsp.status;
}
#endif


/******************************************************************************/
/* sl_NetAppGetServiceList */
/******************************************************************************/
typedef struct {
    uint8_t  IndexOffest;
    uint8_t  MaxServiceCount;
    uint8_t  Flags;
    int8_t  Padding;
} NetappGetServiceListCMD_t;

typedef union {
    NetappGetServiceListCMD_t      Cmd;
    _BasicResponse_t                Rsp;
} _SlNetappGetServiceListMsg_u;

#if _SL_INCLUDE_FUNC(sl_NetAppGetServiceList)
const _SlCmdCtrl_t _SlGetServiceListeCtrl = {
    SL_OPCODE_NETAPP_NETAPP_MDNS_LOOKUP_SERVICE,
    sizeof(NetappGetServiceListCMD_t),
    sizeof(_BasicResponse_t)
};

int16_t cc3100_netapp::sl_NetAppGetServiceList(const uint8_t  IndexOffest,
        const uint8_t  MaxServiceCount,
        const uint8_t  Flags,
        int8_t  *pBuffer,
        const uint32_t  RxBufferLength
                                              )
{
    int32_t 					 retVal= 0;
    _SlNetappGetServiceListMsg_u Msg;
    _SlCmdExt_t                  CmdExt;
    uint16_t               ServiceSize = 0;
    uint16_t               BufferSize = 0;

    /*
    Calculate RX pBuffer size
    WARNING:
    if this size is BufferSize than 1480 error should be returned because there
    is no place in the RX packet.
    */
    switch(Flags) {
        case SL_NET_APP_FULL_SERVICE_WITH_TEXT_IPV4_TYPE:
            ServiceSize =  sizeof(SlNetAppGetFullServiceWithTextIpv4List_t);
            break;

        case SL_NET_APP_FULL_SERVICE_IPV4_TYPE:
            ServiceSize =  sizeof(SlNetAppGetFullServiceIpv4List_t);
            break;

        case SL_NET_APP_SHORT_SERVICE_IPV4_TYPE:
            ServiceSize =  sizeof(SlNetAppGetShortServiceIpv4List_t);
            break;

        default:
            ServiceSize =  sizeof(_BasicResponse_t);
            break;
    }



    BufferSize =  MaxServiceCount * ServiceSize;

    /*Check the size of the requested services is smaller than size of the user buffer.
      If not an error is returned in order to avoid overwriting memory. */
    if(RxBufferLength <= BufferSize) {
        return SL_ERROR_NETAPP_RX_BUFFER_LENGTH_ERROR;
    }

    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.RxPayloadLen = BufferSize;

    CmdExt.pRxPayload = (uint8_t *)pBuffer;

    Msg.Cmd.IndexOffest		= IndexOffest;
    Msg.Cmd.MaxServiceCount = MaxServiceCount;
    Msg.Cmd.Flags			= Flags;
    Msg.Cmd.Padding			= 0;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlGetServiceListeCtrl, &Msg, &CmdExt));
    retVal = Msg.Rsp.status;

    return (int16_t)retVal;
}

#endif

/*****************************************************************************/
/* sl_mDNSRegisterService */
/*****************************************************************************/
/*
 * The below struct depicts the constant parameters of the command/API RegisterService.
 *
   1. ServiceLen                      - The length of the service should be smaller than NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH.
   2. TextLen                         - The length of the text should be smaller than NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH.
   3. port                            - The port on this target host.
   4. TTL                             - The TTL of the service
   5. Options                         - bitwise parameters:
                                        bit 0  - is unique (means if the service needs to be unique)
										bit 31  - for internal use if the service should be added or deleted (set means ADD).
                                        bit 1-30 for future.

   NOTE:

   1. There are another variable parameter is this API which is the service name and the text.
   2. According to now there is no warning and Async event to user on if the service is a unique.
*
 */

typedef struct {
    uint8_t   ServiceNameLen;
    uint8_t   TextLen;
    uint16_t  Port;
    uint32_t   TTL;
    uint32_t   Options;
} NetappMdnsSetService_t;

typedef union {
    NetappMdnsSetService_t         Cmd;
    _BasicResponse_t                Rsp;
} _SlNetappMdnsRegisterServiceMsg_u;

#if _SL_INCLUDE_FUNC(sl_NetAppMDNSRegisterUnregisterService)
const _SlCmdCtrl_t _SlRegisterServiceCtrl = {
    SL_OPCODE_NETAPP_MDNSREGISTERSERVICE,
    sizeof(NetappMdnsSetService_t),
    sizeof(_BasicResponse_t)
};

/******************************************************************************

    sl_NetAppMDNSRegisterService

    CALLER          user from its host


    DESCRIPTION:
                    Add/delete  service
					The function manipulates the command that register the service and call
					to the NWP in order to add/delete the service to/from the mDNS package and to/from the DB.

					This register service is a service offered by the application.
					This unregister service is a service offered by the application before.

					The service name should be full service name according to RFC
                    of the DNS-SD - means the value in name field in SRV answer.

					Example for service name:
                    1. PC1._ipp._tcp.local
                    2. PC2_server._ftp._tcp.local

                    If the option is_unique is set, mDNS probes the service name to make sure
                    it is unique before starting to announce the service on the network.
                    Instance is the instance portion of the service name.




    PARAMETERS:

                    The command is from constant parameters and variables parameters.

					Constant parameters are:

                    ServiceLen                          - The length of the service.
                    TextLen                             - The length of the service should be smaller than 64.
                    port                                - The port on this target host.
                    TTL                                 - The TTL of the service
                    Options                             - bitwise parameters:
                                                            bit 0  - is unique (means if the service needs to be unique)
                                                            bit 31  - for internal use if the service should be added or deleted (set means ADD).
                                                            bit 1-30 for future.

                   The variables parameters are:

                    Service name(full service name)     - The service name.
                                                          Example for service name:
                                                          1. PC1._ipp._tcp.local
                                                          2. PC2_server._ftp._tcp.local

                    Text                                - The description of the service.
                                                          should be as mentioned in the RFC
                                                          (according to type of the service IPP,FTP...)

					NOTE - pay attention

						1. Temporary -  there is an allocation on stack of internal buffer.
						Its size is NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH.
						It means that the sum of the text length and service name length cannot be bigger than
						NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH.
						If it is - An error is returned.

					    2. According to now from certain constraints the variables parameters are set in the
					    attribute part (contain constant parameters)



	  RETURNS:        Status - the immediate response of the command status.
							   0 means success.



******************************************************************************/
int16_t cc3100_netapp::sl_NetAppMDNSRegisterUnregisterService(const char* pServiceName, 
                                                              const uint8_t ServiceNameLen, 
                                                              const char* pText, 
                                                              const uint8_t TextLen, 
                                                              const uint16_t Port, 
                                                              const uint32_t TTL, 
                                                              const uint32_t Options)
{

    _SlNetappMdnsRegisterServiceMsg_u Msg;
    _SlCmdExt_t	CmdExt ;
    unsigned char ServiceNameAndTextBuffer[NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH];
    unsigned char *TextPtr;

    /*

    NOTE - pay attention

    	1. Temporary -  there is an allocation on stack of internal buffer.
    	Its size is NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH.
    	It means that the sum of the text length and service name length cannot be bigger than
    	NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH.
    	If it is - An error is returned.

    	2. According to now from certain constraints the variables parameters are set in the
    	attribute part (contain constant parameters)


    */

    /*build the attribute part of the command.
      It contains the constant parameters of the command*/

    Msg.Cmd.ServiceNameLen	= ServiceNameLen;
    Msg.Cmd.Options			= Options;
    Msg.Cmd.Port			= Port;
    Msg.Cmd.TextLen			= TextLen;
    Msg.Cmd.TTL				= TTL;

    /*Build the payload part of the command
     Copy the service name and text to one buffer.
     NOTE - pay attention
     			The size of the service length + the text length should be smaller than 255,
     			Until the simplelink drive supports to variable length through SPI command. */
    if(TextLen + ServiceNameLen > (NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH - 1 )) { /*-1 is for giving a place to set null termination at the end of the text*/
        return -1;
    }

    _driver._SlDrvMemZero(ServiceNameAndTextBuffer, NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH);


    /*Copy the service name*/
    memcpy(ServiceNameAndTextBuffer,
              pServiceName,
              ServiceNameLen);

    if(TextLen > 0 ) {

        TextPtr = &ServiceNameAndTextBuffer[ServiceNameLen];
        /*Copy the text just after the service name*/
        memcpy(TextPtr, pText, TextLen);
    }

    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.TxPayloadLen = (TextLen + ServiceNameLen);
    CmdExt.pTxPayload   = (uint8_t *)ServiceNameAndTextBuffer;


    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlRegisterServiceCtrl, &Msg, &CmdExt));

    return (int16_t)Msg.Rsp.status;


}
#endif

/**********************************************************************************************/
#if _SL_INCLUDE_FUNC(sl_NetAppMDNSRegisterService)
int16_t cc3100_netapp::sl_NetAppMDNSRegisterService(const char* pServiceName, 
                                                    const uint8_t ServiceNameLen, 
                                                    const char* pText, 
                                                    const uint8_t TextLen, 
                                                    const uint16_t Port, 
                                                    const uint32_t TTL, 
                                                    uint32_t Options)
{

    /*

    NOTE - pay attention

    1. Temporary -  there is an allocation on stack of internal buffer.
    Its size is NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH.
    It means that the sum of the text length and service name length cannot be bigger than
    NETAPP_MDNS_MAX_SERVICE_NAME_AND_TEXT_LENGTH.
    If it is - An error is returned.

    2. According to now from certain constraints the variables parameters are set in the
    attribute part (contain constant parameters)

    */

    /*Set the add service bit in the options parameter.
      In order not use different opcodes for the register service and unregister service
      bit 31 in option is taken for this purpose. if it is set it means in NWP that the service should be added
      if it is cleared it means that the service should be deleted and there is only meaning to pServiceName
      and ServiceNameLen values. */
    Options |=  NETAPP_MDNS_OPTIONS_ADD_SERVICE_BIT;

    return  (sl_NetAppMDNSRegisterUnregisterService(	pServiceName, ServiceNameLen, pText, TextLen, Port, TTL, Options));


}
#endif
/**********************************************************************************************/



/**********************************************************************************************/
#if _SL_INCLUDE_FUNC(sl_NetAppMDNSUnRegisterService)
int16_t cc3100_netapp::sl_NetAppMDNSUnRegisterService(const char* pServiceName, const uint8_t ServiceNameLen)
{
    uint32_t    Options = 0;

    /*

    NOTE - pay attention

    		The size of the service length  should be smaller than 255,
    		Until the simplelink drive supports to variable length through SPI command.


    */

    /*Clear the add service bit in the options parameter.
      In order not use different opcodes for the register service and unregister service
      bit 31 in option is taken for this purpose. if it is set it means in NWP that the service should be added
      if it is cleared it means that the service should be deleted and there is only meaning to pServiceName
      and ServiceNameLen values.*/

    Options &=  (~NETAPP_MDNS_OPTIONS_ADD_SERVICE_BIT);

    return  (sl_NetAppMDNSRegisterUnregisterService(pServiceName, ServiceNameLen, NULL, 0, 0, 0, Options));


}
#endif
/**********************************************************************************************/



/*****************************************************************************/
/* sl_DnsGetHostByService */
/*****************************************************************************/
/*
 * The below struct depicts the constant parameters of the command/API sl_DnsGetHostByService.
 *
   1. ServiceLen                      - The length of the service should be smaller than 255.
   2. AddrLen                         - TIPv4 or IPv6 (SL_AF_INET , SL_AF_INET6).
*
 */

typedef struct {
    uint8_t   ServiceLen;
    uint8_t   AddrLen;
    uint16_t  Padding;
} _GetHostByServiceCommand_t;



/*
 * The below structure depict the constant parameters that are returned in the Async event answer
 * according to command/API sl_DnsGetHostByService for IPv4 and IPv6.
 *
	1Status						- The status of the response.
	2.Address						- Contains the IP address of the service.
	3.Port							- Contains the port of the service.
	4.TextLen						- Contains the max length of the text that the user wants to get.
												it means that if the test of service is bigger that its value than
												the text is cut to inout_TextLen value.
										Output: Contain the length of the text that is returned. Can be full text or part
												of the text (see above).

*
 
typedef struct {
    uint16_t   Status;
    uint16_t   TextLen;
    uint32_t    Port;
    uint32_t    Address;
} _GetHostByServiceIPv4AsyncResponse_t;
*/

typedef struct {
    uint16_t   Status;
    uint16_t   TextLen;
    uint32_t    Port;
    uint32_t    Address[4];
} _GetHostByServiceIPv6AsyncResponse_t;


typedef union {
    _GetHostByServiceIPv4AsyncResponse_t IpV4;
    _GetHostByServiceIPv6AsyncResponse_t IpV6;
} _GetHostByServiceAsyncResponseAttribute_u;

typedef union {
    _GetHostByServiceCommand_t      Cmd;
    _BasicResponse_t                Rsp;
} _SlGetHostByServiceMsg_u;

#if _SL_INCLUDE_FUNC(sl_NetAppDnsGetHostByService)
const _SlCmdCtrl_t _SlGetHostByServiceCtrl = {
    SL_OPCODE_NETAPP_MDNSGETHOSTBYSERVICE,
    sizeof(_GetHostByServiceCommand_t),
    sizeof(_BasicResponse_t)
};

int32_t cc3100_netapp::sl_NetAppDnsGetHostByService(unsigned char *pServiceName,	/* string containing all (or only part): name + subtype + service */
        const uint8_t ServiceLen,
        const uint8_t Family,			/* 4-IPv4 , 16-IPv6 */
        uint32_t pAddr[],
        uint32_t *pPort,
        uint16_t *pTextLen, /* in: max len , out: actual len */
        unsigned char *pText)
{

    _SlGetHostByServiceMsg_u         Msg;
    _SlCmdExt_t                      CmdExt ;
    _GetHostByServiceAsyncResponse_t AsyncRsp;
    uint8_t ObjIdx = MAX_CONCURRENT_ACTIONS;

    /*
    	Note:
    	1. The return's attributes are belonged to first service that is found.
    	It can be other services with the same service name will response to
    	the query. The results of these responses are saved in the peer cache of the NWP, and
    	should be read by another API.

    	2. Text length can be 120 bytes only - not more
    	It is because of constraints in the NWP on the buffer that is allocated for the Async event.

    	3.The API waits to Async event by blocking. It means that the API is finished only after an Async event
    	is sent by the NWP.

    	4.No rolling option!!! - only PTR type is sent.


    */
    /*build the attribute part of the command.
      It contains the constant parameters of the command */

    Msg.Cmd.ServiceLen = ServiceLen;
    Msg.Cmd.AddrLen    = Family;

    /*Build the payload part of the command
      Copy the service name and text to one buffer.*/
    _driver._SlDrvResetCmdExt(&CmdExt);  
    CmdExt.TxPayloadLen = ServiceLen;
    
    CmdExt.pTxPayload   = (uint8_t *)pServiceName;
    

    /*set pointers to the output parameters (the returned parameters).
      This pointers are belonged to local struct that is set to global Async response parameter.
      It is done in order not to run more than one sl_DnsGetHostByService at the same time.
      The API should be run only if global parameter is pointed to NULL. */
    AsyncRsp.out_pText     = pText;
    AsyncRsp.inout_TextLen = (uint16_t* )pTextLen;
    AsyncRsp.out_pPort     = pPort;
    AsyncRsp.out_pAddr     = (uint32_t *)pAddr;


    ObjIdx = _driver._SlDrvProtectAsyncRespSetting((uint8_t*)&AsyncRsp, GETHOSYBYSERVICE_ID, SL_MAX_SOCKETS);

    if (MAX_CONCURRENT_ACTIONS == ObjIdx)
    {
        return SL_POOL_IS_EMPTY;
    }

    if (SL_AF_INET6 == Family) {
        g_pCB->ObjPool[ObjIdx].AdditionalData |= SL_NETAPP_FAMILY_MASK;
    }
    /* Send the command */
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlGetHostByServiceCtrl, &Msg, &CmdExt));



    /* If the immediate reponse is O.K. than  wait for aSYNC event response. */
    if(SL_RET_CODE_OK == Msg.Rsp.status) {
        _driver._SlDrvSyncObjWaitForever(&g_pCB->ObjPool[ObjIdx].SyncObj);

        /* If we are - it means that Async event was sent.
           The results are copied in the Async handle return functions */

        Msg.Rsp.status = AsyncRsp.Status;
    }

    _driver._SlDrvReleasePoolObj(ObjIdx);
    return Msg.Rsp.status;
}
#endif

/*****************************************************************************/
/*  _sl_HandleAsync_DnsGetHostByAddr */
/*****************************************************************************/
#ifndef SL_TINY_EXT
void cc3100_netapp::_sl_HandleAsync_DnsGetHostByAddr(void *pVoidBuf)
{
    SL_TRACE0(DBG_MSG, MSG_303, "STUB: _sl_HandleAsync_DnsGetHostByAddr not implemented yet!");
    return;
}
#endif
/*****************************************************************************/
/* sl_DnsGetHostByName */
/*****************************************************************************/
typedef union {
    _GetHostByNameIPv4AsyncResponse_t IpV4;
    _GetHostByNameIPv6AsyncResponse_t IpV6;
} _GetHostByNameAsyncResponse_u;

typedef union {
    _GetHostByNameCommand_t         Cmd;
    _BasicResponse_t                Rsp;
} _SlGetHostByNameMsg_u;

#if _SL_INCLUDE_FUNC(sl_NetAppDnsGetHostByName)
const _SlCmdCtrl_t _SlGetHostByNameCtrl = {
    SL_OPCODE_NETAPP_DNSGETHOSTBYNAME,
    sizeof(_GetHostByNameCommand_t),
    sizeof(_BasicResponse_t)
};

int16_t cc3100_netapp::sl_NetAppDnsGetHostByName(unsigned char * hostname, const uint16_t usNameLen, uint32_t*  out_ip_addr, const uint8_t family)
{
    _SlGetHostByNameMsg_u           Msg;
    _SlCmdExt_t                     ExtCtrl;
    _GetHostByNameAsyncResponse_u   AsyncRsp;
    uint8_t ObjIdx = MAX_CONCURRENT_ACTIONS;
    
    _driver._SlDrvResetCmdExt(&ExtCtrl);
    ExtCtrl.TxPayloadLen = usNameLen;
    
    ExtCtrl.pTxPayload = (unsigned char *)hostname;
    

    Msg.Cmd.Len = usNameLen;
    Msg.Cmd.family = family;

    /*Use Obj to issue the command, if not available try later */
    ObjIdx = (uint8_t)_driver._SlDrvWaitForPoolObj(GETHOSYBYNAME_ID,SL_MAX_SOCKETS);
    if (MAX_CONCURRENT_ACTIONS == ObjIdx) {
        printf("SL_POOL_IS_EMPTY \r\n");
        return SL_POOL_IS_EMPTY;
    }
    
    _driver._SlDrvProtectionObjLockWaitForever();
    
    g_pCB->ObjPool[ObjIdx].pRespArgs =  (uint8_t *)&AsyncRsp;
    /*set bit to indicate IPv6 address is expected */
    if (SL_AF_INET6 == family) {
        g_pCB->ObjPool[ObjIdx].AdditionalData |= SL_NETAPP_FAMILY_MASK;
    }

     _driver._SlDrvProtectionObjUnLock();
    
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlGetHostByNameCtrl, &Msg, &ExtCtrl));

    if(SL_RET_CODE_OK == Msg.Rsp.status) {
        _driver._SlDrvSyncObjWaitForever(&g_pCB->ObjPool[ObjIdx].SyncObj);
        Msg.Rsp.status = AsyncRsp.IpV4.status;

        if(SL_OS_RET_CODE_OK == (int16_t)Msg.Rsp.status) {
            memcpy((int8_t *)out_ip_addr, (signed char *)&AsyncRsp.IpV4.ip0, (SL_AF_INET == family) ? SL_IPV4_ADDRESS_SIZE : SL_IPV6_ADDRESS_SIZE);
        }
    }
    _driver._SlDrvReleasePoolObj(ObjIdx);
    return Msg.Rsp.status;
}
#endif

#ifndef SL_TINY_EXT
void cc3100_netapp::CopyPingResultsToReport(_PingReportResponse_t *pResults,SlPingReport_t *pReport)
{
    pReport->PacketsSent     = pResults->numSendsPings;
    pReport->PacketsReceived = pResults->numSuccsessPings;
    pReport->MinRoundTime    = pResults->rttMin;
    pReport->MaxRoundTime    = pResults->rttMax;
    pReport->AvgRoundTime    = pResults->rttAvg;
    pReport->TestTime        = pResults->testTime;
}
#endif
/*****************************************************************************/
/* sl_PingStart */
/*****************************************************************************/
typedef union {
    _PingStartCommand_t   Cmd;
    _PingReportResponse_t  Rsp;
} _SlPingStartMsg_u;


typedef enum {
    CMD_PING_TEST_RUNNING = 0,
    CMD_PING_TEST_STOPPED
} _SlPingStatus_e;

P_SL_DEV_PING_CALLBACK  pPingCallBackFunc;

#if _SL_INCLUDE_FUNC(sl_NetAppPingStart)
int16_t cc3100_netapp::sl_NetAppPingStart(const SlPingStartCommand_t* pPingParams, const uint8_t family, SlPingReport_t *pReport, const P_SL_DEV_PING_CALLBACK pPingCallback)
{

    _SlCmdCtrl_t                CmdCtrl = {0, sizeof(_PingStartCommand_t), sizeof(_BasicResponse_t)};
    _SlPingStartMsg_u           Msg;
    _PingReportResponse_t       PingRsp;
    uint8_t ObjIdx = MAX_CONCURRENT_ACTIONS;

    if( 0 == pPingParams->Ip ) 
    { /* stop any ongoing ping */
        return _driver._SlDrvBasicCmd(SL_OPCODE_NETAPP_PINGSTOP);
    }

    if(SL_AF_INET == family) {
        CmdCtrl.Opcode = SL_OPCODE_NETAPP_PINGSTART;
        memcpy(&Msg.Cmd.ip0, &pPingParams->Ip, SL_IPV4_ADDRESS_SIZE);
    } else {
        CmdCtrl.Opcode = SL_OPCODE_NETAPP_PINGSTART_V6;
        memcpy(&Msg.Cmd.ip0, &pPingParams->Ip, SL_IPV6_ADDRESS_SIZE);
    }

    Msg.Cmd.pingIntervalTime        = pPingParams->PingIntervalTime;
    Msg.Cmd.PingSize                = pPingParams->PingSize;
    Msg.Cmd.pingRequestTimeout      = pPingParams->PingRequestTimeout;
    Msg.Cmd.totalNumberOfAttempts   = pPingParams->TotalNumberOfAttempts;
    Msg.Cmd.flags                   = pPingParams->Flags;

    if( pPingCallback ) {
        pPingCallBackFunc = pPingCallback;
    } else {
        /*Use Obj to issue the command, if not available try later */
        ObjIdx = (uint8_t)_driver._SlDrvWaitForPoolObj(PING_ID,SL_MAX_SOCKETS);
        if (MAX_CONCURRENT_ACTIONS == ObjIdx) {
            return SL_POOL_IS_EMPTY;
        }
        OSI_RET_OK_CHECK(_nonos.sl_LockObjLock(&g_pCB->ProtectionLockObj, NON_OS_LOCK_OBJ_UNLOCK_VALUE, NON_OS_LOCK_OBJ_LOCK_VALUE, SL_OS_WAIT_FOREVER));
        /* async response handler for non callback mode */
        g_pCB->ObjPool[ObjIdx].pRespArgs = (uint8_t *)&PingRsp;
        pPingCallBackFunc = NULL;
        OSI_RET_OK_CHECK(_nonos.sl_LockObjUnlock(&g_pCB->ProtectionLockObj, NON_OS_LOCK_OBJ_UNLOCK_VALUE));
    }


    VERIFY_RET_OK(_driver._SlDrvCmdOp(&CmdCtrl, &Msg, NULL));
    /*send the command*/
    if(CMD_PING_TEST_RUNNING == (int16_t)Msg.Rsp.status || CMD_PING_TEST_STOPPED == (int16_t)Msg.Rsp.status ) {
        /* block waiting for results if no callback function is used */
        if( NULL == pPingCallback ) {
            _driver._SlDrvSyncObjWaitForever(&g_pCB->ObjPool[ObjIdx].SyncObj);
            if( SL_OS_RET_CODE_OK == (int16_t)PingRsp.status ) {
                CopyPingResultsToReport(&PingRsp,pReport);
            }
            _driver._SlDrvReleasePoolObj(ObjIdx);
        }
    } else {
        /* ping failure, no async response */
        if( NULL == pPingCallback ) {
            _driver._SlDrvReleasePoolObj(ObjIdx);
        }
    }

    return Msg.Rsp.status;
}
#endif

/*****************************************************************************/
/* sl_NetAppSet */
/*****************************************************************************/
typedef union {
    _NetAppSetGet_t    Cmd;
    _BasicResponse_t   Rsp;
} _SlNetAppMsgSet_u;

#if _SL_INCLUDE_FUNC(sl_NetAppSet)
const _SlCmdCtrl_t _SlNetAppSetCmdCtrl = {
    SL_OPCODE_NETAPP_NETAPPSET,
    sizeof(_NetAppSetGet_t),
    sizeof(_BasicResponse_t)
};

int32_t cc3100_netapp::sl_NetAppSet(const uint8_t AppId ,const uint8_t Option, const uint8_t OptionLen, const uint8_t *pOptionValue)
{

    _SlNetAppMsgSet_u         Msg;
    _SlCmdExt_t               CmdExt;
    
    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.TxPayloadLen = (OptionLen+3) & (~3);
    
    CmdExt.pTxPayload = (uint8_t *)pOptionValue;
    
    Msg.Cmd.AppId    = AppId;
    Msg.Cmd.ConfigLen   = OptionLen;
    Msg.Cmd.ConfigOpt   = Option;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlNetAppSetCmdCtrl, &Msg, &CmdExt));

    return (int16_t)Msg.Rsp.status;
}
#endif

/*****************************************************************************/
/* sl_NetAppSendTokenValue */
/*****************************************************************************/
typedef union {
    sl_NetAppHttpServerSendToken_t    Cmd;
    _BasicResponse_t   Rsp;
} _SlNetAppMsgSendTokenValue_u;

#if defined(sl_HttpServerCallback) || defined(EXT_LIB_REGISTERED_HTTP_SERVER_EVENTS)
const _SlCmdCtrl_t _SlNetAppSendTokenValueCmdCtrl = {
    SL_OPCODE_NETAPP_HTTPSENDTOKENVALUE,
    sizeof(sl_NetAppHttpServerSendToken_t),
    sizeof(_BasicResponse_t)
};

uint16_t cc3100_netapp::sl_NetAppSendTokenValue(slHttpServerData_t * Token_value)
{

    _SlNetAppMsgSendTokenValue_u    Msg;
    _SlCmdExt_t						CmdExt;

    CmdExt.TxPayloadLen = (Token_value->value_len+3) & (~3);
    CmdExt.RxPayloadLen = 0;
    CmdExt.pTxPayload = (uint8_t *) Token_value->token_value;
    CmdExt.pRxPayload = NULL;

    Msg.Cmd.token_value_len = Token_value->value_len;
    Msg.Cmd.token_name_len = Token_value->name_len;
    memcpy(&Msg.Cmd.token_name[0], Token_value->token_name, Token_value->name_len);


    VERIFY_RET_OK(_driver._SlDrvCmdSend((_SlCmdCtrl_t *)&_SlNetAppSendTokenValueCmdCtrl, &Msg, &CmdExt));

    return Msg.Rsp.status;
}
#endif

/*****************************************************************************/
/* sl_NetAppGet */
/*****************************************************************************/
typedef union {
    _NetAppSetGet_t	    Cmd;
    _NetAppSetGet_t	    Rsp;
} _SlNetAppMsgGet_u;

#if _SL_INCLUDE_FUNC(sl_NetAppGet)
const _SlCmdCtrl_t _SlNetAppGetCmdCtrl = {
    SL_OPCODE_NETAPP_NETAPPGET,
    sizeof(_NetAppSetGet_t),
    sizeof(_NetAppSetGet_t)
};

int32_t cc3100_netapp::sl_NetAppGet(const uint8_t AppId, const uint8_t Option, uint8_t *pOptionLen, uint8_t *pOptionValue)
{
    _SlNetAppMsgGet_u         Msg;
    _SlCmdExt_t               CmdExt;

    if (*pOptionLen == 0) {
        return SL_EZEROLEN;
    }
    
    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.RxPayloadLen = *pOptionLen;
    
    CmdExt.pRxPayload = (uint8_t *)pOptionValue;

    Msg.Cmd.AppId    = AppId;
    Msg.Cmd.ConfigOpt   = Option;
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlNetAppGetCmdCtrl, &Msg, &CmdExt));


    if (CmdExt.RxPayloadLen < CmdExt.ActualRxPayloadLen) {
        *pOptionLen = (uint8_t)CmdExt.RxPayloadLen;
        return SL_ESMALLBUF;
    } else {
        *pOptionLen = (uint8_t)CmdExt.ActualRxPayloadLen;
    }

    return (int16_t)Msg.Rsp.Status;
}
#endif

cc3100_flowcont::cc3100_flowcont(cc3100_driver &driver, cc3100_nonos &nonos)
    : _driver(driver), _nonos(nonos)
{

}

cc3100_flowcont::~cc3100_flowcont()
{

}
#if 0
/*****************************************************************************/
/* _SlDrvFlowContInit */
/*****************************************************************************/
void cc3100_flowcont::_SlDrvFlowContInit(void)
{
    g_pCB->FlowContCB.TxPoolCnt = FLOW_CONT_MIN;

    OSI_RET_OK_CHECK(_nonos.sl_LockObjCreate(&g_pCB->FlowContCB.TxLockObj, "TxLockObj"));

    OSI_RET_OK_CHECK(_nonos.sl_SyncObjCreate(&g_pCB->FlowContCB.TxSyncObj, "TxSyncObj"));
}

/*****************************************************************************/
/* _SlDrvFlowContDeinit */
/*****************************************************************************/
void cc3100_flowcont::_SlDrvFlowContDeinit(void)
{
    g_pCB->FlowContCB.TxPoolCnt = 0;

    OSI_RET_OK_CHECK(_nonos.sl_LockObjDelete(&g_pCB->FlowContCB.TxLockObj, 0));

    OSI_RET_OK_CHECK(_nonos.sl_SyncObjDelete(&g_pCB->FlowContCB.TxSyncObj, 0));
}
#endif
}//namespace mbed_cc3100


