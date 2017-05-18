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

#ifndef fPtr_func_h
#define fPtr_func_h

#include "cc3100_simplelink.h"


/* Note. C function pointers point to the functions below. */
/* C++ member function pointers not used ?           */

namespace mbed_cc3100 { 


    extern uint32_t  g_PingPacketsRecv;
    extern uint32_t  g_GatewayIP;
    extern uint32_t  g_StationIP;
    extern uint32_t  g_DestinationIP;
    extern uint32_t  g_BytesReceived; // variable to store the file size 
    extern uint32_t  g_Status;
    extern uint8_t   g_buff[MAX_BUFF_SIZE+1];
    extern int32_t   g_SockID;
    
}//namespace mbed_cc3100    
    
using namespace mbed_cc3100;
    
#ifdef __cplusplus
extern "C" {
#endif     
    

void SimpleLinkPingReport(SlPingReport_t *pPingReport);
   
/*!
    \brief WLAN Async event handler

    \param[out]      pSlWlanEvent   pointer to SlWlanEvent_t data

    \par
             Parameters:

             - <b>pSlWlanEvent->Event = SL_WLAN_CONNECT_EVENT </b>, STA or P2P client connection indication event
                 - pSlWlanEvent->EventData.STAandP2PModeWlanConnected main fields:
                      - ssid_name
                      - ssid_len
                      - bssid
                      - go_peer_device_name
                      - go_peer_device_name_len

             - <b>pSlWlanEvent->Event = SL_WLAN_DISCONNECT_EVENT </b>, STA or P2P client disconnection event
                 - pSlWlanEvent->EventData.STAandP2PModeDisconnected main fields:
                      - ssid_name
                      - ssid_len
                      - reason_code

             - <b>pSlWlanEvent->Event = SL_WLAN_STA_CONNECTED_EVENT </b>, AP/P2P(Go) connected STA/P2P(Client)
                  - pSlWlanEvent->EventData.APModeStaConnected fields:
                      - go_peer_device_name
                      - mac
                      - go_peer_device_name_len
                      - wps_dev_password_id
                      - own_ssid:  relevant for event sta-connected only
                      - own_ssid_len:  relevant for event sta-connected only

             - <b>pSlWlanEvent->Event = SL_WLAN_STA_DISCONNECTED_EVENT </b>, AP/P2P(Go) disconnected STA/P2P(Client)
                  - pSlWlanEvent->EventData.APModestaDisconnected fields:
                      - go_peer_device_name
                      - mac
                      - go_peer_device_name_len
                      - wps_dev_password_id
                      - own_ssid:  relevant for event sta-connected only
                      - own_ssid_len:  relevant for event sta-connected only

             - <b>pSlWlanEvent->Event = SL_WLAN_SMART_CONFIG_COMPLETE_EVENT </b>
                  - pSlWlanEvent->EventData.smartConfigStartResponse fields:
                     - status
                     - ssid_len
                     - ssid
                     - private_token_len
                     - private_token

             - <b>pSlWlanEvent->Event = SL_WLAN_SMART_CONFIG_STOP_EVENT </b>
                     - pSlWlanEvent->EventData.smartConfigStopResponse fields:
                         - status

             - <b>pSlWlanEvent->Event = SL_WLAN_P2P_DEV_FOUND_EVENT </b>
                     - pSlWlanEvent->EventData.P2PModeDevFound fields:
                         - go_peer_device_name
                         - mac
                         - go_peer_device_name_len
                         - wps_dev_password_id
                         - own_ssid:  relevant for event sta-connected only
                         - own_ssid_len:  relevant for event sta-connected only

             - <b>pSlWlanEvent->Event = SL_WLAN_P2P_NEG_REQ_RECEIVED_EVENT </b>
                      - pSlWlanEvent->EventData.P2PModeNegReqReceived fields
                          - go_peer_device_name
                          - mac
                          - go_peer_device_name_len
                          - wps_dev_password_id
                          - own_ssid:  relevant for event sta-connected only

             - <b>pSlWlanEvent->Event = SL_WLAN_CONNECTION_FAILED_EVENT </b>, P2P only
                       - pSlWlanEvent->EventData.P2PModewlanConnectionFailure fields:
                           - status
*/
#if (defined(sl_WlanEvtHdlr))
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent);
#endif

/*!
    \brief NETAPP Async event handler

    \param[out]      pSlNetApp   pointer to SlNetAppEvent_t data

    \par
             Parameters:
              - <b>pSlWlanEvent->Event = SL_NETAPP_IPV4_IPACQUIRED_EVENT</b>, IPV4 acquired event
                  - pSlWlanEvent->EventData.ipAcquiredV4 fields:
                       - ip
                       - gateway
                       - dns

              - <b>pSlWlanEvent->Event = SL_NETAPP_IP_LEASED_EVENT</b>, AP or P2P go dhcp lease event
                  - pSlWlanEvent->EventData.ipLeased  fields:
                       - ip_address
                       - lease_time
                       - mac

              - <b>pSlWlanEvent->Event = SL_NETAPP_IP_RELEASED_EVENT</b>, AP or P2P go dhcp ip release event
                   - pSlWlanEvent->EventData.ipReleased fields
                       - ip_address
                       - mac
                       - reason

*/
#if (defined(sl_NetAppEvtHdlr))
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent);
#endif

/*!
    \brief Socket Async event handler

    \param[out]      pSlSockEvent   pointer to SlSockEvent_t data

    \par
             Parameters:\n
             - <b>pSlSockEvent->Event = SL_SOCKET_TX_FAILED_EVENT</b>
                 - pSlSockEvent->EventData fields:
                     - sd
                     - status
             - <b>pSlSockEvent->Event = SL_SOCKET_ASYNC_EVENT</b>
                - pSlSockEvent->EventData fields:
                     - sd
                     - type: SSL_ACCEPT  or RX_FRAGMENTATION_TOO_BIG or OTHER_SIDE_CLOSE_SSL_DATA_NOT_ENCRYPTED
                     - val

*/
#if (defined(sl_SockEvtHdlr))
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock);
#endif

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent);

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse);
#ifndef SL_TINY_EXT
void _sl_HandleAsync_Accept(void *pVoidBuf);

void _sl_HandleAsync_Select(void *pVoidBuf);

void _sl_HandleAsync_DnsGetHostByService(void *pVoidBuf);

void _sl_HandleAsync_PingResponse(void *pVoidBuf);
#endif
void _sl_HandleAsync_Connect(void *pVoidBuf);

void _sl_HandleAsync_DnsGetHostByName(void *pVoidBuf);

void _sl_HandleAsync_Stop(void *pVoidBuf);

_SlReturnVal_t _SlDrvMsgReadSpawnCtx(void *pValue);

void _SlDrvDeviceEventHandler(void *pArgs);

void _SlDrvNetAppEventHandler(void *pArgs);

void _SlDrvHandleHttpServerEvents(SlHttpServerEvent_t *slHttpServerEvent, SlHttpServerResponse_t *slHttpServerResponse);
void _SlDrvHandleSockEvents(SlSockEvent_t *slSockEvent);
void _SlDrvHandleNetAppEvents(SlNetAppEvent_t *slNetAppEvent);
void _SlDrvHandleWlanEvents(SlWlanEvent_t *slWlanEvent);
void _SlDrvHandleGeneralEvents(SlDeviceEvent_t *slGeneralEvent);

#ifdef  __cplusplus
}
#endif /*  __cplusplus */

//}//namespace mbed_cc3100

#endif//fPtr_func_h







