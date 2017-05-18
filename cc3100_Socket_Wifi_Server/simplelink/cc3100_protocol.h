/*
 * protocol.h - CC31xx/CC32xx Host Driver Implementation
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

/*******************************************************************************\
*
*   FILE NAME:      protocol.h
*
*   DESCRIPTION:    Constant and data structure definitions and function
*                   prototypes for the SL protocol module, which implements
*                   processing of SimpleLink Commands.
*
*   AUTHOR:
*
\*******************************************************************************/

#ifndef SL_PROTOCOL_TYPES_H_
#define SL_PROTOCOL_TYPES_H_

namespace mbed_cc3100 {

/****************************************************************************
**
**  User I/F pools definitions
**
****************************************************************************/

/****************************************************************************
**
**  Definitions for SimpleLink Commands
**
****************************************************************************/


/* pattern for LE 8/16/32 or BE*/
#define H2N_SYNC_PATTERN     {0xBBDDEEFF,0x4321,0x34,0x12}
#define H2N_CNYS_PATTERN     {0xBBDDEEFF,0x8765,0x78,0x56}

const uint32_t H2N_DUMMY_PATTERN         =  (uint32_t)0xFFFFFFFF;
const uint32_t N2H_SYNC_PATTERN          =  (uint32_t)0xABCDDCBA;
const uint32_t SYNC_PATTERN_LEN          =  (uint32_t)sizeof(uint32_t);
const uint32_t UART_SET_MODE_MAGIC_CODE  =  (uint32_t)0xAA55AA55;
#define SPI_16BITS_BUG(pattern)     (uint32_t)((uint32_t)pattern & (uint32_t)0xFFFF7FFF)
#define SPI_8BITS_BUG(pattern)      (uint32_t)((uint32_t)pattern & (uint32_t)0xFFFFFF7F)



typedef struct {
    uint16_t Opcode;
    uint16_t Len;
} _SlGenericHeader_t;


typedef struct {
    uint32_t  Long;
    uint16_t  Short;
    uint8_t  Byte1;
    uint8_t  Byte2;
} _SlSyncPattern_t;

typedef _SlGenericHeader_t _SlCommandHeader_t;

typedef struct {
    _SlGenericHeader_t  GenHeader;
    uint8_t               TxPoolCnt;
    uint8_t               DevStatus;
    uint8_t               SocketTXFailure;
    uint8_t               SocketNonBlocking;
} _SlResponseHeader_t;

#define _SL_RESP_SPEC_HDR_SIZE (sizeof(_SlResponseHeader_t) - sizeof(_SlGenericHeader_t))
#define _SL_RESP_HDR_SIZE       sizeof(_SlResponseHeader_t)
#define _SL_CMD_HDR_SIZE        sizeof(_SlCommandHeader_t)

#define _SL_RESP_ARGS_START(_pMsg) (((_SlResponseHeader_t *)(_pMsg)) + 1)

/* Used only in NWP! */
typedef struct {
    _SlCommandHeader_t  sl_hdr;
    uint8_t   func_args_start;
} T_SCMD;


const uint8_t WLAN_CONN_STATUS_BIT  =  0x01;
const uint8_t EVENTS_Q_STATUS_BIT   =  0x02;
const uint8_t PENDING_RCV_CMD_BIT   =  0x04;
const uint8_t FW_BUSY_PACKETS_BIT   =  0x08;

const uint32_t INIT_STA_OK          =  0x11111111;
const uint32_t INIT_STA_ERR         =  0x22222222;
const uint32_t INIT_AP_OK           =  0x33333333;
const uint32_t INIT_AP_ERR          =  0x44444444;
const uint32_t INIT_P2P_OK          =  0x55555555;
const uint32_t INIT_P2P_ERR         =  0x66666666;

/****************************************************************************
**  OPCODES
****************************************************************************/
const uint16_t SL_IPV4_IPV6_OFFSET                        =     ( 9 );
const uint16_t SL_OPCODE_IPV4							  =     ( 0x0 << SL_IPV4_IPV6_OFFSET );
const uint16_t SL_OPCODE_IPV6							  =     ( 0x1 << SL_IPV4_IPV6_OFFSET );

const uint16_t SL_SYNC_ASYNC_OFFSET                       =     ( 10 );
const uint16_t SL_OPCODE_SYNC							  =     (0x1 << SL_SYNC_ASYNC_OFFSET );
const uint16_t SL_OPCODE_SILO_OFFSET                      =     ( 11 );
const uint16_t SL_OPCODE_SILO_MASK                        =     ( 0xF << SL_OPCODE_SILO_OFFSET );
const uint16_t SL_OPCODE_SILO_DEVICE                      =     ( 0x0 << SL_OPCODE_SILO_OFFSET );
const uint16_t SL_OPCODE_SILO_WLAN                        =     ( 0x1 << SL_OPCODE_SILO_OFFSET );
const uint16_t SL_OPCODE_SILO_SOCKET                      =     ( 0x2 << SL_OPCODE_SILO_OFFSET );
const uint16_t SL_OPCODE_SILO_NETAPP                      =     ( 0x3 << SL_OPCODE_SILO_OFFSET );
const uint16_t SL_OPCODE_SILO_NVMEM                       =     ( 0x4 << SL_OPCODE_SILO_OFFSET );
const uint16_t SL_OPCODE_SILO_NETCFG                      =     ( 0x5 << SL_OPCODE_SILO_OFFSET );

const uint16_t SL_FAMILY_SHIFT                            =     (0x4);
const uint16_t SL_FLAGS_MASK                              =     (0xF);

const uint16_t SL_OPCODE_DEVICE_INITCOMPLETE                         =      	0x0008;
const uint16_t SL_OPCODE_DEVICE_ABORT			                     =          0x000C;
const uint16_t SL_OPCODE_DEVICE_STOP_COMMAND                         =      	0x8473;
const uint16_t SL_OPCODE_DEVICE_STOP_RESPONSE                        =      	0x0473;
const uint16_t SL_OPCODE_DEVICE_STOP_ASYNC_RESPONSE                  =      	0x0073;
const uint16_t SL_OPCODE_DEVICE_DEVICEASYNCDUMMY                     =      	0x0063;

const uint16_t SL_OPCODE_DEVICE_VERSIONREADCOMMAND	                 =          0x8470;
const uint16_t SL_OPCODE_DEVICE_VERSIONREADRESPONSE	                 =          0x0470;
const uint16_t SL_OPCODE_DEVICE_DEVICEASYNCFATALERROR                =      	0x0078;
const uint16_t SL_OPCODE_WLAN_WLANCONNECTCOMMAND                     =      	0x8C80;
const uint16_t SL_OPCODE_WLAN_WLANCONNECTRESPONSE                    =      	0x0C80;
const uint16_t SL_OPCODE_WLAN_WLANASYNCCONNECTEDRESPONSE             =      	0x0880;
const uint16_t SL_OPCODE_WLAN_P2P_DEV_FOUND                          =          0x0830;
const uint16_t SL_OPCODE_WLAN_CONNECTION_FAILED                      =          0x0831;
const uint16_t SL_OPCODE_WLAN_P2P_NEG_REQ_RECEIVED                   =          0x0832;

const uint16_t SL_OPCODE_WLAN_WLANDISCONNECTCOMMAND                  =      	0x8C81;
const uint16_t SL_OPCODE_WLAN_WLANDISCONNECTRESPONSE                 =      	0x0C81;
const uint16_t SL_OPCODE_WLAN_WLANASYNCDISCONNECTEDRESPONSE          =      	0x0881;
const uint16_t SL_OPCODE_WLAN_WLANCONNECTEAPCOMMAND                  =      	0x8C82;
const uint16_t SL_OPCODE_WLAN_WLANCONNECTEAPCRESPONSE                =      	0x0C82;
const uint16_t SL_OPCODE_WLAN_PROFILEADDCOMMAND                      =      	0x8C83;
const uint16_t SL_OPCODE_WLAN_PROFILEADDRESPONSE                     =      	0x0C83;
const uint16_t SL_OPCODE_WLAN_PROFILEGETCOMMAND                      =      	0x8C84;
const uint16_t SL_OPCODE_WLAN_PROFILEGETRESPONSE                     =      	0x0C84;
const uint16_t SL_OPCODE_WLAN_PROFILEDELCOMMAND                      =      	0x8C85;
const uint16_t SL_OPCODE_WLAN_PROFILEDELRESPONSE                     =      	0x0C85;
const uint16_t SL_OPCODE_WLAN_POLICYSETCOMMAND                       =      	0x8C86;
const uint16_t SL_OPCODE_WLAN_POLICYSETRESPONSE                      =      	0x0C86;
const uint16_t SL_OPCODE_WLAN_POLICYGETCOMMAND                       =      	0x8C87;
const uint16_t SL_OPCODE_WLAN_POLICYGETRESPONSE                      =      	0x0C87;
const uint16_t SL_OPCODE_WLAN_FILTERADD                              =      	0x8C88;
const uint16_t SL_OPCODE_WLAN_FILTERADDRESPONSE                      =      	0x0C88;
const uint16_t SL_OPCODE_WLAN_FILTERGET                              =      	0x8C89;
const uint16_t SL_OPCODE_WLAN_FILTERGETRESPONSE                      =      	0x0C89;
const uint16_t SL_OPCODE_WLAN_FILTERDELETE                           =      	0x8C8A;
const uint16_t SL_OPCODE_WLAN_FILTERDELETERESPOSNE                   =      	0x0C8A;
const uint16_t SL_OPCODE_WLAN_WLANGETSTATUSCOMMAND                   =      	0x8C8F;
const uint16_t SL_OPCODE_WLAN_WLANGETSTATUSRESPONSE                  =      	0x0C8F;
const uint16_t SL_OPCODE_WLAN_STARTTXCONTINUESCOMMAND                =      	0x8CAA;
const uint16_t SL_OPCODE_WLAN_STARTTXCONTINUESRESPONSE               =      	0x0CAA;
const uint16_t SL_OPCODE_WLAN_STOPTXCONTINUESCOMMAND                 =      	0x8CAB;
const uint16_t SL_OPCODE_WLAN_STOPTXCONTINUESRESPONSE                =      	0x0CAB;
const uint16_t SL_OPCODE_WLAN_STARTRXSTATCOMMAND                     =      	0x8CAC;
const uint16_t SL_OPCODE_WLAN_STARTRXSTATRESPONSE                    =      	0x0CAC;
const uint16_t SL_OPCODE_WLAN_STOPRXSTATCOMMAND                      =      	0x8CAD;
const uint16_t SL_OPCODE_WLAN_STOPRXSTATRESPONSE                     =      	0x0CAD;
const uint16_t SL_OPCODE_WLAN_GETRXSTATCOMMAND                       =      	0x8CAF;
const uint16_t SL_OPCODE_WLAN_GETRXSTATRESPONSE                      =      	0x0CAF;
const uint16_t SL_OPCODE_WLAN_POLICYSETCOMMANDNEW                    =      	0x8CB0;
const uint16_t SL_OPCODE_WLAN_POLICYSETRESPONSENEW                   =      	0x0CB0;
const uint16_t SL_OPCODE_WLAN_POLICYGETCOMMANDNEW                    =      	0x8CB1;
const uint16_t SL_OPCODE_WLAN_POLICYGETRESPONSENEW                   =      	0x0CB1;

const uint16_t SL_OPCODE_WLAN_SMART_CONFIG_START_COMMAND             =      	0x8CB2;
const uint16_t SL_OPCODE_WLAN_SMART_CONFIG_START_RESPONSE            =      	0x0CB2;
const uint16_t SL_OPCODE_WLAN_SMART_CONFIG_START_ASYNC_RESPONSE      =      	0x08B2;
const uint16_t SL_OPCODE_WLAN_SMART_CONFIG_STOP_COMMAND              =      	0x8CB3;
const uint16_t SL_OPCODE_WLAN_SMART_CONFIG_STOP_RESPONSE             =      	0x0CB3;
const uint16_t SL_OPCODE_WLAN_SMART_CONFIG_STOP_ASYNC_RESPONSE       =      	0x08B3;
const uint16_t SL_OPCODE_WLAN_SET_MODE                               =          0x8CB4;
const uint16_t SL_OPCODE_WLAN_SET_MODE_RESPONSE                      =          0x0CB4;
const uint16_t SL_OPCODE_WLAN_CFG_SET                                =          0x8CB5;
const uint16_t SL_OPCODE_WLAN_CFG_SET_RESPONSE                       =          0x0CB5;
const uint16_t SL_OPCODE_WLAN_CFG_GET                                =          0x8CB6;
const uint16_t SL_OPCODE_WLAN_CFG_GET_RESPONSE                       =          0x0CB6;
const uint16_t SL_OPCODE_WLAN_STA_CONNECTED                          =      	0x082E;
const uint16_t SL_OPCODE_WLAN_STA_DISCONNECTED                       =      	0x082F;
const uint16_t SL_OPCODE_WLAN_EAP_PROFILEADDCOMMAND                  =          0x8C67;
const uint16_t SL_OPCODE_WLAN_EAP_PROFILEADDCOMMAND_RESPONSE         =          0x0C67;

const uint16_t SL_OPCODE_SOCKET_SOCKET                               =      	0x9401;
const uint16_t SL_OPCODE_SOCKET_SOCKETRESPONSE                       =      	0x1401;
const uint16_t SL_OPCODE_SOCKET_CLOSE                                =      	0x9402;
const uint16_t SL_OPCODE_SOCKET_CLOSERESPONSE                        =      	0x1402;
const uint16_t SL_OPCODE_SOCKET_ACCEPT                               =      	0x9403;
const uint16_t SL_OPCODE_SOCKET_ACCEPTRESPONSE                       =      	0x1403;
const uint16_t SL_OPCODE_SOCKET_ACCEPTASYNCRESPONSE                  =      	0x1003;
const uint16_t SL_OPCODE_SOCKET_ACCEPTASYNCRESPONSE_V6               =      	0x1203;
const uint16_t SL_OPCODE_SOCKET_BIND                                 =      	0x9404;
const uint16_t SL_OPCODE_SOCKET_BIND_V6                              =      	0x9604;
const uint16_t SL_OPCODE_SOCKET_BINDRESPONSE                         =      	0x1404;
const uint16_t SL_OPCODE_SOCKET_LISTEN                               =      	0x9405;
const uint16_t SL_OPCODE_SOCKET_LISTENRESPONSE                       =      	0x1405;
const uint16_t SL_OPCODE_SOCKET_CONNECT                              =      	0x9406;
const uint16_t SL_OPCODE_SOCKET_CONNECT_V6                           =      	0x9606;
const uint16_t SL_OPCODE_SOCKET_CONNECTRESPONSE                      =      	0x1406;
const uint16_t SL_OPCODE_SOCKET_CONNECTASYNCRESPONSE                 =      	0x1006;
const uint16_t SL_OPCODE_SOCKET_SELECT                               =      	0x9407;
const uint16_t SL_OPCODE_SOCKET_SELECTRESPONSE                       =      	0x1407;
const uint16_t SL_OPCODE_SOCKET_SELECTASYNCRESPONSE                  =      	0x1007;
const uint16_t SL_OPCODE_SOCKET_SETSOCKOPT                           =      	0x9408;
const uint16_t SL_OPCODE_SOCKET_SETSOCKOPTRESPONSE                   =      	0x1408;
const uint16_t SL_OPCODE_SOCKET_GETSOCKOPT                           =      	0x9409;
const uint16_t SL_OPCODE_SOCKET_GETSOCKOPTRESPONSE                   =      	0x1409;
const uint16_t SL_OPCODE_SOCKET_RECV                                 =      	0x940A;
const uint16_t SL_OPCODE_SOCKET_RECVASYNCRESPONSE                    =      	0x100A;
const uint16_t SL_OPCODE_SOCKET_RECVFROM                             =      	0x940B;
const uint16_t SL_OPCODE_SOCKET_RECVFROMASYNCRESPONSE                =      	0x100B;
const uint16_t SL_OPCODE_SOCKET_RECVFROMASYNCRESPONSE_V6             =      	0x120B;
const uint16_t SL_OPCODE_SOCKET_SEND                                 =      	0x940C;
const uint16_t SL_OPCODE_SOCKET_SENDTO                               =      	0x940D;
const uint16_t SL_OPCODE_SOCKET_SENDTO_V6                            =      	0x960D;
const uint16_t SL_OPCODE_SOCKET_TXFAILEDASYNCRESPONSE                =      	0x100E;
const uint16_t SL_OPCODE_SOCKET_SOCKETASYNCEVENT                     =          0x100F;
const uint16_t SL_OPCODE_NETAPP_START_COMMAND                        =          0x9C0A;
const uint16_t SL_OPCODE_NETAPP_START_RESPONSE                       =         	0x1C0A;
const uint16_t SL_OPCODE_NETAPP_NETAPPSTARTRESPONSE                  =      	0x1C0A;
const uint16_t SL_OPCODE_NETAPP_STOP_COMMAND                         =      	0x9C61;
const uint16_t SL_OPCODE_NETAPP_STOP_RESPONSE                        =      	0x1C61;
const uint16_t SL_OPCODE_NETAPP_NETAPPSET                            =	        0x9C0B;
const uint16_t SL_OPCODE_NETAPP_NETAPPSETRESPONSE                    =	        0x1C0B;
const uint16_t SL_OPCODE_NETAPP_NETAPPGET                            =	        0x9C27;
const uint16_t SL_OPCODE_NETAPP_NETAPPGETRESPONSE                    =	        0x1C27;
const uint16_t SL_OPCODE_NETAPP_DNSGETHOSTBYNAME                     =      	0x9C20;
const uint16_t SL_OPCODE_NETAPP_DNSGETHOSTBYNAMERESPONSE             =      	0x1C20;
const uint16_t SL_OPCODE_NETAPP_DNSGETHOSTBYNAMEASYNCRESPONSE        =      	0x1820;
const uint16_t SL_OPCODE_NETAPP_DNSGETHOSTBYNAMEASYNCRESPONSE_V6     =      	0x1A20;
const uint16_t SL_OPCODE_NETAPP_NETAPP_MDNS_LOOKUP_SERVICE           =          0x9C71;
const uint16_t SL_OPCODE_NETAPP_NETAPP_MDNS_LOOKUP_SERVICE_RESPONSE  =          0x1C72;
const uint16_t SL_OPCODE_NETAPP_MDNSREGISTERSERVICE                  =          0x9C34;
const uint16_t SL_OPCODE_NETAPP_MDNSREGISTERSERVICERESPONSE          =          0x1C34;
const uint16_t SL_OPCODE_NETAPP_MDNSGETHOSTBYSERVICE                 =          0x9C35;
const uint16_t SL_OPCODE_NETAPP_MDNSGETHOSTBYSERVICERESPONSE         =          0x1C35;
const uint16_t SL_OPCODE_NETAPP_MDNSGETHOSTBYSERVICEASYNCRESPONSE    =          0x1835;
const uint16_t SL_OPCODE_NETAPP_MDNSGETHOSTBYSERVICEASYNCRESPONSE_V6 =          0x1A35;
const uint16_t SL_OPCODE_NETAPP_DNSGETHOSTBYADDR                     =      	0x9C26;
const uint16_t SL_OPCODE_NETAPP_DNSGETHOSTBYADDR_V6                  =      	0x9E26;
const uint16_t SL_OPCODE_NETAPP_DNSGETHOSTBYADDRRESPONSE             =      	0x1C26;
const uint16_t SL_OPCODE_NETAPP_DNSGETHOSTBYADDRASYNCRESPONSE        =      	0x1826;
const uint16_t SL_OPCODE_NETAPP_PINGSTART                            =      	0x9C21;
const uint16_t SL_OPCODE_NETAPP_PINGSTART_V6                         =      	0x9E21;
const uint16_t SL_OPCODE_NETAPP_PINGSTARTRESPONSE                    =      	0x1C21;
const uint16_t SL_OPCODE_NETAPP_PINGREPORTREQUEST                    =      	0x9C22;
const uint16_t SL_OPCODE_NETAPP_PINGREPORTREQUESTRESPONSE            =      	0x1822;
const uint16_t SL_OPCODE_NETAPP_PINGSTOP                             =      	0x9C23;
const uint16_t SL_OPCODE_NETAPP_PINGSTOPRESPONSE                     =      	0x1C23;
const uint16_t SL_OPCODE_NETAPP_ARPFLUSH                             =      	0x9C24;
const uint16_t SL_OPCODE_NETAPP_ARPFLUSHRESPONSE                     =      	0x1C24;
const uint16_t SL_OPCODE_NETAPP_IPACQUIRED                           =      	0x1825;
const uint16_t SL_OPCODE_NETAPP_IPV4_LOST	                         =        	0x1832;
const uint16_t SL_OPCODE_NETAPP_DHCP_IPV4_ACQUIRE_TIMEOUT            =      	0x1833;
const uint16_t SL_OPCODE_NETAPP_IPACQUIRED_V6                        =      	0x1A25;
const uint16_t SL_OPCODE_NETAPP_IPERFSTARTCOMMAND                    =      	0x9C28;
const uint16_t SL_OPCODE_NETAPP_IPERFSTARTRESPONSE                   =      	0x1C28;
const uint16_t SL_OPCODE_NETAPP_IPERFSTOPCOMMAND                     =      	0x9C29;
const uint16_t SL_OPCODE_NETAPP_IPERFSTOPRESPONSE                    =      	0x1C29;
const uint16_t SL_OPCODE_NETAPP_CTESTSTARTCOMMAND                    =      	0x9C2A;
const uint16_t SL_OPCODE_NETAPP_CTESTSTARTRESPONSE                   =      	0x1C2A;
const uint16_t SL_OPCODE_NETAPP_CTESTASYNCRESPONSE                   =      	0x182A;
const uint16_t SL_OPCODE_NETAPP_CTESTSTOPCOMMAND                     =      	0x9C2B;
const uint16_t SL_OPCODE_NETAPP_CTESTSTOPRESPONSE                    =      	0x1C2B;
const uint16_t SL_OPCODE_NETAPP_IP_LEASED                            =      	0x182C;
const uint16_t SL_OPCODE_NETAPP_IP_RELEASED                          =      	0x182D;
const uint16_t SL_OPCODE_NETAPP_HTTPGETTOKENVALUE                    =      	0x182E;
const uint16_t SL_OPCODE_NETAPP_HTTPSENDTOKENVALUE                   =      	0x9C2F;
const uint16_t SL_OPCODE_NETAPP_HTTPPOSTTOKENVALUE                   =      	0x1830;
const uint16_t SL_OPCODE_NVMEM_FILEOPEN                              =      	0xA43C;
const uint16_t SL_OPCODE_NVMEM_FILEOPENRESPONSE                      =       	0x243C;
const uint16_t SL_OPCODE_NVMEM_FILECLOSE                             =       	0xA43D;
const uint16_t SL_OPCODE_NVMEM_FILECLOSERESPONSE                     =      	0x243D;
const uint16_t SL_OPCODE_NVMEM_FILEREADCOMMAND                       =       	0xA440;
const uint16_t SL_OPCODE_NVMEM_FILEREADRESPONSE                      =      	0x2440;
const uint16_t SL_OPCODE_NVMEM_FILEWRITECOMMAND                      =      	0xA441;
const uint16_t SL_OPCODE_NVMEM_FILEWRITERESPONSE                     =      	0x2441;
const uint16_t SL_OPCODE_NVMEM_FILEGETINFOCOMMAND                    =      	0xA442;
const uint16_t SL_OPCODE_NVMEM_FILEGETINFORESPONSE                   =      	0x2442;
const uint16_t SL_OPCODE_NVMEM_FILEDELCOMMAND                        =      	0xA443;
const uint16_t SL_OPCODE_NVMEM_FILEDELRESPONSE                       =      	0x2443;
const uint16_t SL_OPCODE_NVMEM_NVMEMFORMATCOMMAND                    =      	0xA444;
const uint16_t SL_OPCODE_NVMEM_NVMEMFORMATRESPONSE                   =      	0x2444;

const uint16_t SL_OPCODE_DEVICE_SETDEBUGLEVELCOMMAND                 =      	0x846A;
const uint16_t SL_OPCODE_DEVICE_SETDEBUGLEVELRESPONSE                =      	0x046A;

const uint16_t SL_OPCODE_DEVICE_NETCFG_SET_COMMAND                 	 =          0x8432;
const uint16_t SL_OPCODE_DEVICE_NETCFG_SET_RESPONSE                	 =          0x0432;
const uint16_t SL_OPCODE_DEVICE_NETCFG_GET_COMMAND                 	 =          0x8433;
const uint16_t SL_OPCODE_DEVICE_NETCFG_GET_RESPONSE                	 =          0x0433;
/*  */
const uint16_t SL_OPCODE_DEVICE_SETUARTMODECOMMAND                   =      	0x846B;
const uint16_t SL_OPCODE_DEVICE_SETUARTMODERESPONSE                  =      	0x046B;
const uint16_t SL_OPCODE_DEVICE_SSISIZESETCOMMAND	                 =          0x846B;
const uint16_t SL_OPCODE_DEVICE_SSISIZESETRESPONSE	                 =          0x046B;

/*  */
const uint16_t SL_OPCODE_DEVICE_EVENTMASKSET                         =      	0x8464;
const uint16_t SL_OPCODE_DEVICE_EVENTMASKSETRESPONSE                 =      	0x0464;
const uint16_t SL_OPCODE_DEVICE_EVENTMASKGET                         =      	0x8465;
const uint16_t SL_OPCODE_DEVICE_EVENTMASKGETRESPONSE                 =      	0x0465;

const uint16_t SL_OPCODE_DEVICE_DEVICEGET                            =      	0x8466;
const uint16_t SL_OPCODE_DEVICE_DEVICEGETRESPONSE                    =          0x0466;
const uint16_t SL_OPCODE_DEVICE_DEVICESET							 =			0x84B7;
const uint16_t SL_OPCODE_DEVICE_DEVICESETRESPONSE					 =			0x04B7;

const uint16_t SL_OPCODE_WLAN_SCANRESULTSGETCOMMAND                  =      	0x8C8C;
const uint16_t SL_OPCODE_WLAN_SCANRESULTSGETRESPONSE                 =      	0x0C8C;
const uint16_t SL_OPCODE_WLAN_SMARTCONFIGOPTSET                      =      	0x8C8D;
const uint16_t SL_OPCODE_WLAN_SMARTCONFIGOPTSETRESPONSE              =      	0x0C8D;
const uint16_t SL_OPCODE_WLAN_SMARTCONFIGOPTGET                      =      	0x8C8E;
const uint16_t SL_OPCODE_WLAN_SMARTCONFIGOPTGETRESPONSE              =      	0x0C8E;


/* Rx Filters opcodes */
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERADDCOMMAND                 =          0x8C6C;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERADDRESPONSE                =          0x0C6C;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERSETCOMMAND                 =          0x8C6D;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERSETRESPONSE                =          0x0C6D;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERGETSTATISTICSINFOCOMMAND   =          0x8C6E;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERGETSTATISTICSINFORESPONSE  =          0x0C6E;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERGETCOMMAND                 =          0x8C6F;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERGETRESPONSE                =          0x0C6F;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERGETINFO                    =          0x8C70;
const uint16_t SL_OPCODE_WLAN_WLANRXFILTERGETINFORESPONSE            =          0x0C70;


/******************************************************************************************/
/*   Device structs  */
/******************************************************************************************/
typedef uint32_t InitStatus_t;


typedef struct {
    int32_t Status;
} InitComplete_t;

typedef struct {
    int16_t status;
    uint16_t padding;

} _BasicResponse_t;

typedef struct {
    uint16_t Timeout;
    uint16_t padding;
} _DevStopCommand_t;

typedef struct {
    uint32_t group;
    uint32_t mask;
} _DevMaskEventSetCommand_t;

typedef _BasicResponse_t _DevMaskEventSetResponse_t;


typedef struct {
    uint32_t group;
} _DevMaskEventGetCommand_t;


typedef struct {
    uint32_t group;
    uint32_t mask;
} _DevMaskEventGetResponse_t;


typedef struct {
    uint32_t group;
} _DevStatusGetCommand_t;


typedef struct {
    uint32_t group;
    uint32_t status;
} _DevStatusGetResponse_t;

typedef struct {
    uint32_t  ChipId;
    uint32_t  FwVersion[4];
    uint8_t   PhyVersion[4];
} _Device_VersionReadResponsePart_t;

typedef struct {
    _Device_VersionReadResponsePart_t part;
    uint32_t                            NwpVersion[4];
    uint16_t                            RomVersion;
    uint16_t                            Padding;
} _Device_VersionReadResponseFull_t;


typedef struct {
    uint32_t BaudRate;
    uint8_t  FlowControlEnable;
} _DevUartSetModeCommand_t;

typedef _BasicResponse_t _DevUartSetModeResponse_t;

/******************************************************/

typedef struct {
    uint8_t SsiSizeInBytes;
    uint8_t Padding[3];
} _StellarisSsiSizeSet_t;

/*****************************************************************************************/
/*   WLAN structs */
/*****************************************************************************************/
#define MAXIMAL_PASSWORD_LENGTH					(64)

typedef struct {
    uint8_t	SecType;
    uint8_t	SsidLen;
    uint8_t	Bssid[6];
    uint8_t	PasswordLen;
} _WlanConnectCommon_t;

#define SSID_STRING(pCmd)       (int8_t *)((_WlanConnectCommon_t *)(pCmd) + 1)
#define PASSWORD_STRING(pCmd)   (SSID_STRING(pCmd) + ((_WlanConnectCommon_t *)(pCmd))->SsidLen)

typedef struct {
    _WlanConnectCommon_t            Common;
    uint8_t							UserLen;
    uint8_t							AnonUserLen;
    uint8_t   						CertIndex;
    uint32_t  						EapBitmask;
} _WlanConnectEapCommand_t;

#define EAP_SSID_STRING(pCmd)       (int8_t *)((_WlanConnectEapCommand_t *)(pCmd) + 1)
#define EAP_PASSWORD_STRING(pCmd)   (EAP_SSID_STRING(pCmd) + ((_WlanConnectEapCommand_t *)(pCmd))->Common.SsidLen)
#define EAP_USER_STRING(pCmd)       (EAP_PASSWORD_STRING(pCmd) + ((_WlanConnectEapCommand_t *)(pCmd))->Common.PasswordLen)
#define EAP_ANON_USER_STRING(pCmd)  (EAP_USER_STRING(pCmd) + ((_WlanConnectEapCommand_t *)(pCmd))->UserLen)


typedef struct {
    uint8_t	PolicyType;
    uint8_t       Padding;
    uint8_t	PolicyOption;
    uint8_t	PolicyOptionLen;
} _WlanPoliciySetGet_t;


typedef struct {
    uint32_t  minDwellTime;
    uint32_t  maxDwellTime;
    uint32_t  numProbeResponse;
    uint32_t  G_Channels_mask;
    int32_t   rssiThershold;
    int32_t   snrThershold;
    int32_t   defaultTXPower;
    uint16_t  intervalList[16];
} _WlanScanParamSetCommand_t;


typedef struct {
    int8_t	SecType;
    uint8_t	SsidLen;
    uint8_t	Priority;
    uint8_t	Bssid[6];
    uint8_t   PasswordLen;
    uint8_t   WepKeyId;
} _WlanAddGetProfile_t;


typedef struct {
    _WlanAddGetProfile_t              Common;
    uint8_t                             UserLen;
    uint8_t                             AnonUserLen;
    uint8_t                             CertIndex;
    uint16_t                            padding;
    uint32_t                            EapBitmask;
} _WlanAddGetEapProfile_t;

#define PROFILE_SSID_STRING(pCmd)       ((int8_t *)((_WlanAddGetProfile_t *)(pCmd) + 1))
#define PROFILE_PASSWORD_STRING(pCmd)   (PROFILE_SSID_STRING(pCmd) + ((_WlanAddGetProfile_t *)(pCmd))->SsidLen)

#define EAP_PROFILE_SSID_STRING(pCmd)       (int8_t *)((_WlanAddGetEapProfile_t *)(pCmd) + 1)
#define EAP_PROFILE_PASSWORD_STRING(pCmd)   (EAP_PROFILE_SSID_STRING(pCmd) + ((_WlanAddGetEapProfile_t *)(pCmd))->Common.SsidLen)
#define EAP_PROFILE_USER_STRING(pCmd)       (EAP_PROFILE_PASSWORD_STRING(pCmd) + ((_WlanAddGetEapProfile_t *)(pCmd))->Common.PasswordLen)
#define EAP_PROFILE_ANON_USER_STRING(pCmd)  (EAP_PROFILE_USER_STRING(pCmd) + ((_WlanAddGetEapProfile_t *)(pCmd))->UserLen)

typedef struct {
    uint8_t	index;
    uint8_t	padding[3];
} _WlanProfileDelGetCommand_t;

typedef _BasicResponse_t _WlanGetNetworkListResponse_t;

typedef struct {
    uint8_t 	index;
    uint8_t 	count;
    int8_t 	padding[2];
} _WlanGetNetworkListCommand_t;




typedef struct {
    uint32_t  						groupIdBitmask;
    uint8_t                           cipher;
    uint8_t                           publicKeyLen;
    uint8_t                           group1KeyLen;
    uint8_t                           group2KeyLen;
} _WlanSmartConfigStartCommand_t;

#define SMART_CONFIG_START_PUBLIC_KEY_STRING(pCmd)       ((int8_t *)((_WlanSmartConfigStartCommand_t *)(pCmd) + 1))
#define SMART_CONFIG_START_GROUP1_KEY_STRING(pCmd)       ((int8_t *) (SMART_CONFIG_START_PUBLIC_KEY_STRING(pCmd) + ((_WlanSmartConfigStartCommand_t *)(pCmd))->publicKeyLen))
#define SMART_CONFIG_START_GROUP2_KEY_STRING(pCmd)       ((int8_t *) (SMART_CONFIG_START_GROUP1_KEY_STRING(pCmd) + ((_WlanSmartConfigStartCommand_t *)(pCmd))->group1KeyLen))

typedef	struct {
    uint8_t	mode;
    uint8_t   padding[3];
} _WlanSetMode_t;




typedef struct {
    uint16_t  Status;
    uint16_t  ConfigId;
    uint16_t  ConfigOpt;
    uint16_t  ConfigLen;
} _WlanCfgSetGet_t;


//wlan_rx_filters moved

typedef struct {
    uint16_t status;
    uint8_t  WlanRole;     /* 0 = station, 2 = AP */
    uint8_t  Ipv6Enabled;
    uint8_t  Ipv6DhcpEnabled;

    uint32_t ipV6Global[4];
    uint32_t ipV6Local[4];
    uint32_t ipV6DnsServer[4];
    uint8_t  Ipv6DhcpState;

} _NetappIpV6configRetArgs_t;


typedef struct {
    uint8_t  ipV4[4];
    uint8_t  ipV4Mask[4];
    uint8_t  ipV4Gateway[4];
    uint8_t  ipV4DnsServer[4];
    uint8_t  ipV4Start[4];
    uint8_t  ipV4End[4];
} _NetCfgIpV4AP_Args_t;



typedef struct {
    uint16_t status;
    uint8_t  MacAddr[6];
} _MAC_Address_SetGet_t;


typedef struct {
    uint16_t  Status;
    uint16_t	ConfigId;
    uint16_t	ConfigOpt;
    uint16_t	ConfigLen;
} _NetCfgSetGet_t;

typedef struct {
    uint16_t  Status;
    uint16_t  DeviceSetId;
    uint16_t  Option;
    uint16_t  ConfigLen;
} _DeviceSetGet_t;




/******************************************************************************************/
/*   Socket structs  */
/******************************************************************************************/

typedef struct {
    uint8_t Domain;
    uint8_t Type;
    uint8_t Protocol;
    uint8_t Padding;
} _SocketCommand_t;


typedef struct {
    int16_t statusOrLen;
    uint8_t  sd;
    uint8_t  padding;
} _SocketResponse_t;

typedef struct {
    uint8_t sd;
    uint8_t family;
    uint8_t padding1;
    uint8_t padding2;
} _AcceptCommand_t;


typedef struct {
    int16_t statusOrLen;
    uint8_t sd;
    uint8_t family;
    uint16_t port;
    uint16_t paddingOrAddr;
    uint32_t address;
} _SocketAddrAsyncIPv4Response_t;

typedef struct {
    int16_t statusOrLen;
    uint8_t sd;
    uint8_t family;
    uint16_t port;
    uint8_t address[6];
} _SocketAddrAsyncIPv6EUI48Response_t;
typedef struct {
    int16_t statusOrLen;
    uint8_t sd;
    uint8_t family;
    uint16_t port;
    uint16_t paddingOrAddr;
    uint32_t address[4];
} _SocketAddrAsyncIPv6Response_t;


typedef struct {
    int16_t lenOrPadding;
    uint8_t sd;
    uint8_t FamilyAndFlags;
    uint16_t port;
    uint16_t paddingOrAddr;
    uint32_t address;
} _SocketAddrIPv4Command_t;

typedef struct {
    int16_t lenOrPadding;
    uint8_t sd;
    uint8_t FamilyAndFlags;
    uint16_t port;
    uint8_t address[6];
} _SocketAddrIPv6EUI48Command_t;
typedef struct {
    int16_t lenOrPadding;
    uint8_t sd;
    uint8_t FamilyAndFlags;
    uint16_t port;
    uint16_t paddingOrAddr;
    uint32_t address[4];
} _SocketAddrIPv6Command_t;

typedef union {
    _SocketAddrIPv4Command_t IpV4;
    _SocketAddrIPv6EUI48Command_t IpV6EUI48;
#ifdef SL_SUPPORT_IPV6
    _SocketAddrIPv6Command_t IpV6;
#endif
} _SocketAddrCommand_u;

typedef union {
    _SocketAddrAsyncIPv4Response_t IpV4;
    _SocketAddrAsyncIPv6EUI48Response_t IpV6EUI48;
#ifdef SL_SUPPORT_IPV6
    _SocketAddrAsyncIPv6Response_t IpV6;
#endif
} _SocketAddrResponse_u;

typedef struct {
    uint8_t sd;
    uint8_t backlog;
    uint8_t padding1;
    uint8_t padding2;
} _ListenCommand_t;

typedef struct {
    uint8_t sd;
    uint8_t padding0;
    uint8_t padding1;
    uint8_t padding2;
} _CloseCommand_t;


typedef struct {
    uint8_t nfds;
    uint8_t readFdsCount;
    uint8_t writeFdsCount;
    uint8_t padding;
    uint16_t readFds;
    uint16_t writeFds;
    uint16_t tv_usec;
    uint16_t tv_sec;
} _SelectCommand_t;


typedef struct {
    uint16_t status;
    uint8_t readFdsCount;
    uint8_t writeFdsCount;
    uint16_t readFds;
    uint16_t writeFds;
} _SelectAsyncResponse_t;

typedef struct {
    uint8_t sd;
    uint8_t level;
    uint8_t optionName;
    uint8_t optionLen;
} _setSockOptCommand_t;

typedef struct {
    uint8_t sd;
    uint8_t level;
    uint8_t optionName;
    uint8_t optionLen;
} _getSockOptCommand_t;

typedef struct {
    int16_t status;
    uint8_t sd;
    uint8_t optionLen;
} _getSockOptResponse_t;


typedef struct {
    uint16_t StatusOrLen;
    uint8_t  sd;
    uint8_t FamilyAndFlags;
} _sendRecvCommand_t;

//netapp structs moved

/*****************************************************************************************
*   FS structs
******************************************************************************************/

typedef struct {
    uint32_t FileHandle;
    uint32_t Offset;
    uint16_t Len;
    uint16_t Padding;
} _FsReadCommand_t;

typedef struct {
    uint32_t Mode;
    uint32_t Token;
} _FsOpenCommand_t;

typedef struct {
    uint32_t FileHandle;
    uint32_t Token;
} _FsOpenResponse_t;


typedef struct {
    uint32_t FileHandle;
    uint32_t CertificFileNameLength;
    uint32_t SignatureLen;
} _FsCloseCommand_t;


typedef _BasicResponse_t _FsReadResponse_t;
typedef _BasicResponse_t _FsDeleteResponse_t;
typedef _BasicResponse_t _FsCloseResponse_t;

typedef struct {
    uint16_t Status;
    uint16_t flags;
    uint32_t FileLen;
    uint32_t AllocatedLen;
    uint32_t Token[4];
} _FsGetInfoResponse_t;

typedef struct {
    uint8_t DeviceID;
    uint8_t Padding[3];
} _FsFormatCommand_t;

typedef _BasicResponse_t _FsFormatResponse_t;

typedef struct {
    uint32_t Token;
} _FsDeleteCommand_t;

typedef   _FsDeleteCommand_t  _FsGetInfoCommand_t;

typedef struct {
    uint32_t FileHandle;
    uint32_t Offset;
    uint16_t Len;
    uint16_t Padding;
} _FsWriteCommand_t;

typedef _BasicResponse_t _FsWriteResponse_t;


/* TODO: Set MAx Async Payload length depending on flavor (Tiny, Small, etc.) */

#ifdef SL_TINY_EXT
#define SL_ASYNC_MAX_PAYLOAD_LEN        120  /* size must be aligned to 4 */
#else
#define SL_ASYNC_MAX_PAYLOAD_LEN        160 /* size must be aligned to 4 */
#endif
#define SL_ASYNC_MAX_MSG_LEN            (_SL_RESP_HDR_SIZE + SL_ASYNC_MAX_PAYLOAD_LEN)

#define RECV_ARGS_SIZE                  (sizeof(_SocketResponse_t))
#define RECVFROM_IPV4_ARGS_SIZE         (sizeof(_SocketAddrAsyncIPv4Response_t))
#define RECVFROM_IPV6_ARGS_SIZE         (sizeof(_SocketAddrAsyncIPv6Response_t))

#define SL_IPV4_ADDRESS_SIZE 			(sizeof(uint32_t))
#define SL_IPV6_ADDRESS_SIZE 			(4 * sizeof(uint32_t))

}//namespace mbed_cc3100

#endif /*  _SL_PROTOCOL_TYPES_H_  */

