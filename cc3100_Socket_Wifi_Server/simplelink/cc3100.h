/*
 * device.h - CC31xx/CC32xx Host Driver Implementation
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

#ifndef DEVICE_H_
#define DEVICE_H_

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "mbed.h"
#include "cc3100_simplelink.h"
#include "cc3100_driver.h"
#include "cc3100_wlan_rx_filters.h"

#include "cc3100_spi.h"
#include "cc3100_netcfg.h"

namespace mbed_cc3100 {

/*!

    \addtogroup device
    @{

*/
    const int16_t ROLE_UNKNOWN_ERR          =            -1;

    const uint16_t   MAX_BUFF_SIZE =  1460;
    extern uint32_t  g_PingPacketsRecv;
    extern uint32_t  g_GatewayIP;
    extern uint32_t  g_StationIP;
    extern uint32_t  g_DestinationIP;
    extern uint32_t  g_BytesReceived; // variable to store the file size 
    extern uint32_t  g_Status;
    extern uint8_t   g_buff[MAX_BUFF_SIZE+1];
    extern int32_t   g_SockID;


/* File on the serial flash */
#define FILE_NAME "cc3000_module.pdf"
#define HOST_NAME       "www.ti.com"

#define HTTP_FILE_NOT_FOUND    "404 Not Found" /* HTTP file not found response */
#define HTTP_STATUS_OK         "200 OK"  /* HTTP status ok response */
#define HTTP_CONTENT_LENGTH    "Content-Length:"  /* HTTP content length header */
#define HTTP_TRANSFER_ENCODING "Transfer-Encoding:" /* HTTP transfer encoding header */
#define HTTP_ENCODING_CHUNKED  "chunked" /* HTTP transfer encoding header value */
#define HTTP_CONNECTION        "Connection:" /* HTTP Connection header */
#define HTTP_CONNECTION_CLOSE  "close"  /* HTTP Connection header value */
#define HTTP_END_OF_HEADER  "\r\n\r\n"  /* string marking the end of headers in response */

/*****************************************************************************/
/* Macro declarations                                                        */
/*****************************************************************************/

const uint16_t IP_LEASE_TIME    =    3600;

const uint16_t SIZE_45K      =  46080;  /* Serial flash file size 45 KB */
const uint16_t READ_SIZE     =  1450;
const uint8_t SPACE          =  32;

const uint16_t PING_INTERVAL   =   1000;
const uint8_t  PING_SIZE       =   20;
const uint16_t PING_TIMEOUT    =   3000;
const uint8_t  PING_ATTEMPTS   =   3;
const uint8_t  PING_PKT_SIZE   =   20; 

const uint8_t SL_STOP_TIMEOUT    =    0xFF;

/* SL internal Error codes */

/* Receive this error in case there are no resources to issue the command
   If possible, increase the number of MAX_CUNCURENT_ACTIONS (result in memory increase)
   If not, try again later */
const int16_t SL_POOL_IS_EMPTY  = (-2000);

/* Receive this error in case a given length for RX buffer was too small.
   Receive payload was bigger than the given buffer size. Therefore, payload is cut according to receive size
   Recommend to increase buffer size */
const int16_t SL_ESMALLBUF      = (-2001);

/* Receive this error in case zero length is supplied to a "get" API
   Recommend to supply length according to requested information (view options defines for help) */
const int16_t SL_EZEROLEN       = (-2002);

/* User supplied invalid parameter */
const int16_t SL_INVALPARAM     = (-2003);

/* Failed to open interface  */
const int16_t SL_BAD_INTERFACE  = (-2004);

/* End of SL internal Error codes */

/*****************************************************************************/
/* Errors returned from the general error async event                        */
/*****************************************************************************/        
    
/* Use bit 32: Lower bits of status variable are used for NWP events
 *      1 in a 'status_variable', the device has completed the ping operation
 *      0 in a 'status_variable', the device has not completed the ping operation
 */
//const uint32_t STATUS_BIT_PING_DONE = 31;

/* Status bits - These are used to set/reset the corresponding bits in a 'status_variable' */
typedef enum {
    STATUS_BIT_CONNECTION =  0, /* If this bit is:
                                 *      1 in a 'status_variable', the device is connected to the AP
                                 *      0 in a 'status_variable', the device is not connected to the AP
                                 */

    STATUS_BIT_STA_CONNECTED,    /* If this bit is:
                                  *      1 in a 'status_variable', client is connected to device
                                  *      0 in a 'status_variable', client is not connected to device
                                  */

    STATUS_BIT_IP_ACQUIRED,       /* If this bit is:
                                   *      1 in a 'status_variable', the device has acquired an IP
                                   *      0 in a 'status_variable', the device has not acquired an IP
                                   */

    STATUS_BIT_IP_LEASED,           /* If this bit is:
                                      *      1 in a 'status_variable', the device has leased an IP
                                      *      0 in a 'status_variable', the device has not leased an IP
                                      */

    STATUS_BIT_CONNECTION_FAILED,   /* If this bit is:
                                     *      1 in a 'status_variable', failed to connect to device
                                     *      0 in a 'status_variable'
                                     */

    STATUS_BIT_P2P_NEG_REQ_RECEIVED,/* If this bit is:
                                     *      1 in a 'status_variable', connection requested by remote wifi-direct device
                                     *      0 in a 'status_variable',
                                     */
    STATUS_BIT_SMARTCONFIG_DONE,    /* If this bit is:
                                     *      1 in a 'status_variable', smartconfig completed
                                     *      0 in a 'status_variable', smartconfig event couldn't complete
                                     */

    STATUS_BIT_SMARTCONFIG_STOPPED,  /* If this bit is:
                                     *      1 in a 'status_variable', smartconfig process stopped
                                     *      0 in a 'status_variable', smartconfig process running
                                     */
                                     
    STATUS_BIT_PING_DONE = 31
                                     /* Use bit 32: Lower bits of status variable are used for NWP events
                                      *      1 in a 'status_variable', the device has completed the ping operation
                                      *      0 in a 'status_variable', the device has not completed the ping operation
                                      */                                 

} e_StatusBits;        
    
/* Application specific status/error codes */
typedef enum {
    LAN_CONNECTION_FAILED = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    SNTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    SNTP_RECV_ERROR = SNTP_SEND_ERROR - 1,
    SNTP_SERVER_RESPONSE_ERROR = SNTP_RECV_ERROR - 1,
    INVALID_HEX_STRING = DEVICE_NOT_IN_STATION_MODE - 1,
    TCP_RECV_ERROR = INVALID_HEX_STRING - 1,
    TCP_SEND_ERROR = TCP_RECV_ERROR - 1,
    FILE_NOT_FOUND_ERROR = TCP_SEND_ERROR - 1,
    INVALID_SERVER_RESPONSE = FILE_NOT_FOUND_ERROR - 1,
    FORMAT_NOT_SUPPORTED = INVALID_SERVER_RESPONSE - 1,
    FILE_WRITE_ERROR = FORMAT_NOT_SUPPORTED - 1,
    INVALID_FILE = FILE_WRITE_ERROR - 1,

    STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;    
    
    /* Send types */
typedef enum {
    SL_ERR_SENDER_HEALTH_MON,
    SL_ERR_SENDER_CLI_UART,
    SL_ERR_SENDER_SUPPLICANT,
    SL_ERR_SENDER_NETWORK_STACK,
    SL_ERR_SENDER_WLAN_DRV_IF,
    SL_ERR_SENDER_WILINK,
    SL_ERR_SENDER_INIT_APP,
    SL_ERR_SENDER_NETX,
    SL_ERR_SENDER_HOST_APD,
    SL_ERR_SENDER_MDNS,
    SL_ERR_SENDER_HTTP_SERVER,
    SL_ERR_SENDER_DHCP_SERVER,
    SL_ERR_SENDER_DHCP_CLIENT,
    SL_ERR_DISPATCHER,
    SL_ERR_NUM_SENDER_LAST=0xFF
} SlErrorSender_e;

/* Error codes */
const int8_t SL_ERROR_STATIC_ADDR_SUBNET_ERROR                  = (-60);  /* network stack error*/
const int8_t SL_ERROR_ILLEGAL_CHANNEL                           = (-61);  /* supplicant error */
const int8_t SL_ERROR_SUPPLICANT_ERROR                          = (-72);  /* init error code */
const int8_t SL_ERROR_HOSTAPD_INIT_FAIL                         = (-73);  /* init error code */
const int8_t SL_ERROR_HOSTAPD_INIT_IF_FAIL                      = (-74);  /* init error code */
const int8_t SL_ERROR_WLAN_DRV_INIT_FAIL                        = (-75);  /* init error code */
const int8_t SL_ERROR_WLAN_DRV_START_FAIL                       = (-76);  /* wlan start error */
const int8_t SL_ERROR_FS_FILE_TABLE_LOAD_FAILED                 = (-77);  /* init file system failed */
const int8_t SL_ERROR_PREFERRED_NETWORKS_FILE_LOAD_FAILED       = (-78);  /* init file system failed */
const int8_t SL_ERROR_HOSTAPD_BSSID_VALIDATION_ERROR            = (-79);  /* Ap configurations BSSID error */
const int8_t SL_ERROR_HOSTAPD_FAILED_TO_SETUP_INTERFACE         = (-80);  /* Ap configurations interface error */
const int8_t SL_ERROR_MDNS_ENABLE_FAIL                          = (-81);  /* mDNS enable failed                */
const int8_t SL_ERROR_HTTP_SERVER_ENABLE_FAILED                 = (-82);  /* HTTP server enable failed         */
const int8_t SL_ERROR_DHCP_SERVER_ENABLE_FAILED                 = (-83);  /* DHCP server enable failed         */
const int8_t SL_ERROR_PREFERRED_NETWORK_LIST_FULL               = (-93);  /* supplicant error  */
const int8_t SL_ERROR_PREFERRED_NETWORKS_FILE_WRITE_FAILED      = (-94);  /* supplicant error  */
const int8_t SL_ERROR_DHCP_CLIENT_RENEW_FAILED                  = (-100); /* DHCP client error */
/* WLAN Connection management status */
const int8_t SL_ERROR_CON_MGMT_STATUS_UNSPECIFIED               = (-102);
const int8_t SL_ERROR_CON_MGMT_STATUS_AUTH_REJECT               = (-103);
const int8_t SL_ERROR_CON_MGMT_STATUS_ASSOC_REJECT              = (-104);
const int8_t SL_ERROR_CON_MGMT_STATUS_SECURITY_FAILURE          = (-105);
const int8_t SL_ERROR_CON_MGMT_STATUS_AP_DEAUTHENTICATE         = (-106);
const int8_t SL_ERROR_CON_MGMT_STATUS_AP_DISASSOCIATE           = (-107);
const int8_t SL_ERROR_CON_MGMT_STATUS_ROAMING_TRIGGER           = (-108);
const int8_t SL_ERROR_CON_MGMT_STATUS_DISCONNECT_DURING_CONNECT = (-109);
const int8_t SL_ERROR_CON_MGMT_STATUS_SG_RESELECT               = (-110);
const int8_t SL_ERROR_CON_MGMT_STATUS_ROC_FAILURE               = (-111);
const int8_t SL_ERROR_CON_MGMT_STATUS_MIC_FAILURE               = (-112);
/* end of WLAN connection management error statuses */
const int8_t SL_ERROR_WAKELOCK_ERROR_PREFIX                     = (-115);  /* Wake lock expired */
const int8_t SL_ERROR_LENGTH_ERROR_PREFIX                       = (-116);  /* Uart header length error */
const int8_t SL_ERROR_MDNS_CREATE_FAIL                          = (-121);  /* mDNS create failed */
const int8_t SL_ERROR_GENERAL_ERROR                             = (-127);



const int8_t SL_DEVICE_GENERAL_CONFIGURATION           = (1);
const int8_t SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME = (11);
const int8_t SL_DEVICE_GENERAL_VERSION                 = (12);
const int8_t SL_DEVICE_STATUS                          = (2);

/*
    Declare the different event group classifications
    The SimpleLink device send asynchronous events. Each event has a group
    classification according to its nature.
*/
#if 1
/* SL_EVENT_CLASS_WLAN connection user events */
const int8_t SL_WLAN_CONNECT_EVENT                   =  (1);
const int8_t SL_WLAN_DISCONNECT_EVENT                =  (2);
/* WLAN Smart Config user events */
const int8_t SL_WLAN_SMART_CONFIG_COMPLETE_EVENT     =  (3);
const int8_t SL_WLAN_SMART_CONFIG_STOP_EVENT         =  (4);
/* WLAN AP user events */
const int8_t SL_WLAN_STA_CONNECTED_EVENT             =  (5);
const int8_t SL_WLAN_STA_DISCONNECTED_EVENT          =  (6);
/* WLAN P2P user events */
const int8_t SL_WLAN_P2P_DEV_FOUND_EVENT             =  (7);
const int8_t    SL_WLAN_P2P_NEG_REQ_RECEIVED_EVENT   =  (8);
const int8_t SL_WLAN_CONNECTION_FAILED_EVENT         =  (9);
/* SL_EVENT_CLASS_DEVICE user events */
const int8_t SL_DEVICE_FATAL_ERROR_EVENT             =  (1);
const int8_t SL_DEVICE_ABORT_ERROR_EVENT             =  (2);
/* SL_EVENT_CLASS_BSD user events */
const int8_t    SL_SOCKET_TX_FAILED_EVENT            =  (1);
const int8_t SL_SOCKET_ASYNC_EVENT                   =  (2);
/* SL_EVENT_CLASS_NETAPP user events */
const int8_t    SL_NETAPP_IPV4_IPACQUIRED_EVENT      =  (1);
const int8_t    SL_NETAPP_IPV6_IPACQUIRED_EVENT      =  (2);
const int8_t SL_NETAPP_IP_LEASED_EVENT               =  (3);
const int8_t SL_NETAPP_IP_RELEASED_EVENT             =  (4);

/* Server Events */
const int8_t SL_NETAPP_HTTPGETTOKENVALUE_EVENT       =   (1);
const int8_t SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT      =   (2);
#endif

/*
    Declare the different event group classifications for sl_DevGet
    for getting status indications
 */

/* Events list to mask/unmask*/
const int8_t SL_EVENT_CLASS_GLOBAL                   =  (0);
const int8_t SL_EVENT_CLASS_DEVICE                   =  (1);
const int8_t SL_EVENT_CLASS_WLAN                     =  (2);
const int8_t SL_EVENT_CLASS_BSD                      =  (3);
const int8_t SL_EVENT_CLASS_NETAPP                   =  (4);
const int8_t SL_EVENT_CLASS_NETCFG                   =  (5);
const int8_t SL_EVENT_CLASS_FS                       =  (6);


/******************  DEVICE CLASS status ****************/
const uint32_t EVENT_DROPPED_DEVICE_ASYNC_GENERAL_ERROR         = (0x00000001L);
const uint32_t STATUS_DEVICE_SMART_CONFIG_ACTIVE                = (0x80000000L);

/******************  WLAN CLASS status ****************/
const uint32_t EVENT_DROPPED_WLAN_WLANASYNCONNECTEDRESPONSE     = (0x00000001L);
const uint32_t EVENT_DROPPED_WLAN_WLANASYNCDISCONNECTEDRESPONSE = (0x00000002L);
const uint32_t EVENT_DROPPED_WLAN_STA_CONNECTED                 = (0x00000004L);
const uint32_t EVENT_DROPPED_WLAN_STA_DISCONNECTED              = (0x00000008L);
const uint32_t STATUS_WLAN_STA_CONNECTED                        = (0x80000000L);

/******************  NETAPP CLASS status ****************/
const uint32_t EVENT_DROPPED_NETAPP_IPACQUIRED                  = (0x00000001L);
const uint32_t EVENT_DROPPED_NETAPP_IPACQUIRED_V6               = (0x00000002L);
const uint32_t EVENT_DROPPED_NETAPP_IP_LEASED                   = (0x00000004L);
const uint32_t EVENT_DROPPED_NETAPP_IP_RELEASED                 = (0x00000008L);

/******************  BSD CLASS status ****************/
const uint32_t EVENT_DROPPED_SOCKET_TXFAILEDASYNCRESPONSE       = (0x00000001L);

/******************  FS CLASS  ****************/

/*****************************************************************************/
/* Structure/Enum declarations                                               */
/*****************************************************************************/

#ifdef SL_IF_TYPE_UART
typedef struct {
    uint32_t             BaudRate;
    uint8_t              FlowControlEnable;
    uint8_t              CommPort;
} SlUartIfParams_t;
#endif

typedef struct {
    uint32_t               ChipId;
    uint32_t               FwVersion[4];
    uint8_t                PhyVersion[4];
} _SlPartialVersion;

typedef struct {
    _SlPartialVersion ChipFwAndPhyVersion;
    uint32_t               NwpVersion[4];
    uint16_t               RomVersion;
    uint16_t               Padding;
} SlVersionFull;

typedef struct
{
    uint32_t                AbortType;
    uint32_t                AbortData;
}sl_DeviceReportAbort;

typedef struct {
    int8_t                status;
    SlErrorSender_e        sender;
} sl_DeviceReport;

typedef union {
    sl_DeviceReport           deviceEvent;
    sl_DeviceReportAbort      deviceReport;
} _SlDeviceEventData_u;

typedef struct {
    uint32_t             Event;
    _SlDeviceEventData_u EventData;
} SlDeviceEvent_t;

typedef struct {
    /* time */
    uint32_t                sl_tm_sec;
    uint32_t                sl_tm_min;
    uint32_t                sl_tm_hour;
    /* date */
    uint32_t                sl_tm_day; /* 1-31 */
    uint32_t                sl_tm_mon; /* 1-12 */
    uint32_t                sl_tm_year; /*  YYYY 4 digits  */
    uint32_t                sl_tm_week_day; /* not required */
    uint32_t                sl_tm_year_day; /* not required */
    uint32_t                reserved[3];
} SlDateTime_t;

/******************************************************************************/
/* Type declarations                                                          */
/******************************************************************************/
typedef void (*P_INIT_CALLBACK)(uint32_t Status);

class cc3100_netcfg;

class cc3100
{

public:

    cc3100(PinName cc3100_irq, PinName cc3100_nHIB, PinName cc3100_cs, SPI cc3100_spi);

    ~cc3100();

    /*****************************************************************************/
    /* Function prototypes                                                       */
    /*****************************************************************************/
    int32_t initializeAppVariables();
    
    int32_t establishConnectionWithAP(void);
    
    int32_t checkLanConnection(void);
    
    int32_t checkInternetConnection(void);
    
    int32_t createUDPConnection(void);
    
    int32_t createConnection(uint32_t DestinationIP);

    int32_t getChunkSize(int32_t *len, uint8_t **p_Buff, uint32_t *chunk_size);
    
    int32_t hexToi(unsigned char *ptr);
    
//    int32_t getFile(void);
    
    int32_t disconnectFromAP(void);
    
    uint16_t itoa(int16_t cNum, uint8_t *cString);
    
    int32_t configureSimpleLinkToDefaultState(void);
    
    int16_t _sl_GetStartResponseConvert(uint32_t Status);

    void _sl_HandleAsync_InitComplete(void *pVoidBuf); 
    
    bool IS_PING_DONE(uint32_t status_variable,const uint32_t bit);
    bool IS_CONNECTED(uint32_t status_variable,const uint32_t bit);
    bool IS_STA_CONNECTED(uint32_t status_variable,const uint32_t bit);
    bool IS_IP_ACQUIRED(uint32_t status_variable,const uint32_t bit);
    bool IS_IP_LEASED(uint32_t status_variable,const uint32_t bit);
    bool IS_CONNECTION_FAILED(uint32_t status_variable,const uint32_t bit);
    bool IS_P2P_NEG_REQ_RECEIVED(uint32_t status_variable,const uint32_t bit);
    bool IS_SMARTCONFIG_DONE(uint32_t status_variable,const uint32_t bit);
    bool IS_SMARTCONFIG_STOPPED(uint32_t status_variable,const uint32_t bit);
    
    
    
    void CLR_STATUS_BIT(uint32_t status_variable, const uint32_t bit);
    void SET_STATUS_BIT(uint32_t status_variable, const uint32_t bit);
    

    /*!
        \brief Start the SimpleLink device

        This function initialize the communication interface, set the enable pin
        of the device, and call to the init complete callback.

        \param[in]      pIfHdl              Opened Interface Object. In case the interface
                                            must be opened outside the SimpleLink Driver, the
                                            user might give the handler to be used in \n
                                            any access of the communication interface with the
                                            device (UART/SPI). \n
                                            The SimpleLink driver will open an interface port
                                            only if this parameter is null! \n
        \param[in]      pDevName            The name of the device to open. Could be used when
                                            the pIfHdl is null, to transfer information to the
                                            open interface function \n
                                            This pointer could be used to pass additional information to
                                            sl_IfOpen in case it is required (e.g. UART com port name)
        \param[in]      pInitCallBack       Pointer to function that would be called
                                            on completion of the initialization process.\n
                                            If this parameter is NULL the function is
                                            blocked until the device initialization
                                            is completed, otherwise the function returns
                                            immediately.

        \return         Returns the current active role (STA/AP/P2P) or an error code:
                        - ROLE_STA, ROLE_AP, ROLE_P2P in case of success,
                          otherwise in failure one of the following is return:
                        - ROLE_STA_ERR  (Failure to load MAC/PHY in STA role)
                        - ROLE_AP_ERR  (Failure to load MAC/PHY in AP role)
                        - ROLE_P2P_ERR  (Failure to load MAC/PHY in P2P role)


         \sa             sl_Stop

        \note           belongs to \ref basic_api

        \warning        This function must be called before any other SimpleLink API is used, or after sl_Stop is called for reinit the device
        \par            Example:
        \code
                       An example for open interface without callback routine. The interface name and handler are
                       handled by the sl_IfOpen routine:

                        if( sl_Start(NULL, NULL, NULL) < 0 )
                       {
                           LOG("Error opening interface to device\n");
                       }
        \endcode
    */
#if _SL_INCLUDE_FUNC(sl_Start)
    int16_t sl_Start(const void* pIfHdl, int8_t*  pDevName, const P_INIT_CALLBACK pInitCallBack);
#endif

    /*!
        \brief Stop the SimpleLink device

        This function clears the enable pin of the device, closes the communication \n
        interface and invokes the stop complete callback

        \param[in]      timeout                       Stop timeout in msec. Should be used to give the device time to finish \n
                                                      any transmission/reception that is not completed when the function was called. \n
                        Additional options:
                        - 0                             Enter to hibernate immediately \n
                        - 0xFFFF                        Host waits for device's response before \n
                                                        hibernating, without timeout protection \n
                        - 0 < Timeout[msec] < 0xFFFF    Host waits for device's response before \n
                                                        hibernating, with a defined timeout protection \n
                                                        This timeout defines the max time to wait. The NWP \n
                                                        response can be sent earlier than this timeout.

        \return         On success, zero is returned. On error, -1 is returned

        \sa             sl_Start

        \note           This API will shutdown the device and invoke the "i/f close" function regardless \n
                        if it was opened implicitly or explicitly. \n
                        It is up to the platform interface library to properly handle interface close \n
                        routine \n
                        belongs to \ref basic_api \n
        \warning
    */
#if _SL_INCLUDE_FUNC(sl_Stop)
    int16_t sl_Stop(const uint16_t timeout);
#endif


    /*!
        \brief     Internal function for setting device configurations

        \return    On success, zero is returned. On error, -1 is
                   returned

        \param[in] DeviceSetId   configuration id
        \param[in] Option         configurations option
        \param[in] ConfigLen     configurations len
        \param[in] pValues         configurations values

        \sa
        \note
        \warning
        \par   Examples:
        \code
             Setting device time and date example:

             SlDateTime_t dateTime= {0};
             dateTime.sl_tm_day =   (uint32_t)23;          // Day of month (DD format) range 1-13
             dateTime.sl_tm_mon =   (uint32_t)6;           // Month (MM format) in the range of 1-12
             dateTime.sl_tm_year =  (uint32_t)2014;        // Year (YYYY format)
             dateTime.sl_tm_hour =  (uint32_t)17;          // Hours in the range of 0-23
             dateTime.sl_tm_min =   (uint32_t)55;          // Minutes in the range of 0-59
             dateTime.sl_tm_sec =   (uint32_t)22;          // Seconds in the range of  0-59
             sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                       SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                       sizeof(SlDateTime_t),
                       (uint8_t *)(&dateTime));

        \endcode
    */
#if _SL_INCLUDE_FUNC(sl_DevSet)
    int32_t sl_DevSet(const uint8_t DeviceSetId , const uint8_t Option, const uint8_t ConfigLen, const uint8_t *pValues);
#endif

    /*!
        \brief      Internal function for getting device configurations
        \return     On success, zero is returned. On error, -1 is
                    returned
        \param[in]  DeviceGetId   configuration id - example SL_DEVICE_STATUS
        \param[out] pOption   Get configurations option, example for get status options
                                - SL_EVENT_CLASS_GLOBAL
                                - SL_EVENT_CLASS_DEVICE
                                - SL_EVENT_CLASS_WLAN
                                - SL_EVENT_CLASS_BSD
                                - SL_EVENT_CLASS_NETAPP
                                - SL_EVENT_CLASS_NETCFG
                                - SL_EVENT_CLASS_FS
        \param[out] pConfigLen   The length of the allocated memory as input, when the
                                 function complete, the value of this parameter would be
                                 the len that actually read from the device.\n
                                 If the device return length that is longer from the input
                                 value, the function will cut the end of the returned structure
                                 and will return SL_ESMALLBUF
        \param[out] pValues      Get configurations values
        \sa
        \note
        \warning
        \par        Examples:
        \code
         Example for getting WLAN class status:
         uint32_t statusWlan;
         uint8_t pConfigOpt;
         uint8_t pConfigLen;
         pConfigLen = sizeof(_u32);
         pConfigOpt = SL_EVENT_CLASS_WLAN;
         sl_DevGet(SL_DEVICE_STATUS,&pConfigOpt,&pConfigLen,(uint8_t *)(&statusWlan));
         Example for getting version:
         SlVersionFull ver;
         pConfigLen = sizeof(ver);
         pConfigOpt = SL_DEVICE_GENERAL_VERSION;
         sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION,&pConfigOpt,&pConfigLen,(uint8_t *)(&ver));
         printf("CHIP %d\nMAC 31.%d.%d.%d.%d\nPHY %d.%d.%d.%d\nNWP %d.%d.%d.%d\nROM %d\nHOST %d.%d.%d.%d\n",
                 ver.ChipFwAndPhyVersion.ChipId,
                 ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
                 ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
                 ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
                 ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3],
                 ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
                 ver.RomVersion,
                 SL_MAJOR_VERSION_NUM,SL_MINOR_VERSION_NUM,SL_VERSION_NUM,SL_SUB_VERSION_NUM);

         \endcode
         \code
             Getting Device time and date example:

             SlDateTime_t dateTime =  {0};
             int8_t configLen = sizeof(SlDateTime_t);
             int8_t configOpt = SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME;
             sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION,&configOpt, &configLen,(uint8_t *)(&dateTime));

             printf("Day %d,Mon %d,Year %d,Hour %,Min %d,Sec %d\n",dateTime.sl_tm_day,dateTime.sl_tm_mon,dateTime.sl_tm_year
                     dateTime.sl_tm_hour,dateTime.sl_tm_min,dateTime.sl_tm_sec);
         \endcode
    */
#if _SL_INCLUDE_FUNC(sl_DevGet)
    int32_t sl_DevGet(const uint8_t DeviceGetId, uint8_t *pOption,uint8_t *pConfigLen, uint8_t *pValues);
#endif


    /*!
        \brief          Set asynchronous event mask

        Mask asynchronous events from the device. Masked events do not
        generate asynchronous messages from the device.
        By default - all events are active

        \param[in]      EventClass          The classification groups that the
                                            mask is referred to. Need to be one of
                                            the following:
                                            - SL_EVENT_CLASS_GLOBAL
                                            - SL_EVENT_CLASS_DEVICE
                                            - SL_EVENT_CLASS_WLAN
                                            - SL_EVENT_CLASS_BSD
                                            - SL_EVENT_CLASS_NETAPP
                                            - SL_EVENT_CLASS_NETCFG
                                            - SL_EVENT_CLASS_FS


        \param[in]      Mask               Event Mask bitmap. Valid mask are (per group):
                                            - SL_EVENT_CLASS_WLAN user events
                                              - SL_WLAN_CONNECT_EVENT
                                              - SL_WLAN_DISCONNECT_EVENT
                                            - SL_EVENT_CLASS_DEVICE user events
                                              - SL_DEVICE_FATAL_ERROR_EVENT
                                            - SL_EVENT_CLASS_BSD user events
                                              - SL_SOCKET_TX_FAILED_EVENT
                                              - SL_SOCKET_ASYNC_EVENT
                                            - SL_EVENT_CLASS_NETAPP user events
                                              - SL_NETAPP_IPV4_IPACQUIRED_EVENT
                                              - SL_NETAPP_IPV6_IPACQUIRED_EVENT

        \return         On success, zero is returned. On error, -1 is returned

        \sa             sl_EventMaskGet

        \note           belongs to \ref ext_api

        \warning
        \par           Example:
        \code

        An example of masking connection/disconnection async events from WLAN class:
                       sl_EventMaskSet(SL_EVENT_CLASS_WLAN, (SL_WLAN_CONNECT_EVENT | SL_WLAN_DISCONNECT_EVENT) );

        \endcode
    */
#if _SL_INCLUDE_FUNC(sl_EventMaskSet)
    int16_t sl_EventMaskSet(const uint8_t EventClass , const uint32_t Mask);
#endif

    /*!
        \brief Get current event mask of the device

        return the events bit mask from the device. In case that event is
        masked, the device is not sending this event.

        \param[in]      EventClass          The classification groups that the
                                            mask is referred to. Need to be one of
                                            the following:
                                            - SL_EVENT_CLASS_GLOBAL
                                            - SL_EVENT_CLASS_DEVICE
                                            - SL_EVENT_CLASS_WLAN
                                            - SL_EVENT_CLASS_BSD
                                            - SL_EVENT_CLASS_NETAPP
                                            - SL_EVENT_CLASS_NETCFG
                                            - SL_EVENT_CLASS_FS

        \param[out]      pMask              Pointer to Mask bitmap where the
                                            value should be stored. Bitmasks are the same as in \ref sl_EventMaskSet

        \return         On success, zero is returned. On error, -1 is returned

        \sa             sl_EventMaskSet

        \note           belongs to \ref ext_api

        \warning
        \par           Example:
        \code

        An example of getting an event mask for WLAN class
        uint32_t maskWlan;
                       sl_StatusGet(SL_EVENT_CLASS_WLAN,&maskWlan);

       \endcode
    */
#if _SL_INCLUDE_FUNC(sl_EventMaskGet)
    int16_t sl_EventMaskGet(const uint8_t EventClass, uint32_t *pMask);
#endif


    /*!
        \brief the simple link task entry

        \Param
        This function must be called from the main loop or from dedicated thread in
        the following cases:
            - Non-Os Platform - should be called from the mail loop
            - Multi Threaded Platform when the user does not implement the external spawn functions -
               should be called from dedicated thread allocated to the simplelink driver.
               In this mode the function never return.

        \return         None

        \sa             sl_Stop

        \note           belongs to \ref basic_api

        \warning        This function must be called from a thread that is start running before
                        any call to other simple link API
    */
#if _SL_INCLUDE_FUNC(sl_Task)
    void sl_Task(void);
#endif


    /*!
        \brief Setting the internal uart mode

        \param[in]      pUartParams          Pointer to the uart configuration parameter set:
                                             baudrate     - up to 711 Kbps
                                             flow control - enable/disable
                                             comm port    - the comm port number

        \return         On success zero is returned, otherwise - Failed.

        \sa             sl_Stop

        \note           belongs to \ref basic_api

        \warning        This function must consider the host uart capability
    */
#ifdef SL_IF_TYPE_UART
#if _SL_INCLUDE_FUNC(sl_UartSetMode)
    int16_t sl_UartSetMode(const SlUartIfParams_t* pUartParams);
#endif
#endif

public:
    
    cc3100_spi               _spi;
    cc3100_driver            _driver;
    cc3100_nonos             _nonos;
    cc3100_wlan              _wlan;
    cc3100_wlan_rx_filters   _wlan_filters;    
    cc3100_netapp            _netapp;
    cc3100_fs                _fs;
    cc3100_netcfg            _netcfg;    
    cc3100_socket            _socket;
    cc3100_flowcont          _flowcont;


protected:
    

};//class

}//namespace mbed_cc3100

/*!

 Close the Doxygen group.
 @}

 */


#endif  /*  __DEVICE_H__ */



