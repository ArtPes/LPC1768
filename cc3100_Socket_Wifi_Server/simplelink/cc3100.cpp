/*
* device.c - CC31xx/CC32xx Host Driver Implementation
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
#include "cc3100_sl_common.h"
#include "cc3100.h"

#include "fPtr_func.h"


namespace mbed_cc3100 {	

    uint32_t  g_PingPacketsRecv;
    uint32_t  g_GatewayIP;
    uint32_t  g_StationIP;
    uint32_t  g_DestinationIP;
    uint32_t  g_BytesReceived; // variable to store the file size 
    uint32_t  g_Status;
    uint8_t   g_buff[MAX_BUFF_SIZE+1];
    int32_t   g_SockID;		


cc3100::cc3100(PinName cc3100_irq, PinName cc3100_nHIB, PinName cc3100_cs, SPI cc3100_spi)
    : _spi(cc3100_irq, cc3100_nHIB, cc3100_cs, cc3100_spi, _driver),
      _driver(_nonos, _netapp, _flowcont, _spi), _nonos(_driver), _wlan(_driver, _wlan_filters),
      _wlan_filters(_driver), _netapp(_driver, _nonos), _fs(_driver), _netcfg(_driver),
      _socket(_driver, _nonos), _flowcont(_driver, _nonos)

       
{

}

cc3100::~cc3100()
{

}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
int32_t cc3100::initializeAppVariables()
{
    
    g_Status = 0;
    g_PingPacketsRecv = 0;
    g_StationIP = 0;
    g_GatewayIP = 0;
    g_DestinationIP = 0;
    g_BytesReceived = 0; /* variable to store the file size */
    g_SockID = 0;
    memset(g_buff, 0, sizeof(g_buff));

    return SUCCESS;
}
    
/*!
    \brief Disconnecting from a WLAN Access point

    This function disconnects from the connected AP

    \param[in]      None

    \return         none

    \note

    \warning        If the WLAN disconnection fails, we will be stuck in this function forever.
*/
int32_t cc3100::disconnectFromAP()
{
    int32_t retVal = -1;

    /*
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = _wlan.sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status,STATUS_BIT_CONNECTION)) { _nonos._SlNonOsMainLoopTask(); }
    }

    return SUCCESS;
}     

/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
int32_t cc3100::configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    uint8_t           val = 1;
    uint8_t           configOpt = 0;
    uint8_t           configLen = 0;
    uint8_t           power = 0;

    int32_t          retVal = -1;
    int32_t          role = -1;

    role = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(role);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != role) {
        if (ROLE_AP == role) {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status,STATUS_BIT_IP_ACQUIRED)) {
                _nonos._SlNonOsMainLoopTask();
            }
        }
        
        /* Switch to STA role and restart */
        retVal = _wlan.sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);
        
        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal) {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }
    
    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (uint8_t *)(&ver));
    ASSERT_ON_ERROR(retVal);
    
    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = _wlan.sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);
    
    /* Remove all profiles */
    retVal = _wlan.sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);
    
    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = _wlan.sl_WlanDisconnect();
    if(0 == retVal) {
        /* Wait */
        while(IS_CONNECTED(g_Status,STATUS_BIT_CONNECTION)) {
            _nonos._SlNonOsMainLoopTask();
        }
    }
    
    /* Enable DHCP client*/
    retVal = _netcfg.sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);
    
    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = _wlan.sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);
    
    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from maximum power - 0 will set maximum power */
    power = 0;
    retVal = _wlan.sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (uint8_t *)&power);
    ASSERT_ON_ERROR(retVal);
    
    /* Set PM policy to normal */
    retVal = _wlan.sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = _netapp.sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = _wlan_filters.sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (uint8_t *)&RxFilterIdMask,
             sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Create UDP socket to communicate with server.

    \param[in]      none

    \return         Socket descriptor for success otherwise negative

    \warning
*/
int32_t cc3100::createUDPConnection()
{
    int32_t sd = 0;

    sd = _socket.sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, IPPROTO_UDP);
    if( sd < 0 )
    {
        printf("Error creating socket\n\r\n\r");
    }

    return sd;
}	
/*!
    \brief Create connection with server.

    This function opens a socket and create the endpoint communication with server

    \param[in]      DestinationIP - IP address of the server

    \return         socket id for success and negative for error
*/
int32_t cc3100::createConnection(uint32_t DestinationIP)
{
    SlSockAddrIn_t  Addr = {0};
    int32_t           Status = 0;
    int32_t           AddrSize = 0;
    int32_t           SockID = 0;

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = _socket.sl_Htons(80);
    Addr.sin_addr.s_addr = _socket.sl_Htonl(DestinationIP);

    AddrSize = sizeof(SlSockAddrIn_t);

    SockID = _socket.sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    ASSERT_ON_ERROR(SockID);

    Status = _socket.sl_Connect(SockID, ( SlSockAddr_t *)&Addr, AddrSize);
    if (Status < 0)
    {
        _socket.sl_Close(SockID);
        ASSERT_ON_ERROR(Status);
    }

    return SockID;
}

/*!
    \brief Convert hex to decimal base

    \param[in]      ptr - pointer to string containing number in hex

    \return         number in decimal base

*/
int32_t cc3100::hexToi(unsigned char *ptr)
{
    uint32_t result = 0;
    uint32_t len = 0;

    int32_t  idx = -1;

    len = strlen((const char*) ptr);

    /* convert characters to upper case */
    for(idx = 0; ptr[idx] != '\0'; ++idx)
    {
        if( (ptr[idx] >= 'a') &&
            (ptr[idx] <= 'f') )
        {
            ptr[idx] -= 32;         /* Change case - ASCII 'a' = 97, 'A' = 65 => 97-65 = 32 */
        }
    }

    for(idx = 0; ptr[idx] != '\0'; ++idx)
    {
        if(ptr[idx] >= '0' && ptr[idx] <= '9')
        {
            /* Converting '0' to '9' to their decimal value */
            result += (ptr[idx] - '0') * (1 << (4 * (len - 1 - idx)));
        }
        else if(ptr[idx] >= 'A' && ptr[idx] <= 'F')
        {
            /* Converting hex 'A' to 'F' to their decimal value */
            result += (ptr[idx] - 55) * (1 << (4 * (len -1 - idx))); /* .i.e. 'A' - 55 = 10, 'F' - 55 = 15 */
        }
        else
        {
            ASSERT_ON_ERROR(INVALID_HEX_STRING);
        }
    }

    return result;
}

/*!
    \brief Calculate the file chunk size

    \param[in]      len - pointer to length of the data in the buffer
    \param[in]      p_Buff - pointer to pointer of buffer containing data
    \param[out]     chunk_size - pointer to variable containing chunk size

    \return         0 for success, -ve for error

*/
int32_t cc3100::getChunkSize(int32_t *len, uint8_t **p_Buff, uint32_t *chunk_size)
{
    int32_t   idx = -1;
    unsigned char   lenBuff[10];

    idx = 0;
    memset(lenBuff, 0, sizeof(lenBuff));
    while(*len >= 0 && **p_Buff != 13) /* check for <CR> */
    {
        if(0 == *len)
        {
            memset(g_buff, 0, sizeof(g_buff));
            *len = _socket.sl_Recv(g_SockID, &g_buff[0], MAX_BUFF_SIZE, 0);
            if(*len <= 0)
                ASSERT_ON_ERROR(TCP_RECV_ERROR);

            *p_Buff = g_buff;
        }

        lenBuff[idx] = **p_Buff;
        idx++;
        (*p_Buff)++;
        (*len)--;
    }

    (*p_Buff) += 2; /* skip <CR><LF> */
    (*len) -= 2;
    *chunk_size = hexToi(lenBuff);

    return SUCCESS;
}

/*!
    \brief Obtain the file from the server

    This function requests the file from the server and save it on serial flash.
    To request a different file for different user needs to modify the
    PREFIX_BUFFER and POST_BUFFER macros.

    \param[in]      None

    \return         0 for success and negative for error

*/
/*
int32_t cc3100::getFile()
{
    uint32_t Token = 0;
    uint32_t recv_size = 0;
    uint8_t *pBuff = 0;
    uint8_t eof_detected = 0;
    uint8_t isChunked = 0;

    int32_t transfer_len = -1;
    int32_t retVal = -1;
    int32_t fileHandle = -1;

    memset(g_buff, 0, sizeof(g_buff));

    //Puts together the HTTP GET string.
    strcpy((char*)g_buff, PREFIX_BUFFER);
    strcat((char*)g_buff, POST_BUFFER);

    //Send the HTTP GET string to the opened TCP/IP socket.
    transfer_len = _socket.sl_Send(g_SockID, g_buff, strlen((const char*)g_buff), 0);

    if (transfer_len < 0)
    {
        // error 
        printf(" Socket Send Error\r\n");
        ASSERT_ON_ERROR(TCP_SEND_ERROR);
    }

    memset(g_buff, 0, sizeof(g_buff));

    //get the reply from the server in buffer.
    transfer_len = _socket.sl_Recv(g_SockID, &g_buff[0], MAX_BUFF_SIZE, 0);

    if(transfer_len <= 0)
        ASSERT_ON_ERROR(TCP_RECV_ERROR);

    // Check for 404 return code 
    if(strstr((const char*)g_buff, HTTP_FILE_NOT_FOUND) != 0)
    {
        printf(" File not found, check the file and try again\r\n");
        ASSERT_ON_ERROR(FILE_NOT_FOUND_ERROR);
    }

    // if not "200 OK" return error 
    if(strstr((const char*)g_buff, HTTP_STATUS_OK) == 0)
    {
        printf(" Error during downloading the file\r\n");
        ASSERT_ON_ERROR(INVALID_SERVER_RESPONSE);
    }

    // check if content length is transferred with headers 
    pBuff = (uint8_t *)strstr((const char*)g_buff, HTTP_CONTENT_LENGTH);
    if(pBuff != 0)
    {
        // not supported 
        printf(" Server response format is not supported\r\n");
        ASSERT_ON_ERROR(FORMAT_NOT_SUPPORTED);
    }

    // Check if data is chunked 
    pBuff = (uint8_t *)strstr((const char*)g_buff, HTTP_TRANSFER_ENCODING);
    if(pBuff != 0)
    {
        pBuff += strlen(HTTP_TRANSFER_ENCODING);
        while(*pBuff == SPACE)
            pBuff++;

        if(memcmp(pBuff, HTTP_ENCODING_CHUNKED, strlen(HTTP_ENCODING_CHUNKED)) == 0)
        {
            recv_size = 0;
            isChunked = 1;
        }
    }
    else
    {
        // Check if connection will be closed by after sending data
         // In this method the content length is not received and end of
         // connection marks the end of data 
        pBuff = (uint8_t *)strstr((const char*)g_buff, HTTP_CONNECTION);
        if(pBuff != 0)
        {
            pBuff += strlen(HTTP_CONNECTION);
            while(*pBuff == SPACE)
                pBuff++;

            if(memcmp(pBuff, HTTP_ENCODING_CHUNKED, strlen(HTTP_CONNECTION_CLOSE)) == 0)
            {
                // not supported 
                printf(" Server response format is not supported\r\n");
                ASSERT_ON_ERROR(FORMAT_NOT_SUPPORTED);
            }
        }
    }

    // "\r\n\r\n" marks the end of headers 
    pBuff = (uint8_t *)strstr((const char*)g_buff, HTTP_END_OF_HEADER);
    if(pBuff == 0)
    {
        printf(" Invalid response\r\n");
        ASSERT_ON_ERROR(INVALID_SERVER_RESPONSE);
    }
    // Increment by 4 to skip "\r\n\r\n" 
    pBuff += 4;

    // Adjust buffer data length for header size 
    transfer_len -= (pBuff - g_buff);

    // If data in chunked format, calculate the chunk size 
    if(isChunked == 1)
    {
        retVal = getChunkSize(&transfer_len, &pBuff, &recv_size);
        if(retVal < 0)
        {
            // Error 
            printf(" Problem with connection to server\r\n");
            return retVal;
        }
    }

    // Open file to save the downloaded file 
    retVal = _fs.sl_FsOpen((uint8_t *)FILE_NAME,
                       FS_MODE_OPEN_WRITE, &Token, &fileHandle);
    if(retVal < 0)
    {
        // File Doesn't exit create a new of 45 KB file 
        retVal = _fs.sl_FsOpen((uint8_t *)FILE_NAME,
                           _fs.FS_MODE_OPEN_CREATE(SIZE_45K,_FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
                           &Token, &fileHandle);
        if(retVal < 0)
        {
            printf(" Error during opening the file\r\n");
            return retVal;
        }
    }

    while (0 < transfer_len)
    {
        // For chunked data recv_size contains the chunk size to be received
         // while the transfer_len contains the data in the buffer 
        if(recv_size <= transfer_len)
        {
            // write the recv_size 
            retVal = _fs.sl_FsWrite(fileHandle, g_BytesReceived,
                    (uint8_t *)pBuff, recv_size);
            if(retVal < recv_size)
            {
                // Close file without saving 
                retVal = _fs.sl_FsClose(fileHandle, 0, (unsigned char*)"A", 1);
                printf(" Error during writing the file\r\n");
                return FILE_WRITE_ERROR;
            }
            transfer_len -= recv_size;
            g_BytesReceived +=recv_size;
            pBuff += recv_size;
            recv_size = 0;

            if(isChunked == 1)
            {
                // if data in chunked format calculate next chunk size 
                pBuff += 2; // 2 bytes for <CR> <LF> 
                transfer_len -= 2;

                if(getChunkSize(&transfer_len, &pBuff, &recv_size) < 0)
                {
                    // Error 
                    break;
                }

                // if next chunk size is zero we have received the complete file 
                if(recv_size == 0)
                {
                    eof_detected = 1;
                    break;
                }

                if(recv_size < transfer_len)
                {
                    // Code will enter this section if the new chunk size is less then
                     // then the transfer size. This will the last chunk of file received
                     
                    retVal = _fs.sl_FsWrite(fileHandle, g_BytesReceived,
                            (uint8_t *)pBuff, recv_size);
                    if(retVal < recv_size)
                    {
                        // Close file without saving 
                        retVal = _fs.sl_FsClose(fileHandle, 0, (unsigned char*)"A", 1);
                        printf(" Error during writing the file\r\n");
                        return FILE_WRITE_ERROR;
                    }
                    transfer_len -= recv_size;
                    g_BytesReceived +=recv_size;
                    pBuff += recv_size;
                    recv_size = 0;

                    pBuff += 2; // 2bytes for <CR> <LF> 
                    transfer_len -= 2;

                    // Calculate the next chunk size, should be zero 
                    if(getChunkSize(&transfer_len, &pBuff, &recv_size) < 0)
                    {
                        // Error 
                        break;
                    }

                    // if next chunk size is non zero error 
                    if(recv_size != 0)
                    {
                        // Error 
                        break;
                    }
                    eof_detected = 1;
                    break;
                }
                else
                {
                    // write data on the file 
                    retVal = _fs.sl_FsWrite(fileHandle, g_BytesReceived,
                            (uint8_t *)pBuff, transfer_len);
                    if(retVal < transfer_len)
                    {
                        // Close file without saving 
                        retVal = _fs.sl_FsClose(fileHandle, 0, (unsigned char*)"A", 1);
                        printf(" Error during writing the file\r\n");
                        ASSERT_ON_ERROR(FILE_WRITE_ERROR);
                    }
                    recv_size -= transfer_len;
                    g_BytesReceived +=transfer_len;
                }
            }
            // complete file received exit 
            if(recv_size == 0)
            {
                eof_detected = 1;
                break;
            }
        }
        else
        {
            // write data on the file 
            retVal = _fs.sl_FsWrite(fileHandle, g_BytesReceived,
                                 (uint8_t *)pBuff, transfer_len);
            if (retVal < 0)
            {
                // Close file without saving 
                retVal = _fs.sl_FsClose(fileHandle, 0, (unsigned char*)"A", 1);
                printf(" Error during writing the file\r\n");
                ASSERT_ON_ERROR(FILE_WRITE_ERROR);
            }
            g_BytesReceived +=transfer_len;
            recv_size -= transfer_len;
        }

        memset(g_buff, 0, sizeof(g_buff));

        transfer_len = _socket.sl_Recv(g_SockID, &g_buff[0], MAX_BUFF_SIZE, 0);
        if(transfer_len <= 0)
            ASSERT_ON_ERROR(TCP_RECV_ERROR);

        pBuff = g_buff;
    }

    // If user file has checksum which can be used to verify the temporary
     // file then file should be verified
     // In case of invalid file (FILE_NAME) should be closed without saving to
     // recover the previous version of file 
    if(0 > transfer_len || eof_detected == 0)
    {
        // Close file without saving 
        retVal = _fs.sl_FsClose(fileHandle, 0, (unsigned char*)"A", 1);
        printf(" Error While File Download\r\n");
        ASSERT_ON_ERROR(INVALID_FILE);
    }
    else
    {
        // Save and close file 
        retVal = _fs.sl_FsClose(fileHandle, 0, 0, 0);
        ASSERT_ON_ERROR(retVal);
    }

    return SUCCESS;;
}
*/

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
int32_t cc3100::establishConnectionWithAP()
{
    
    SlSecParams_t secParams = {0};
    int32_t retVal = 0;

    secParams.Key = (signed char *)PASSKEY;
    secParams.KeyLen = strlen(PASSKEY);
    secParams.Type = SEC_TYPE;

    retVal = _wlan.sl_WlanConnect((signed char *)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);
    
    /* Wait */
    while((!IS_CONNECTED(g_Status,STATUS_BIT_CONNECTION)) || (!IS_IP_ACQUIRED(g_Status,STATUS_BIT_IP_ACQUIRED))) { _nonos._SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief This function checks the LAN connection by pinging the AP's gateway

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
int32_t cc3100::checkLanConnection()
{
    SlPingStartCommand_t pingParams = {0};
    SlPingReport_t pingReport = {0};

    int32_t retVal = -1;

    CLR_STATUS_BIT(g_Status, STATUS_BIT_PING_DONE);
    g_PingPacketsRecv = 0;

    /* Set the ping parameters */
    pingParams.PingIntervalTime = PING_INTERVAL;
    pingParams.PingSize = PING_PKT_SIZE;
    pingParams.PingRequestTimeout = PING_TIMEOUT;
    pingParams.TotalNumberOfAttempts = PING_ATTEMPTS;
    pingParams.Flags = 0;
    pingParams.Ip = g_GatewayIP;

    /* Check for LAN connection */
    retVal = _netapp.sl_NetAppPingStart( (SlPingStartCommand_t*)&pingParams, SL_AF_INET,
                                 (SlPingReport_t*)&pingReport, SimpleLinkPingReport);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while(!IS_PING_DONE(g_Status,STATUS_BIT_PING_DONE)) { _nonos._SlNonOsMainLoopTask(); }

    if(0 == g_PingPacketsRecv)
    {
        /* Problem with LAN connection */
        ASSERT_ON_ERROR(LAN_CONNECTION_FAILED);
    }

    /* LAN connection is successful */
    return SUCCESS;
}

/*!
    \brief This function checks the internet connection by pinging
           the external-host (HOST_NAME)

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
int32_t cc3100::checkInternetConnection()
{
    SlPingStartCommand_t pingParams = {0};
    SlPingReport_t pingReport = {0};

    uint32_t ipAddr = 0;

    int32_t retVal = -1;

    CLR_STATUS_BIT(g_Status, STATUS_BIT_PING_DONE);
    g_PingPacketsRecv = 0;

    /* Set the ping parameters */
    pingParams.PingIntervalTime = PING_INTERVAL;
    pingParams.PingSize = PING_PKT_SIZE;
    pingParams.PingRequestTimeout = PING_TIMEOUT;
    pingParams.TotalNumberOfAttempts = PING_ATTEMPTS;
    pingParams.Flags = 0;
    pingParams.Ip = g_GatewayIP;

    /* Check for Internet connection */
    retVal = _netapp.sl_NetAppDnsGetHostByName((unsigned char *)HOST_NAME, strlen(HOST_NAME), &ipAddr, SL_AF_INET);
    ASSERT_ON_ERROR(retVal);

    /* Replace the ping address to match HOST_NAME's IP address */
    pingParams.Ip = ipAddr;

    /* Try to ping HOST_NAME */
    retVal = _netapp.sl_NetAppPingStart( (SlPingStartCommand_t*)&pingParams, SL_AF_INET,
                                 (SlPingReport_t*)&pingReport, SimpleLinkPingReport);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while(!IS_PING_DONE(g_Status,STATUS_BIT_PING_DONE)) { _nonos._SlNonOsMainLoopTask(); }

    if (0 == g_PingPacketsRecv)
    {
        /* Problem with internet connection*/
        ASSERT_ON_ERROR(INTERNET_CONNECTION_FAILED);
    }

    /* Internet connection is successful */
    return SUCCESS;
}

const int8_t StartResponseLUT[8] = 
{
    ROLE_UNKNOWN_ERR,
    ROLE_STA,
    ROLE_STA_ERR,
    ROLE_AP,
    ROLE_AP_ERR,
    ROLE_P2P,
    ROLE_P2P_ERR,
    ROLE_UNKNOWN_ERR    
};
    
/*****************************************************************************/
/* Internal functions                                                        */
/*****************************************************************************/

int16_t cc3100::_sl_GetStartResponseConvert(uint32_t Status)
{
    return (int16_t)StartResponseLUT[Status & 0x7];
    
}

/*****************************************************************************/
/* API Functions                                                             */
/*****************************************************************************/

bool cc3100::IS_PING_DONE(uint32_t status_variable,const uint32_t bit){
	
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){
	    	return TRUE;
        }else{
        	return FALSE;
        }
}
        
bool cc3100::IS_CONNECTED(uint32_t status_variable,const uint32_t bit){
	
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){
	    	return TRUE;
        }else{
        	return FALSE;
        }
}
        
bool cc3100::IS_STA_CONNECTED(uint32_t status_variable,const uint32_t bit){
	
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){
	    	return TRUE;
        }else{
        	return FALSE;
        }
} 
       
bool cc3100::IS_IP_ACQUIRED(uint32_t status_variable,const uint32_t bit){
	
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){
	    	return TRUE;
        }else{
        	return FALSE;
        }
}
        
bool cc3100::IS_IP_LEASED(uint32_t status_variable,const uint32_t bit){
	
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){
	    	return TRUE;
        }else{
        	return FALSE;
        }
}
        
bool cc3100::IS_CONNECTION_FAILED(uint32_t status_variable,const uint32_t bit){
	    
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){
	    	return TRUE;
        }else{
        	return FALSE;
        }
}
        
bool cc3100::IS_P2P_NEG_REQ_RECEIVED(uint32_t status_variable,const uint32_t bit){
	
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){
	    	return TRUE;
        }else{
        	return FALSE;
        }
}
        
bool cc3100::IS_SMARTCONFIG_DONE(uint32_t status_variable,const uint32_t bit){
	
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){
	    	return TRUE;
        }else{
        	return FALSE;
        }
}
        
bool cc3100::IS_SMARTCONFIG_STOPPED(uint32_t status_variable,const uint32_t bit){
	
	    g_Status = status_variable;
	    
	    if(0 != (g_Status & ((uint32_t)1L<<(bit)))){	    	
	    	return TRUE;
        }else{
        	return FALSE;       	
        }        
}

void cc3100::CLR_STATUS_BIT(uint32_t status_variable, const uint32_t bit){
	    
	    g_Status = status_variable;
	    g_Status &= ~((uint32_t)1L<<(bit));
	    
}

void cc3100::SET_STATUS_BIT(uint32_t status_variable, const uint32_t bit){

	    g_Status = status_variable;
	    g_Status |= ((uint32_t)1L<<(bit));
	    
}
	    
/*****************************************************************************/
/* sl_Task                                                                   */
/*****************************************************************************/
#if _SL_INCLUDE_FUNC(sl_Task)
void cc3100::sl_Task(void)
{
#ifdef _SlTaskEntry
    _nonos._SlNonOsMainLoopTask;
    
#endif
}
#endif

/*****************************************************************************/
/* sl_Start                                                                  */
/*****************************************************************************/
#if _SL_INCLUDE_FUNC(sl_Start)
int16_t cc3100::sl_Start(const void* pIfHdl, int8_t*  pDevName, const P_INIT_CALLBACK pInitCallBack)
{
    int16_t ObjIdx = MAX_CONCURRENT_ACTIONS;
    InitComplete_t  AsyncRsp;

    /* Perform any preprocessing before enable networking services */
    sl_DeviceEnablePreamble();//stub only
    
    /* ControlBlock init */
    _driver._SlDrvDriverCBInit();
    
    /* open the interface: usually SPI or UART */
    if (NULL == pIfHdl) 
    {
        g_pCB->FD = _spi.spi_Open((int8_t *)pDevName, 0);
    } 
    else 
    {
        g_pCB->FD = (_SlFd_t)pIfHdl;
    }
       
    ObjIdx = _driver._SlDrvProtectAsyncRespSetting((uint8_t *)&AsyncRsp, START_STOP_ID, SL_MAX_SOCKETS);
    
    if (MAX_CONCURRENT_ACTIONS == ObjIdx) 
    {
        printf("SL_POOL_IS_EMPTY\r\n");
        return SL_POOL_IS_EMPTY;
    }

    if( g_pCB->FD >= (_SlFd_t)0) {
        _spi.CC3100_disable();
        
        g_pCB->pInitCallback = pInitCallBack;
            
        _spi.CC3100_enable();

        if (NULL == pInitCallBack) {
            
            _driver._SlDrvSyncObjWaitForever(&g_pCB->ObjPool[ObjIdx].SyncObj);
            
            /*release Pool Object*/           
            _driver._SlDrvReleasePoolObj(g_pCB->FunctionParams.AsyncExt.ActionIndex);            
            return _sl_GetStartResponseConvert(AsyncRsp.Status);
        }
        else
        {
            return SL_RET_CODE_OK;    
        }
    }
    
    return SL_BAD_INTERFACE;


}
#endif

/***************************************************************************
_sl_HandleAsync_InitComplete - handles init complete signalling to
a waiting object
****************************************************************************/
void cc3100::_sl_HandleAsync_InitComplete(void *pVoidBuf)
{

    InitComplete_t *pMsgArgs = (InitComplete_t *)_SL_RESP_ARGS_START(pVoidBuf);

    _driver._SlDrvProtectionObjLockWaitForever();

    if(g_pCB->pInitCallback) {
        g_pCB->pInitCallback(_sl_GetStartResponseConvert(pMsgArgs->Status));
    } else {
        memcpy(g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].pRespArgs, pMsgArgs, sizeof(InitComplete_t));        
        _driver._SlDrvSyncObjSignal(&g_pCB->ObjPool[g_pCB->FunctionParams.AsyncExt.ActionIndex].SyncObj);
    }
    _driver._SlDrvProtectionObjUnLock();

    if(g_pCB->pInitCallback) {
        _driver._SlDrvReleasePoolObj(g_pCB->FunctionParams.AsyncExt.ActionIndex);
    }

}

/*****************************************************************************
sl_stop
******************************************************************************/
typedef union {
    _DevStopCommand_t  Cmd;
    _BasicResponse_t   Rsp;
} _SlStopMsg_u;

const _SlCmdCtrl_t _SlStopCmdCtrl = {
    SL_OPCODE_DEVICE_STOP_COMMAND,
    sizeof(_DevStopCommand_t),
    sizeof(_BasicResponse_t)
};

#if _SL_INCLUDE_FUNC(sl_Stop)
int16_t cc3100::sl_Stop(const uint16_t timeout)
{
    int16_t RetVal=0;
    _SlStopMsg_u      Msg;
    _BasicResponse_t  AsyncRsp;
    int8_t ObjIdx = MAX_CONCURRENT_ACTIONS;
    /* if timeout is 0 the shutdown is forced immediately */
    if( 0 == timeout ) {
        _spi.registerInterruptHandler(NULL, NULL);
        _spi.CC3100_disable();
        RetVal = _spi.spi_Close(g_pCB->FD);

    } else {
        /* let the device make the shutdown using the defined timeout */
        Msg.Cmd.Timeout = timeout;
        
        
        
        
        
        ObjIdx = _driver._SlDrvProtectAsyncRespSetting((uint8_t *)&AsyncRsp, START_STOP_ID, SL_MAX_SOCKETS);
      if (MAX_CONCURRENT_ACTIONS == ObjIdx)
      {
          return SL_POOL_IS_EMPTY;
      }

      VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlStopCmdCtrl, &Msg, NULL));

      if(SL_OS_RET_CODE_OK == (int16_t)Msg.Rsp.status)
      {
         _driver._SlDrvSyncObjWaitForever(&g_pCB->ObjPool[ObjIdx].SyncObj);
         Msg.Rsp.status = AsyncRsp.status;
         RetVal = Msg.Rsp.status;
      }

      _driver._SlDrvReleasePoolObj((uint8_t)ObjIdx);

      _spi.registerInterruptHandler(NULL, NULL);
      _spi.CC3100_disable();
      _spi.spi_Close(g_pCB->FD);
    }
    _driver._SlDrvDriverCBDeinit();

    return RetVal;
}
#endif


/*****************************************************************************
sl_EventMaskSet
*****************************************************************************/
typedef union {
    _DevMaskEventSetCommand_t	    Cmd;
    _BasicResponse_t	            Rsp;
} _SlEventMaskSetMsg_u;

#if _SL_INCLUDE_FUNC(sl_EventMaskSet)
const _SlCmdCtrl_t _SlEventMaskSetCmdCtrl = {
    SL_OPCODE_DEVICE_EVENTMASKSET,
    sizeof(_DevMaskEventSetCommand_t),
    sizeof(_BasicResponse_t)
};

int16_t cc3100::sl_EventMaskSet(uint8_t EventClass , uint32_t Mask)
{
    _SlEventMaskSetMsg_u Msg;
    Msg.Cmd.group = EventClass;
    Msg.Cmd.mask = Mask;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlEventMaskSetCmdCtrl, &Msg, NULL));

    return (int16_t)Msg.Rsp.status;
}
#endif

/******************************************************************************
sl_EventMaskGet
******************************************************************************/
typedef union {
    _DevMaskEventGetCommand_t	    Cmd;
    _DevMaskEventGetResponse_t      Rsp;
} _SlEventMaskGetMsg_u;

#if _SL_INCLUDE_FUNC(sl_EventMaskGet)
const _SlCmdCtrl_t _SlEventMaskGetCmdCtrl = {
    SL_OPCODE_DEVICE_EVENTMASKGET,
    sizeof(_DevMaskEventGetCommand_t),
    sizeof(_DevMaskEventGetResponse_t)
};

int16_t cc3100::sl_EventMaskGet(uint8_t EventClass, uint32_t *pMask)
{
    _SlEventMaskGetMsg_u Msg;

    Msg.Cmd.group = EventClass;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlEventMaskGetCmdCtrl, &Msg, NULL));

    *pMask = Msg.Rsp.mask;
    return SL_RET_CODE_OK;
}
#endif

/******************************************************************************
sl_DevGet
******************************************************************************/

typedef union {
    _DeviceSetGet_t	    Cmd;
    _DeviceSetGet_t	    Rsp;
} _SlDeviceMsgGet_u;

#if _SL_INCLUDE_FUNC(sl_DevGet)
const _SlCmdCtrl_t _SlDeviceGetCmdCtrl = {
    SL_OPCODE_DEVICE_DEVICEGET,
    sizeof(_DeviceSetGet_t),
    sizeof(_DeviceSetGet_t)
};

int32_t cc3100::sl_DevGet(uint8_t DeviceGetId, uint8_t *pOption,uint8_t *pConfigLen, uint8_t *pValues)
{
    _SlDeviceMsgGet_u         Msg;
    _SlCmdExt_t               CmdExt;
    
    if (*pConfigLen == 0) {
        return SL_EZEROLEN;
    }
   
    if( pOption ) {
        _driver._SlDrvResetCmdExt(&CmdExt);
        CmdExt.RxPayloadLen = *pConfigLen;
        
        CmdExt.pRxPayload = (uint8_t *)pValues;
        

        Msg.Cmd.DeviceSetId = DeviceGetId;

        Msg.Cmd.Option   = (uint16_t)*pOption;

        VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlDeviceGetCmdCtrl, &Msg, &CmdExt));
        
        if( pOption ) {
            *pOption = (uint8_t)Msg.Rsp.Option;
        }
        
        if (CmdExt.RxPayloadLen < CmdExt.ActualRxPayloadLen) {
            *pConfigLen = (uint8_t)CmdExt.RxPayloadLen;
            return SL_ESMALLBUF;
        } else {
            *pConfigLen = (uint8_t)CmdExt.ActualRxPayloadLen;
        }
        
        return (int16_t)Msg.Rsp.Status;
    } else {
        return -1;
    }
}
#endif

/******************************************************************************
sl_DevSet
******************************************************************************/
typedef union {
    _DeviceSetGet_t    Cmd;
    _BasicResponse_t   Rsp;
} _SlDeviceMsgSet_u;

#if _SL_INCLUDE_FUNC(sl_DevSet)
const _SlCmdCtrl_t _SlDeviceSetCmdCtrl = {
    SL_OPCODE_DEVICE_DEVICESET,
    sizeof(_DeviceSetGet_t),
    sizeof(_BasicResponse_t)
};

int32_t cc3100::sl_DevSet(const uint8_t DeviceSetId, const uint8_t Option, const uint8_t ConfigLen, const uint8_t *pValues)
{
    _SlDeviceMsgSet_u         Msg;
    _SlCmdExt_t               CmdExt;
    
    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.TxPayloadLen = (ConfigLen+3) & (~3);
    
    CmdExt.pTxPayload = (uint8_t *)pValues;
    


    Msg.Cmd.DeviceSetId    = DeviceSetId;
    Msg.Cmd.ConfigLen   = ConfigLen;
    Msg.Cmd.Option   = Option;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlDeviceSetCmdCtrl, &Msg, &CmdExt));

    return (int16_t)Msg.Rsp.status;
}
#endif

/******************************************************************************
sl_UartSetMode
******************************************************************************/
#ifdef SL_IF_TYPE_UART
typedef union {
    _DevUartSetModeCommand_t	  Cmd;
    _DevUartSetModeResponse_t     Rsp;
} _SlUartSetModeMsg_u;

#if _SL_INCLUDE_FUNC(sl_UartSetMode)
const _SlCmdCtrl_t _SlUartSetModeCmdCtrl = {
    SL_OPCODE_DEVICE_SETUARTMODECOMMAND,
    sizeof(_DevUartSetModeCommand_t),
    sizeof(_DevUartSetModeResponse_t)
};

int16_t cc3100::sl_UartSetMode(const SlUartIfParams_t* pUartParams)
{
    _SlUartSetModeMsg_u Msg;
    uint32_t magicCode = 0xFFFFFFFF;

    Msg.Cmd.BaudRate = pUartParams->BaudRate;
    Msg.Cmd.FlowControlEnable = pUartParams->FlowControlEnable;


    VERIFY_RET_OK(_SlDrvCmdOp((_SlCmdCtrl_t *)&_SlUartSetModeCmdCtrl, &Msg, NULL));

    /* cmd response OK, we can continue with the handshake */
    if (SL_RET_CODE_OK == Msg.Rsp.status) {
        sl_IfMaskIntHdlr();

        /* Close the comm port */
        sl_IfClose(g_pCB->FD);

        /* Re-open the comm port */
        sl_IfOpen((void * )pUartParams, UART_IF_OPEN_FLAG_RE_OPEN);
        sl_IfUnMaskIntHdlr();

        /* send the magic code and wait for the response */
        sl_IfWrite(g_pCB->FD, (uint8_t* )&magicCode, 4);

        magicCode = UART_SET_MODE_MAGIC_CODE;
        sl_IfWrite(g_pCB->FD, (uint8_t* )&magicCode, 4);

        /* clear magic code */
        magicCode = 0;

        /* wait (blocking) till the magic code to be returned from device */
        sl_IfRead(g_pCB->FD, (uint8_t* )&magicCode, 4);

        /* check for the received magic code matching */
        if (UART_SET_MODE_MAGIC_CODE != magicCode) {
            _SL_ASSERT(0);
        }
    }

    return (int16_t)Msg.Rsp.status;
}
#endif
#endif

}//namespace


