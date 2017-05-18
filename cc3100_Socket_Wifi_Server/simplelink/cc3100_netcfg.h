/*
 * netcfg.h - CC31xx/CC32xx Host Driver Implementation
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

#ifndef NETCFG_H_
#define NETCFG_H_

namespace mbed_cc3100 {

/*!

    \addtogroup netcfg
    @{

*/


/*****************************************************************************/
/* Macro declarations                                                        */
/*****************************************************************************/
const uint8_t SL_MAC_ADDR_LEN                       =   (6);
const uint8_t IPCONFIG_MODE_DISABLE_IPV4            =   (0);
const uint8_t IPCONFIG_MODE_ENABLE_IPV4             =   (1);

/*****************************************************************************/
/* Structure/Enum declarations                                               */
/*****************************************************************************/
typedef enum {
    SL_MAC_ADDRESS_SET          = 1,
    SL_MAC_ADDRESS_GET          = 2,
    SL_IPV4_STA_P2P_CL_GET_INFO           = 3,
    SL_IPV4_STA_P2P_CL_DHCP_ENABLE        = 4,
    SL_IPV4_STA_P2P_CL_STATIC_ENABLE      = 5,
    SL_IPV4_AP_P2P_GO_GET_INFO            = 6,
    SL_IPV4_AP_P2P_GO_STATIC_ENABLE       = 7,
    SL_SET_HOST_RX_AGGR                   = 8,
    MAX_SETTINGS = 0xFF
} Sl_NetCfg_e;


typedef struct {
    uint32_t  ipV4;
    uint32_t  ipV4Mask;
    uint32_t  ipV4Gateway;
    uint32_t  ipV4DnsServer;
} SlNetCfgIpV4Args_t;

class cc3100_netcfg
{

public:

    cc3100_netcfg(cc3100_driver  &driver);

    ~cc3100_netcfg();


    /*****************************************************************************/
    /* Function prototypes                                                       */
    /*****************************************************************************/

    /*!
        \brief     Internal function for setting network configurations

        \return    On success, zero is returned. On error, -1 is
                   returned

        \param[in] ConfigId   configuration id
        \param[in] ConfigOpt  configurations option
        \param[in] ConfigLen  configurations len
        \param[in] pValues    configurations values

        \sa
        \note
        \warning

        \par   Examples:
        \code
            SL_MAC_ADDRESS_SET:

            Setting MAC address to the Device.
            The new MAC address will override the default MAC address and it be saved in the FileSystem.
            Requires restarting the device for updating this setting.

            uint8_t MAC_Address[6];
            MAC_Address[0] = 0x8;
            MAC_Address[1] = 0x0;
            MAC_Address[2] = 0x28;
            MAC_Address[3] = 0x22;
            MAC_Address[4] = 0x69;
            MAC_Address[5] = 0x31;
            sl_NetCfgSet(SL_MAC_ADDRESS_SET,1,SL_MAC_ADDR_LEN,(uint8_t *)newMacAddress);
            sl_Stop(0);
            sl_Start(NULL,NULL,NULL);
        \endcode

        \code
            SL_IPV4_STA_P2P_CL_STATIC_ENABLE:

            Setting a static IP address to the device working in STA mode or P2P client.
            The IP address will be stored in the FileSystem.
            In order to disable the static IP and get the address assigned from DHCP one should use SL_STA_P2P_CL_IPV4_DHCP_SET

            SlNetCfgIpV4Args_t ipV4;
            ipV4.ipV4          = (uint32_t)SL_IPV4_VAL(10,1,1,201);            // uint32_t IP address
            ipV4.ipV4Mask      = (uint32_t)SL_IPV4_VAL(255,255,255,0);         // uint32_t Subnet mask for this STA/P2P
            ipV4.ipV4Gateway   = (uint32_t)SL_IPV4_VAL(10,1,1,1);              // uint32_t Default gateway address
            ipV4.ipV4DnsServer = (uint32_t)SL_IPV4_VAL(8,16,32,64);            // uint32_t DNS server address

            sl_NetCfgSet(SL_IPV4_STA_P2P_CL_STATIC_ENABLE,IPCONFIG_MODE_ENABLE_IPV4,sizeof(SlNetCfgIpV4Args_t),(uint8_t *)&ipV4);
            sl_Stop(0);
            sl_Start(NULL,NULL,NULL);
        \endcode

        \code
            SL_IPV4_STA_P2P_CL_DHCP_ENABLE:

            Setting IP address by DHCP to FileSystem using WLAN sta mode or P2P client.
                    This should be done once if using Serial Flash.
                    This is the system's default mode for acquiring an IP address after WLAN connection.
            uint8_t val = 1;
            sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,IPCONFIG_MODE_ENABLE_IPV4,1,&val);
            sl_Stop(0);
            sl_Start(NULL,NULL,NULL);
        \endcode

        \code
            SL_IPV4_AP_P2P_GO_STATIC_ENABLE:

            Setting a static IP address to the device working in AP mode or P2P go.
            The IP address will be stored in the FileSystem. Requires restart.

            SlNetCfgIpV4Args_t ipV4;
            ipV4.ipV4          = (uint32_t)SL_IPV4_VAL(10,1,1,201);            // uint32_t IP address
            ipV4.ipV4Mask      = (uint32_t)SL_IPV4_VAL(255,255,255,0);         // uint32_t Subnet mask for this AP/P2P
            ipV4.ipV4Gateway   = (uint32_t)SL_IPV4_VAL(10,1,1,1);              // uint32_t Default gateway address
            ipV4.ipV4DnsServer = (uint32_t)SL_IPV4_VAL(8,16,32,64);            // uint32_t DNS server address

            sl_NetCfgSet(SL_IPV4_AP_P2P_GO_STATIC_ENABLE,IPCONFIG_MODE_ENABLE_IPV4,sizeof(SlNetCfgIpV4Args_t),(uint8_t *)&ipV4);
            sl_Stop(0);
            sl_Start(NULL,NULL,NULL);
        \endcode


    */
#if _SL_INCLUDE_FUNC(sl_NetCfgSet)
    int32_t sl_NetCfgSet(const uint8_t ConfigId, const uint8_t ConfigOpt, const uint8_t ConfigLen, const uint8_t *pValues);
#endif


    /*!
        \brief     Internal function for getting network configurations

        \return    On success, zero is returned. On error, -1 is
                   returned

        \param[in] ConfigId      configuration id

        \param[out] pConfigOpt   Get configurations option

        \param[out] pConfigLen   The length of the allocated memory as input, when the
                                            function complete, the value of this parameter would be
                                 the len that actually read from the device.\n
                                            If the device return length that is longer from the input
                                            value, the function will cut the end of the returned structure
                                            and will return ESMALLBUF

        \param[out] pValues - get configurations values

        \sa
        \note
        \warning
        \par   Examples:
        \code
           SL_MAC_ADDRESS_GET:

           Get the device MAC address.
           The returned MAC address is taken from FileSystem first. If the MAC address was not set by SL_MAC_ADDRESS_SET, the default MAC address
           is retrieved from HW.

           uint8_t macAddressVal[SL_MAC_ADDR_LEN];
           uint8_t macAddressLen = SL_MAC_ADDR_LEN;
           sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(uint8_t *)macAddressVal);

        \endcode

        \code
            SL_IPV4_STA_P2P_CL_GET_INFO:

            Get IP address from WLAN station or P2P client. A DHCP flag is returned to indicate if the IP address is static or from DHCP.

            uint8_t len = sizeof(SlNetCfgIpV4Args_t);
            uint8_t dhcpIsOn = 0;
            SlNetCfgIpV4Args_t ipV4 = {0};
            sl_NetCfgGet(SL_IPV4_STA_P2P_CL_GET_INFO,&dhcpIsOn,&len,(uint8_t *)&ipV4);

            printf("DHCP is %s IP %d.%d.%d.%d MASK %d.%d.%d.%d GW %d.%d.%d.%d DNS %d.%d.%d.%d\n",
                    (dhcpIsOn > 0) ? "ON" : "OFF",
                    SL_IPV4_BYTE(ipV4.ipV4,3),SL_IPV4_BYTE(ipV4.ipV4,2),SL_IPV4_BYTE(ipV4.ipV4,1),SL_IPV4_BYTE(ipV4.ipV4,0),
                    SL_IPV4_BYTE(ipV4.ipV4Mask,3),SL_IPV4_BYTE(ipV4.ipV4Mask,2),SL_IPV4_BYTE(ipV4.ipV4Mask,1),SL_IPV4_BYTE(ipV4.ipV4Mask,0),
                    SL_IPV4_BYTE(ipV4.ipV4Gateway,3),SL_IPV4_BYTE(ipV4.ipV4Gateway,2),SL_IPV4_BYTE(ipV4.ipV4Gateway,1),SL_IPV4_BYTE(ipV4.ipV4Gateway,0),
                    SL_IPV4_BYTE(ipV4.ipV4DnsServer,3),SL_IPV4_BYTE(ipV4.ipV4DnsServer,2),SL_IPV4_BYTE(ipV4.ipV4DnsServer,1),SL_IPV4_BYTE(ipV4.ipV4DnsServer,0));

        \endcode

        \code
            SL_IPV4_AP_P2P_GO_GET_INFO:

            Get static IP address for AP or P2P go.

            uint8_t len = sizeof(SlNetCfgIpV4Args_t);
            uint8_t dhcpIsOn = 0; // this flag is meaningless on AP/P2P go.
            SlNetCfgIpV4Args_t ipV4 = {0};
            sl_NetCfgGet(SL_IPV4_AP_P2P_GO_GET_INFO,&dhcpIsOn,&len,(uint8_t *)&ipV4);

            printf("IP %d.%d.%d.%d MASK %d.%d.%d.%d GW %d.%d.%d.%d DNS %d.%d.%d.%d\n",
                    SL_IPV4_BYTE(ipV4.ipV4,3),SL_IPV4_BYTE(ipV4.ipV4,2),SL_IPV4_BYTE(ipV4.ipV4,1),SL_IPV4_BYTE(ipV4.ipV4,0),
                    SL_IPV4_BYTE(ipV4.ipV4Mask,3),SL_IPV4_BYTE(ipV4.ipV4Mask,2),SL_IPV4_BYTE(ipV4.ipV4Mask,1),SL_IPV4_BYTE(ipV4.ipV4Mask,0),
                    SL_IPV4_BYTE(ipV4.ipV4Gateway,3),SL_IPV4_BYTE(ipV4.ipV4Gateway,2),SL_IPV4_BYTE(ipV4.ipV4Gateway,1),SL_IPV4_BYTE(ipV4.ipV4Gateway,0),
                    SL_IPV4_BYTE(ipV4.ipV4DnsServer,3),SL_IPV4_BYTE(ipV4.ipV4DnsServer,2),SL_IPV4_BYTE(ipV4.ipV4DnsServer,1),SL_IPV4_BYTE(ipV4.ipV4DnsServer,0));

        \endcode


    */
#if _SL_INCLUDE_FUNC(sl_NetCfgGet)
    int32_t sl_NetCfgGet(const uint8_t ConfigId ,uint8_t *pConfigOpt, uint8_t *pConfigLen, uint8_t *pValues);
#endif

uint32_t SL_IPV4_VAL(uint8_t add_3,uint8_t add_2,uint8_t add_1,uint8_t add_0);
uint8_t SL_IPV4_BYTE(uint32_t val,uint8_t index);

private:

    cc3100_driver  &_driver;
    
};//class

}//namespace mbed_cc3100    

    /*!

     Close the Doxygen group.
     @}

     */


#endif    /*  __NETCFG_H__ */


