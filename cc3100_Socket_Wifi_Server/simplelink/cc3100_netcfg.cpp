/*
* netcfg.c - CC31xx/CC32xx Host Driver Implementation
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

#include "cc3100_netcfg.h"

namespace mbed_cc3100 {

cc3100_netcfg::cc3100_netcfg(cc3100_driver &driver)
    : _driver(driver)
{

}

cc3100_netcfg::~cc3100_netcfg()
{

}

/*****************************************************************************/
/* sl_NetCfgSet */
/*****************************************************************************/
typedef union {
    _NetCfgSetGet_t    Cmd;
    _BasicResponse_t   Rsp;
} _SlNetCfgMsgSet_u;

#if _SL_INCLUDE_FUNC(sl_NetCfgSet)
const _SlCmdCtrl_t _SlNetCfgSetCmdCtrl = {
    SL_OPCODE_DEVICE_NETCFG_SET_COMMAND,
    sizeof(_NetCfgSetGet_t),
    sizeof(_BasicResponse_t)
};

int32_t cc3100_netcfg::sl_NetCfgSet(const uint8_t ConfigId, const uint8_t ConfigOpt, const uint8_t ConfigLen, const uint8_t *pValues)
{
    _SlNetCfgMsgSet_u         Msg;
    _SlCmdExt_t               CmdExt;

    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.TxPayloadLen = (ConfigLen+3) & (~3);
    CmdExt.pTxPayload = (uint8_t *)pValues;

    Msg.Cmd.ConfigId    = ConfigId;
    Msg.Cmd.ConfigLen   = ConfigLen;
    Msg.Cmd.ConfigOpt   = ConfigOpt;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlNetCfgSetCmdCtrl, &Msg, &CmdExt));

    return (int16_t)Msg.Rsp.status;
}
#endif

uint32_t cc3100_netcfg::SL_IPV4_VAL(uint8_t add_3,uint8_t add_2,uint8_t add_1,uint8_t add_0){
	
	     return((((uint32_t)add_3 << 24) & 0xFF000000) | (((uint32_t)add_2 << 16) & 0xFF0000) | (((uint32_t)add_1 << 8) & 0xFF00) | ((uint32_t)add_0 & 0xFF) );
}

uint8_t cc3100_netcfg::SL_IPV4_BYTE(uint32_t val,uint8_t index){
	                  
	    return( (val >>= (index*8)) & 0xFF );
}	                  

/*****************************************************************************/
/* sl_NetCfgGet */
/*****************************************************************************/
typedef union {
    _NetCfgSetGet_t	    Cmd;
    _NetCfgSetGet_t	    Rsp;
} _SlNetCfgMsgGet_u;

#if _SL_INCLUDE_FUNC(sl_NetCfgGet)
const _SlCmdCtrl_t _SlNetCfgGetCmdCtrl = {
    SL_OPCODE_DEVICE_NETCFG_GET_COMMAND,
    sizeof(_NetCfgSetGet_t),
    sizeof(_NetCfgSetGet_t)
};

int32_t cc3100_netcfg::sl_NetCfgGet(const uint8_t ConfigId, uint8_t *pConfigOpt,uint8_t *pConfigLen, uint8_t *pValues)
{
    _SlNetCfgMsgGet_u         Msg;
    _SlCmdExt_t               CmdExt;

    if (*pConfigLen == 0) {
        return SL_EZEROLEN;
    }
    
    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.RxPayloadLen = *pConfigLen;
    CmdExt.pRxPayload = (uint8_t *)pValues;
    
    Msg.Cmd.ConfigLen    = *pConfigLen;
    Msg.Cmd.ConfigId     = ConfigId;

    if( pConfigOpt ) {
        Msg.Cmd.ConfigOpt   = (uint16_t)*pConfigOpt;
    }
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlNetCfgGetCmdCtrl, &Msg, &CmdExt));

    if( pConfigOpt ) {
        *pConfigOpt = (uint8_t)Msg.Rsp.ConfigOpt;
    }
    if (CmdExt.RxPayloadLen < CmdExt.ActualRxPayloadLen) {
        *pConfigLen = (uint8_t)CmdExt.RxPayloadLen;
         if( SL_MAC_ADDRESS_GET == ConfigId )
         {
           return SL_RET_CODE_OK;  /* sp fix */
         }
         else
         {
           return SL_ESMALLBUF;
         }
    } 
    else 
    {
        *pConfigLen = (uint8_t)CmdExt.ActualRxPayloadLen;
    }

    return (int16_t)Msg.Rsp.Status;
}
#endif

}//namespace mbed_cc3100
