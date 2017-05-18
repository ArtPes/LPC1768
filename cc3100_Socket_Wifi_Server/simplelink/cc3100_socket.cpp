/*
 * socket.c - CC31xx/CC32xx Host Driver Implementation
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

#include "cc3100_socket.h"

namespace mbed_cc3100 {
	
/*  Note: parsing of family and port in the generic way for all IPV4, IPV6 and EUI48 */
    /*  is possible as _i32 as these parameters are in the same offset and size for these */
    /*  three families. */
#define SL_SOCKET_PAYLOAD_BASE (1350)

const uint8_t _SlPayloadByProtocolLUT[16] = 
{
    (1472 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_UDP_IPV4 */
    (1460 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_TCP_IPV4 */
    (1452 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_UDP_IPV6 */
    (1440 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_TCP_IPV6 */
    (1386 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_TCP_IPV4_SECURE */
    (1386 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_UDP_IPV4_SECURE */
    (1396 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_UDP_IPV6_SECURE */
    (1396 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_TCP_IPV6_SECURE */
    (1476 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_RAW_TRANCEIVER */
    (1514 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_RAW_PACKET */
    (1480 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_RAW_IP4 */
    (1480 - SL_SOCKET_PAYLOAD_BASE), /* SL_SOCKET_PAYLOAD_TYPE_RAW_IP6 */
    (1440 - SL_SOCKET_PAYLOAD_BASE), /* Default */
    (1440 - SL_SOCKET_PAYLOAD_BASE), /* Default */
    (1440 - SL_SOCKET_PAYLOAD_BASE), /* Default */
    (1440 - SL_SOCKET_PAYLOAD_BASE)  /* Default */
};	


cc3100_socket::cc3100_socket(cc3100_driver &driver, cc3100_nonos &nonos)
    : _driver(driver), _nonos(nonos)
{

}

cc3100_socket::~cc3100_socket()
{

}

/*******************************************************************************/
/* Functions                                                                   */
/*******************************************************************************/

/* ******************************************************************************/
/*  _sl_BuildAddress */
/* ******************************************************************************/
void cc3100_socket::_sl_BuildAddress(const SlSockAddr_t *addr, _SocketAddrCommand_u *pCmd)
{
    /* Note: parsing of family and port in the generic way for all IPV4, IPV6 and EUI48
           is possible as long as these parameters are in the same offset and size for these
           three families. */
    pCmd->IpV4.FamilyAndFlags = (addr->sa_family << 4) & 0xF0;
    pCmd->IpV4.port = ((SlSockAddrIn_t *)addr)->sin_port;

    if(SL_AF_INET == addr->sa_family) {
        pCmd->IpV4.address  = ((SlSockAddrIn_t *)addr)->sin_addr.s_addr;
    } else if (SL_AF_INET6_EUI_48 == addr->sa_family ) {
        memcpy( pCmd->IpV6EUI48.address,((SlSockAddrIn6_t *)addr)->sin6_addr._S6_un._S6_u8, 6);
    }
#ifdef SL_SUPPORT_IPV6
    else {
        memcpy(pCmd->IpV6.address, ((sockaddr_in6 *)addr)->sin6_addr._S6_un._S6_u32, 16 );
    }
#endif
}

/* ******************************************************************************/
/*  _sl_TruncatePayloadByProtocol */
/* ******************************************************************************/
uint16_t cc3100_socket::_sl_TruncatePayloadByProtocol(const int16_t sd,const uint16_t length)
{

    uint32_t maxLength;

   maxLength = SL_SOCKET_PAYLOAD_BASE + _SlPayloadByProtocolLUT[((sd & SL_SOCKET_PAYLOAD_TYPE_MASK) >> 4)];
   
   if( length > maxLength )
   {
      return maxLength;
   }
   else
   {
      return length;
   }

}

/*******************************************************************************/
/*  _sl_ParseAddress */
/*******************************************************************************/
#ifndef SL_TINY_EXT
void cc3100_socket::_sl_ParseAddress(_SocketAddrResponse_u    *pRsp, SlSockAddr_t *addr, SlSocklen_t *addrlen)
{
    /*  Note: parsing of family and port in the generic way for all IPV4, IPV6 and EUI48 */
    /*  is possible as long as these parameters are in the same offset and size for these */
    /*  three families. */
    addr->sa_family                 = pRsp->IpV4.family;
    ((SlSockAddrIn_t *)addr)->sin_port = pRsp->IpV4.port;

    *addrlen = (SL_AF_INET == addr->sa_family) ? sizeof(SlSockAddrIn_t) : sizeof(SlSockAddrIn6_t);

    if(SL_AF_INET == addr->sa_family) {
        ((SlSockAddrIn_t *)addr)->sin_addr.s_addr  = pRsp->IpV4.address;
    } else if (SL_AF_INET6_EUI_48 == addr->sa_family ) {
        memcpy(((SlSockAddrIn6_t *)addr)->sin6_addr._S6_un._S6_u8, pRsp->IpV6EUI48.address, 6);
    }
#ifdef SL_SUPPORT_IPV6
    else {
        memcpy(((sockaddr_in6 *)addr)->sin6_addr._S6_un._S6_u32, pRsp->IpV6.address, 16);
    }
#endif
}
#endif

/*******************************************************************************/
/* sl_Socket */
/*******************************************************************************/
typedef union {
    uint32_t                Dummy;
    _SocketCommand_t 	Cmd;
    _SocketResponse_t	Rsp;
} _SlSockSocketMsg_u;

#if _SL_INCLUDE_FUNC(sl_Socket)
const _SlCmdCtrl_t _SlSockSocketCmdCtrl = {
    SL_OPCODE_SOCKET_SOCKET,
    sizeof(_SocketCommand_t),
    sizeof(_SocketResponse_t)
};

int16_t cc3100_socket::sl_Socket(int16_t Domain, int16_t Type, int16_t Protocol)
{
    _SlSockSocketMsg_u  Msg;

    Msg.Cmd.Domain	    = (uint8_t)Domain;
    Msg.Cmd.Type     	= (uint8_t)Type;
    Msg.Cmd.Protocol 	= (uint8_t)Protocol;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlSockSocketCmdCtrl, &Msg, NULL));

    if( Msg.Rsp.statusOrLen < 0 ) {
        return( Msg.Rsp.statusOrLen );
    } else {
        return (int16_t)((uint8_t)Msg.Rsp.sd);
    }
}
#endif

/*******************************************************************************/
/*  sl_Close  */
/*******************************************************************************/
typedef union {
    _CloseCommand_t	    Cmd;
    _SocketResponse_t	Rsp;
} _SlSockCloseMsg_u;

#if _SL_INCLUDE_FUNC(sl_Close)
const _SlCmdCtrl_t _SlSockCloseCmdCtrl = {
    SL_OPCODE_SOCKET_CLOSE,
    sizeof(_CloseCommand_t),
    sizeof(_SocketResponse_t)
};

int16_t cc3100_socket::sl_Close(int16_t sd)
{
    _SlSockCloseMsg_u   Msg;

    Msg.Cmd.sd = (uint8_t)sd;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlSockCloseCmdCtrl, &Msg, NULL));

    return Msg.Rsp.statusOrLen;
}
#endif

/*******************************************************************************/
/*  sl_Bind */
/*******************************************************************************/
typedef union {
    _SocketAddrCommand_u    Cmd;
    _SocketResponse_t	    Rsp;
} _SlSockBindMsg_u;

#if _SL_INCLUDE_FUNC(sl_Bind)
int16_t cc3100_socket::sl_Bind(int16_t sd, const SlSockAddr_t *addr, int16_t addrlen)
{
    _SlSockBindMsg_u    Msg;
    _SlCmdCtrl_t         CmdCtrl = {0, 0, sizeof(_SocketResponse_t)};

    switch(addr->sa_family) {
        case SL_AF_INET :
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_BIND;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv4Command_t);
            break;
#ifndef SL_TINY_EXT             
        case SL_AF_INET6_EUI_48:
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_BIND_V6;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv6EUI48Command_t);
            break;
#ifdef SL_SUPPORT_IPV6
        case AF_INET6:
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_BIND_V6;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv6Command_t);
            break;
#endif
#endif
        case SL_AF_RF   :
        default:
            return SL_RET_CODE_INVALID_INPUT;
    }

    Msg.Cmd.IpV4.lenOrPadding = 0;
    Msg.Cmd.IpV4.sd = (uint8_t)sd;

    _sl_BuildAddress(addr, &Msg.Cmd);

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&CmdCtrl, &Msg, NULL));

    return Msg.Rsp.statusOrLen;
}
#endif

/*******************************************************************************/
/*  sl_Sendto */
/*******************************************************************************/
typedef union {
    _SocketAddrCommand_u    Cmd;
    /*  no response for 'sendto' commands*/
} _SlSendtoMsg_u;

#if _SL_INCLUDE_FUNC(sl_SendTo)
int16_t cc3100_socket::sl_SendTo(int16_t sd, const void *pBuf, int16_t Len, int16_t flags, const SlSockAddr_t *to, SlSocklen_t tolen)
{
    _SlSendtoMsg_u   Msg;
    _SlCmdCtrl_t     CmdCtrl = {0, 0, 0};
    _SlCmdExt_t      CmdExt;
    uint16_t         ChunkLen;
    int16_t          RetVal;

    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.TxPayloadLen = (uint16_t)Len;
    CmdExt.pTxPayload = (uint8_t *)pBuf;

    switch(to->sa_family) {
        case SL_AF_INET:
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_SENDTO;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv4Command_t);
            break;
#ifndef SL_TINY_EXT            
        case SL_AF_INET6_EUI_48:
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_BIND_V6;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv6EUI48Command_t);
            break;
#ifdef SL_SUPPORT_IPV6
        case AF_INET6:
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_SENDTO_V6;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv6Command_t);
            break;
#endif
#endif
        case SL_AF_RF:
        default:
            return SL_RET_CODE_INVALID_INPUT;
    }

    ChunkLen = _sl_TruncatePayloadByProtocol(sd,Len);
    Msg.Cmd.IpV4.lenOrPadding = ChunkLen;
    CmdExt.TxPayloadLen = ChunkLen;

    Msg.Cmd.IpV4.sd = (unsigned char)sd;

    _sl_BuildAddress(to, &Msg.Cmd);
    
    Msg.Cmd.IpV4.FamilyAndFlags |= flags & 0x0F;
    
    do {
        RetVal = _driver._SlDrvDataWriteOp((_SlSd_t)sd, &CmdCtrl, &Msg, &CmdExt);
        
        if(SL_OS_RET_CODE_OK == RetVal) {
            CmdExt.pTxPayload += ChunkLen;
            ChunkLen = (uint16_t)((unsigned char *)pBuf + Len - CmdExt.pTxPayload);
            ChunkLen = _sl_TruncatePayloadByProtocol(sd,ChunkLen);
            CmdExt.TxPayloadLen = ChunkLen;
            Msg.Cmd.IpV4.lenOrPadding = ChunkLen;
        } else {
            return RetVal;
        }
    } while(ChunkLen > 0);

    return (int16_t)Len;
}
#endif

/*******************************************************************************/
/*  sl_Recvfrom */
/*******************************************************************************/
typedef union {
    _sendRecvCommand_t	    Cmd;
    _SocketAddrResponse_u	Rsp;
} _SlRecvfromMsg_u;

const _SlCmdCtrl_t _SlRecvfomCmdCtrl = {
    SL_OPCODE_SOCKET_RECVFROM,
    sizeof(_sendRecvCommand_t),
    sizeof(_SocketAddrResponse_u)
};

#if _SL_INCLUDE_FUNC(sl_RecvFrom)
int16_t cc3100_socket::sl_RecvFrom(int16_t sd, void *buf, int16_t Len, int16_t flags, SlSockAddr_t *from, SlSocklen_t *fromlen)
{
    _SlRecvfromMsg_u    Msg;
    _SlCmdExt_t         CmdExt;
    int16_t                 RetVal;

    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.RxPayloadLen = Len;
    CmdExt.pRxPayload = (uint8_t *)buf;

    Msg.Cmd.sd = (uint8_t)sd;
    Msg.Cmd.StatusOrLen = Len;
    /*  no size truncation in recv path */
    CmdExt.RxPayloadLen = Msg.Cmd.StatusOrLen;
    
    Msg.Cmd.FamilyAndFlags = flags & 0x0F;

    if(sizeof(SlSockAddrIn_t) == *fromlen) {
        Msg.Cmd.FamilyAndFlags |= (SL_AF_INET << 4);
    }
    else if (sizeof(SlSockAddrIn6_t) == *fromlen)
    {
        Msg.Cmd.FamilyAndFlags |= (SL_AF_INET6 << 4);
    } 
    else 
    {
        return SL_RET_CODE_INVALID_INPUT;
    }

    RetVal = _driver._SlDrvDataReadOp((_SlSd_t)sd, (_SlCmdCtrl_t *)&_SlRecvfomCmdCtrl, &Msg, &CmdExt);
    if( RetVal != SL_OS_RET_CODE_OK ) {
        return RetVal;
    }

    RetVal = Msg.Rsp.IpV4.statusOrLen;

    if(RetVal >= 0) {
        VERIFY_PROTOCOL(sd == Msg.Rsp.IpV4.sd);
#if 0
        _sl_ParseAddress(&Msg.Rsp, from, fromlen);
#else
        from->sa_family = Msg.Rsp.IpV4.family;
        if(SL_AF_INET == from->sa_family) {
            ((SlSockAddrIn_t *)from)->sin_port = Msg.Rsp.IpV4.port;
            ((SlSockAddrIn_t *)from)->sin_addr.s_addr = Msg.Rsp.IpV4.address;
            *fromlen = sizeof(SlSockAddrIn_t);
        } else if (SL_AF_INET6_EUI_48 == from->sa_family ) {
            ((SlSockAddrIn6_t *)from)->sin6_port  = Msg.Rsp.IpV6EUI48.port;
            memcpy(((SlSockAddrIn6_t *)from)->sin6_addr._S6_un._S6_u8, Msg.Rsp.IpV6EUI48.address, 6);
        }
#ifdef SL_SUPPORT_IPV6
        else if(AF_INET6 == from->sa_family) {
            VERIFY_PROTOCOL(*fromlen >= sizeof(sockaddr_in6));

            ((sockaddr_in6 *)from)->sin6_port = Msg.Rsp.IpV6.port;
            memcpy(((sockaddr_in6 *)from)->sin6_addr._S6_un._S6_u32, Msg.Rsp.IpV6.address, 16);
            *fromlen = sizeof(sockaddr_in6);
        }
#endif
#endif
    }

    return (int16_t)RetVal;
}
#endif

/*******************************************************************************/
/*  sl_Connect */
/*******************************************************************************/
typedef union {
    _SocketAddrCommand_u    Cmd;
    _SocketResponse_t	    Rsp;
} _SlSockConnectMsg_u;

#if _SL_INCLUDE_FUNC(sl_Connect)
int16_t cc3100_socket::sl_Connect(int16_t sd, const SlSockAddr_t *addr, int16_t addrlen)
{
    _SlSockConnectMsg_u  Msg;
    _SlReturnVal_t       RetVal;
    _SlCmdCtrl_t         CmdCtrl = {0, 0, sizeof(_SocketResponse_t)};
    _SocketResponse_t    AsyncRsp;
    uint8_t ObjIdx = MAX_CONCURRENT_ACTIONS;


    switch(addr->sa_family) {
        case SL_AF_INET :
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_CONNECT;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv4Command_t);
            /* Do nothing - cmd already initialized to this type */
            break;
        case  SL_AF_INET6_EUI_48:
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_CONNECT_V6;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv6EUI48Command_t);
            break;
#ifdef SL_SUPPORT_IPV6
        case AF_INET6:
            CmdCtrl.Opcode = SL_OPCODE_SOCKET_CONNECT_V6;
            CmdCtrl.TxDescLen = sizeof(_SocketAddrIPv6Command_t);
            break;
#endif
        case SL_AF_RF   :
        default:
            return SL_RET_CODE_INVALID_INPUT;
    }

    Msg.Cmd.IpV4.lenOrPadding = 0;
    Msg.Cmd.IpV4.sd = (uint8_t)sd;

    _sl_BuildAddress(addr, &Msg.Cmd);


    ObjIdx = _driver._SlDrvProtectAsyncRespSetting((uint8_t*)&AsyncRsp, CONNECT_ID, sd  & BSD_SOCKET_ID_MASK);

    if (MAX_CONCURRENT_ACTIONS == ObjIdx)
    {
        return SL_POOL_IS_EMPTY;
    }

    /* send the command */
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&CmdCtrl, &Msg, NULL));
    VERIFY_PROTOCOL(Msg.Rsp.sd == sd)

    RetVal = Msg.Rsp.statusOrLen;

    if(SL_RET_CODE_OK == RetVal) {
        /*  wait for async and get Data Read parameters */
        _driver._SlDrvSyncObjWaitForever(&g_pCB->ObjPool[ObjIdx].SyncObj);

        VERIFY_PROTOCOL(AsyncRsp.sd == sd);

        RetVal = AsyncRsp.statusOrLen;
    }
    _driver._SlDrvReleasePoolObj(ObjIdx);
    return RetVal;
}
#endif

/*******************************************************************************/
/*  sl_Send */
/*******************************************************************************/
typedef union {
    _sendRecvCommand_t    Cmd;
    /*  no response for 'sendto' commands*/
} _SlSendMsg_u;

const _SlCmdCtrl_t _SlSendCmdCtrl = {
    SL_OPCODE_SOCKET_SEND,
    sizeof(_sendRecvCommand_t),
    0
};

#if _SL_INCLUDE_FUNC(sl_Send)
int16_t cc3100_socket::sl_Send(int16_t sd, const void *pBuf, int16_t Len, int16_t flags)
{
    _SlSendMsg_u   Msg;
    _SlCmdExt_t    CmdExt;
    uint16_t         ChunkLen;
    int16_t            RetVal;
    uint32_t         tempVal;
    uint8_t  runSingleChunk = FALSE;

    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.TxPayloadLen = Len;
    CmdExt.pTxPayload = (uint8_t *)pBuf;

    /* Only for RAW transceiver type socket, relay the flags parameter in the 2 bytes (4 byte aligned) before the actual payload */
    if ((sd & SL_SOCKET_PAYLOAD_TYPE_MASK) == SL_SOCKET_PAYLOAD_TYPE_RAW_TRANCEIVER) {
        tempVal = flags;
        CmdExt.pRxPayload = (uint8_t *)&tempVal;
        CmdExt.RxPayloadLen = -4; /* mark as Rx data to send */
        runSingleChunk = TRUE;
    } else {
        CmdExt.pRxPayload = NULL;
    }

    ChunkLen = _sl_TruncatePayloadByProtocol(sd,Len);
    CmdExt.TxPayloadLen = ChunkLen;

    Msg.Cmd.StatusOrLen = ChunkLen;
    Msg.Cmd.sd = (uint8_t)sd;
    Msg.Cmd.FamilyAndFlags |= flags & 0x0F;

    do {
        RetVal = _driver._SlDrvDataWriteOp((uint8_t)sd, (_SlCmdCtrl_t *)&_SlSendCmdCtrl, &Msg, &CmdExt);
        if(SL_OS_RET_CODE_OK == RetVal) {
            CmdExt.pTxPayload += ChunkLen;
            ChunkLen = (uint8_t *)pBuf + Len - CmdExt.pTxPayload;
            ChunkLen = _sl_TruncatePayloadByProtocol(sd,ChunkLen);
            CmdExt.TxPayloadLen = ChunkLen;
            Msg.Cmd.StatusOrLen = ChunkLen;
        } else {
            return RetVal;
        }
    } while((ChunkLen > 0) && (runSingleChunk==FALSE));

    return (int16_t)Len;
}
#endif

/*******************************************************************************/
/*  sl_Listen */
/*******************************************************************************/
typedef union {
    _ListenCommand_t    Cmd;
    _BasicResponse_t    Rsp;
} _SlListenMsg_u;

#if _SL_INCLUDE_FUNC(sl_Listen)
const _SlCmdCtrl_t _SlListenCmdCtrl = {
    SL_OPCODE_SOCKET_LISTEN,
    sizeof(_ListenCommand_t),
    sizeof(_BasicResponse_t),
};

int16_t cc3100_socket::sl_Listen(int16_t sd, int16_t backlog)
{
    _SlListenMsg_u  Msg;

    Msg.Cmd.sd = (uint8_t)sd;
    Msg.Cmd.backlog = (uint8_t)backlog;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlListenCmdCtrl, &Msg, NULL));

    return (int16_t)Msg.Rsp.status;
}
#endif

/*******************************************************************************/
/*  sl_Accept */
/*******************************************************************************/
typedef union {
    _AcceptCommand_t    Cmd;
    _SocketResponse_t   Rsp;
} _SlSockAcceptMsg_u;

#if _SL_INCLUDE_FUNC(sl_Accept)
const _SlCmdCtrl_t _SlAcceptCmdCtrl = {
    SL_OPCODE_SOCKET_ACCEPT,
    sizeof(_AcceptCommand_t),
    sizeof(_BasicResponse_t),
};

int16_t cc3100_socket::sl_Accept(int16_t sd, SlSockAddr_t *addr, SlSocklen_t *addrlen)
{
    _SlSockAcceptMsg_u      Msg;
    _SlReturnVal_t          RetVal;
    _SocketAddrResponse_u   AsyncRsp;

    uint8_t ObjIdx = MAX_CONCURRENT_ACTIONS;


    Msg.Cmd.sd = (uint8_t)sd;
    Msg.Cmd.family = (sizeof(SlSockAddrIn_t) == *addrlen) ? SL_AF_INET : SL_AF_INET6;

    ObjIdx = _driver._SlDrvProtectAsyncRespSetting((uint8_t*)&AsyncRsp, ACCEPT_ID, sd  & BSD_SOCKET_ID_MASK );

    if (MAX_CONCURRENT_ACTIONS == ObjIdx)
    {
        return SL_POOL_IS_EMPTY;
    }
    
    /* send the command */
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlAcceptCmdCtrl, &Msg, NULL));
    VERIFY_PROTOCOL(Msg.Rsp.sd == sd);

    RetVal = Msg.Rsp.statusOrLen;

    if(SL_OS_RET_CODE_OK == RetVal) {
        /*  wait for async and get Data Read parameters */
        _driver._SlDrvSyncObjWaitForever(&g_pCB->ObjPool[ObjIdx].SyncObj);

        VERIFY_PROTOCOL(AsyncRsp.IpV4.sd == sd);

        RetVal = AsyncRsp.IpV4.statusOrLen;
        if( (NULL != addr) && (NULL != addrlen) ) {
#if 0 /*  Kept for backup */
            _sl_ParseAddress(&AsyncRsp, addr, addrlen);
#else
            addr->sa_family = AsyncRsp.IpV4.family;

            if(SL_AF_INET == addr->sa_family) {
                if( *addrlen == sizeof( SlSockAddrIn_t ) ) {
                    ((SlSockAddrIn_t *)addr)->sin_port         = AsyncRsp.IpV4.port;
                    ((SlSockAddrIn_t *)addr)->sin_addr.s_addr  = AsyncRsp.IpV4.address;
                } else {
                    *addrlen = 0;
                }
            } else if (SL_AF_INET6_EUI_48 == addr->sa_family ) {
                if( *addrlen == sizeof( SlSockAddrIn6_t ) ) {
                    ((SlSockAddrIn6_t *)addr)->sin6_port                   = AsyncRsp.IpV6EUI48.port    ;
                    /*  will be called from here and from _sl_BuildAddress*/
                    memcpy(((SlSockAddrIn6_t *)addr)->sin6_addr._S6_un._S6_u8, AsyncRsp.IpV6EUI48.address, 6);
                } else {
                    *addrlen = 0;
                }
            }
#ifdef SL_SUPPORT_IPV6
            else {
                if( *addrlen == sizeof( sockaddr_in6 ) ) {
                    ((sockaddr_in6 *)addr)->sin6_port                   = AsyncRsp.IpV6.port    ;
                    memcpy(((sockaddr_in6 *)addr)->sin6_addr._S6_un._S6_u32, AsyncRsp.IpV6.address, 16);
                } else {
                    *addrlen = 0;
                }
            }
#endif
#endif
        }
    }

    _driver._SlDrvReleasePoolObj(ObjIdx);
    return (int16_t)RetVal;
}
#endif


/*******************************************************************************/
/*  sl_Htonl */
/*******************************************************************************/
uint32_t cc3100_socket::sl_Htonl( uint32_t val )
{
    uint32_t i = 1;
    int8_t *p = (int8_t *)&i;
    if (p[0] == 1) { /* little endian */
        p[0] = ((int8_t* )&val)[3];
        p[1] = ((int8_t* )&val)[2];
        p[2] = ((int8_t* )&val)[1];
        p[3] = ((int8_t* )&val)[0];
        return i;
    } else { /* big endian */
        return val;
    }
}

/*******************************************************************************/
/*  sl_Htonl */
/*******************************************************************************/
uint16_t cc3100_socket::sl_Htons( uint16_t val )
{
    int16_t i = 1;
    int8_t *p = (int8_t *)&i;
    if (p[0] == 1) { /* little endian */
        p[0] = ((int8_t* )&val)[1];
        p[1] = ((int8_t* )&val)[0];
        return i;
    } else { /* big endian */
        return val;
    }
}

/*******************************************************************************/
/*  sl_Recv */
/*******************************************************************************/
typedef union {
    _sendRecvCommand_t  Cmd;
    _SocketResponse_t   Rsp;
} _SlRecvMsg_u;

#if _SL_INCLUDE_FUNC(sl_Recv)
const _SlCmdCtrl_t _SlRecvCmdCtrl = {
    SL_OPCODE_SOCKET_RECV,
    sizeof(_sendRecvCommand_t),
    sizeof(_SocketResponse_t)
};

int16_t cc3100_socket::sl_Recv(int16_t sd, void *pBuf, int16_t Len, int16_t flags)
{
    _SlRecvMsg_u    Msg;
    _SlCmdExt_t     CmdExt;
    _SlReturnVal_t status;

    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.RxPayloadLen = Len;
    CmdExt.pRxPayload = (uint8_t *)pBuf;

    Msg.Cmd.sd = (uint8_t)sd;
    Msg.Cmd.StatusOrLen = Len;

    /*  no size truncation in recv path */
    CmdExt.RxPayloadLen = Msg.Cmd.StatusOrLen;

    Msg.Cmd.FamilyAndFlags = flags & 0x0F;

    status = _driver._SlDrvDataReadOp((_SlSd_t)sd, (_SlCmdCtrl_t *)&_SlRecvCmdCtrl, &Msg, &CmdExt);
    if( status != SL_OS_RET_CODE_OK ) {
        return status;
    }

    /*  if the Device side sends less than expected it is not the Driver's role */
    /*  the returned value could be smaller than the requested size */
    return (int16_t)Msg.Rsp.statusOrLen;
}
#endif

/*******************************************************************************/
/*  sl_SetSockOpt */
/*******************************************************************************/
typedef union {
    _setSockOptCommand_t    Cmd;
    _SocketResponse_t       Rsp;
} _SlSetSockOptMsg_u;

const _SlCmdCtrl_t _SlSetSockOptCmdCtrl = {
    SL_OPCODE_SOCKET_SETSOCKOPT,
    sizeof(_setSockOptCommand_t),
    sizeof(_SocketResponse_t)
};

#if _SL_INCLUDE_FUNC(sl_SetSockOpt)
int16_t cc3100_socket::sl_SetSockOpt(int16_t sd, int16_t level, int16_t optname, const void *optval, SlSocklen_t optlen)
{
    _SlSetSockOptMsg_u    Msg;
    _SlCmdExt_t           CmdExt;

     _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.TxPayloadLen = optlen;
    CmdExt.pTxPayload = (uint8_t *)optval;

    Msg.Cmd.sd = (uint8_t)sd;
    Msg.Cmd.level = (uint8_t)level;
    Msg.Cmd.optionLen = (uint8_t)optlen;
    Msg.Cmd.optionName = (uint8_t)optname;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlSetSockOptCmdCtrl, &Msg, &CmdExt));

    return (int16_t)Msg.Rsp.statusOrLen;
}
#endif

/*******************************************************************************/
/*  sl_GetSockOpt */
/*******************************************************************************/
typedef union {
    _getSockOptCommand_t    Cmd;
    _getSockOptResponse_t   Rsp;
} _SlGetSockOptMsg_u;

#if _SL_INCLUDE_FUNC(sl_GetSockOpt)
const _SlCmdCtrl_t _SlGetSockOptCmdCtrl = {
    SL_OPCODE_SOCKET_GETSOCKOPT,
    sizeof(_getSockOptCommand_t),
    sizeof(_getSockOptResponse_t)
};

int16_t cc3100_socket::sl_GetSockOpt(int16_t sd, int16_t level, int16_t optname, void *optval, SlSocklen_t *optlen)
{
    _SlGetSockOptMsg_u    Msg;
    _SlCmdExt_t           CmdExt;

    if (*optlen == 0) {
        return SL_EZEROLEN;
    }
    
    _driver._SlDrvResetCmdExt(&CmdExt);
    CmdExt.RxPayloadLen = *optlen;
    CmdExt.pRxPayload = (uint8_t*)optval;

    Msg.Cmd.sd = (uint8_t)sd;
    Msg.Cmd.level = (uint8_t)level;
    Msg.Cmd.optionLen = (uint8_t)(*optlen);
    Msg.Cmd.optionName = (uint8_t)optname;

    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlGetSockOptCmdCtrl, &Msg, &CmdExt));

    if (CmdExt.RxPayloadLen < CmdExt.ActualRxPayloadLen) {
        *optlen = Msg.Rsp.optionLen;
        return SL_ESMALLBUF;
    } else {
        *optlen = (uint8_t)CmdExt.ActualRxPayloadLen;
    }
    return (int16_t)Msg.Rsp.status;
}
#endif

/*******************************************************************************/
/*  sl_Select */
/* ******************************************************************************/
typedef union {
    _SelectCommand_t   Cmd;
    _BasicResponse_t   Rsp;
} _SlSelectMsg_u;

#ifndef SL_TINY_EXT
#if _SL_INCLUDE_FUNC(sl_Select)
const _SlCmdCtrl_t _SlSelectCmdCtrl = {
    SL_OPCODE_SOCKET_SELECT,
    sizeof(_SelectCommand_t),
    sizeof(_BasicResponse_t)
};

int16_t cc3100_socket::sl_Select(int16_t nfds, SlFdSet_t *readsds, SlFdSet_t *writesds, SlFdSet_t *exceptsds, SlTimeval_t *timeout)
{
    _SlSelectMsg_u          Msg;
    _SelectAsyncResponse_t  AsyncRsp;
    uint8_t ObjIdx = MAX_CONCURRENT_ACTIONS;

    Msg.Cmd.nfds          = (uint8_t)nfds;
    Msg.Cmd.readFdsCount  = 0;
    Msg.Cmd.writeFdsCount = 0;

    Msg.Cmd.readFds = 0;
    Msg.Cmd.writeFds = 0;

    if( readsds ) {
        Msg.Cmd.readFds       = (uint16_t)readsds->fd_array[0];
    }
    if( writesds ) {
        Msg.Cmd.writeFds      = (uint16_t)writesds->fd_array[0];
    }
    if( NULL == timeout ) {
        Msg.Cmd.tv_sec = 0xffff;
        Msg.Cmd.tv_usec = 0xffff;
    } else {
        if( 0xffff <= timeout->tv_sec ) {
            Msg.Cmd.tv_sec = 0xffff;
        } else {
            Msg.Cmd.tv_sec = (uint16_t)timeout->tv_sec;
        }
        timeout->tv_usec = timeout->tv_usec >> 10;  /*  convert to milliseconds */
        if( 0xffff <= timeout->tv_usec ) {
            Msg.Cmd.tv_usec = 0xffff;
        } else {
            Msg.Cmd.tv_usec = (uint16_t)timeout->tv_usec;
        }
    }

    /* Use Obj to issue the command, if not available try later */
    ObjIdx = _driver._SlDrvProtectAsyncRespSetting((uint8_t*)&AsyncRsp, SELECT_ID, SL_MAX_SOCKETS);

    if (MAX_CONCURRENT_ACTIONS == ObjIdx)
    {
        return SL_POOL_IS_EMPTY;
    }
    
    /* send the command */
    VERIFY_RET_OK(_driver._SlDrvCmdOp((_SlCmdCtrl_t *)&_SlSelectCmdCtrl, &Msg, NULL));

    if(SL_OS_RET_CODE_OK == (int16_t)Msg.Rsp.status) {
        _driver._SlDrvSyncObjWaitForever(&g_pCB->ObjPool[ObjIdx].SyncObj);
        Msg.Rsp.status = AsyncRsp.status;

        if(  ((int16_t)Msg.Rsp.status) >= 0 ) {
            if( readsds ) {
                readsds->fd_array[0]  = AsyncRsp.readFds;
            }
            if( writesds ) {
                writesds->fd_array[0] = AsyncRsp.writeFds;
            }
        }
    }

    _driver._SlDrvReleasePoolObj(ObjIdx);
    return (int16_t)Msg.Rsp.status;
}

/*  Select helper functions */
/*******************************************************************************/
/*  SL_FD_SET */
/* ******************************************************************************/
void cc3100_socket::SL_FD_SET(int16_t fd, SlFdSet_t *fdset)
{
    fdset->fd_array[0] |=  (1<< (fd & BSD_SOCKET_ID_MASK));
}
/*******************************************************************************/
/*  SL_FD_CLR */
/*******************************************************************************/
void cc3100_socket::SL_FD_CLR(int16_t fd, SlFdSet_t *fdset)
{
    fdset->fd_array[0] &=  ~(1<< (fd & BSD_SOCKET_ID_MASK));
}
/*******************************************************************************/
/*  SL_FD_ISSET */
/*******************************************************************************/
int16_t  cc3100_socket::SL_FD_ISSET(int16_t fd, SlFdSet_t *fdset)
{
    if( fdset->fd_array[0] & (1<< (fd & BSD_SOCKET_ID_MASK)) ) {
        return 1;
    }
    return 0;
}
/*******************************************************************************/
/*  SL_FD_ZERO */
/*******************************************************************************/
void cc3100_socket::SL_FD_ZERO(SlFdSet_t *fdset)
{
    fdset->fd_array[0] = 0;
}

#endif
#endif

}//namespace mbed_cc3100



