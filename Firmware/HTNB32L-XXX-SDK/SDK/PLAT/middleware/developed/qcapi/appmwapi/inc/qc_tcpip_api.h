/******************************************************************************
 * (C) Copyright 2018 Qualcomm International Ltd.
 * All Rights Reserved
*******************************************************************************
 *  Filename: ec_tcpip_api.h
 *
 *  Description: API interface implementation header file for socket/tcpip service
 *
 *  History: create by xwang
 *
 *  Notes:
 *
******************************************************************************/
#ifndef __EC_TCPIP_API_H__
#define __EC_TCPIP_API_H__

#include "cms_util.h"

#include "commontypedef.h"
/******************************************************************************
 *****************************************************************************
 * MARCO
 *****************************************************************************
******************************************************************************/
#define TCPIP_SDK_API_TIMEOUT 35


/******************************************************************************
 *****************************************************************************
 * ENUM
 *****************************************************************************
******************************************************************************/
typedef enum TcpipConnectionProtocol_Tag
{
    TCPIP_CONNECTION_PROTOCOL_UDP = 0,
    TCPIP_CONNECTION_PROTOCOL_TCP = 1,
}TcpipConnectionProtocol;

typedef enum TcpipConnectionEvent_Tag
{
    TCPIP_CONNECTION_STATUS_EVENT = 0,
    TCPIP_CONNECTION_RECEIVE_EVENT = 1,
    TCPIP_CONNECTION_UL_STATUS_EVENT = 2,
}TcpipConnectionEvent;

typedef enum TcpipConnectionStatus_Tag
{
    TCPIP_CONNECTION_STATUS_CLOSED = 0,
    TCPIP_CONNECTION_STATUS_CONNECTING = 2,
    TCPIP_CONNECTION_STATUS_CONNECTED = 3,
}TcpipConnectionStatus;

typedef enum TcpipConnectionUlStatus_Tag
{
    Tcpip_Connection_UL_DATA_FAIL = 0, 
    Tcpip_Connection_UL_DATA_SUCCESS = 1,
}TcpipConnectionUlStatus;



/******************************************************************************
 *****************************************************************************
 * STRUCT
 *****************************************************************************
******************************************************************************/

typedef struct TcpipConnectionStatusInd_Tag
{
    INT32 connectionId;
    UINT32 status;  //TcpipConnectionStatus
    UINT32 cause;   //HsSocketErrCause
}TcpipConnectionStatusInd;

typedef struct TcpipConnectionRecvDataInd_Tag
{
    INT32 connectionId; 
    UINT16 length; 
    UINT16 reserved;
    void *data;
}TcpipConnectionRecvDataInd;

typedef struct TcpipConnectionUlDataStatusInd_Tag
{
    INT32 connectionId;  
    UINT16 sequence;
    UINT16 status; //TcpipConnectionUlStatus
}TcpipConnectionUlDataStatusInd;


typedef void (*tcpipConnectionCallBack)(UINT8 connectionEventType, void *bodyEvent);



/******************************************************************************
 *****************************************************************************
 * FUNCTION
 *****************************************************************************
******************************************************************************/

/******************************************************************************
 * TcpipConnectionCreate
 * Description: create a tcpip connection
 * input: type(mandatory) 
 *        localIp(option) stringlen <= 63
 *        localPort(option)
 *        destIp(mandatory)"183.100.1.123" or"2001::2"   
 *        destPort(mandatory)
 *        callBack -> the tcpip connection related event(TcpipConnectionEvent) callback handler
 * output: >=0 connectionId, <0 fail
 * Comment:
******************************************************************************/

INT32 tcpipConnectionCreate(TcpipConnectionProtocol protocol, char *localIp, UINT16 localPort, char *destIp, UINT16 destPort, tcpipConnectionCallBack callBack);

/******************************************************************************
 * TcpipConnectionSend
 * Description: send data by a tcpip connection
 * input: connectionId(mandatory)
 *        data(mandatory) -> the data which need send, 
 *        dataLen(mandatory) -> data length
 *        raiInfo(option)
 *        sequence(option) -> the sequence number of the UL data. 1~255: a UL status event will be indicate with the socket event callback 0:no UL status event
 *        expectFlag
 * output:< 0 fail;> 0 send Len
 * Comment:
******************************************************************************/
INT32 tcpipConnectionSend(INT32 connectionId, UINT8 *data, UINT16 dataLen, UINT8 raiInfo, UINT8 expectFlag, UINT8 sequence);


/******************************************************************************
 * TcpipConnectionClose
 * Description: close a tcpip connection
 * input: connectionId(mandatory) 
 * output:
 * Comment: < 0 fail
******************************************************************************/
INT32 tcpipConnectionClose(INT32 connectionId);

#endif


