#include "cc3100_simplelink.h"
#include "cc3100_sl_common.h"
#include "fPtr_func.h"
#include "cc3100.h"
#include "cc3100_spi.h"
#include "myBoardInit.h"
#include "cc3100_socket.h"
#include "cc3100_driver.h"
#include "cc3100_nonos.h"

using namespace mbed_cc3100;

#define SL_STOP_TIMEOUT        0xFF

/* IP addressed of server side socket. Should be in long format,
 * E.g: 0xc0a8010a == 192.168.1.10 */
#define IP_ADDR         0xc0a82b69 //192.168.43.105
#define PORT_NUM        50001            /* Port number to be used */

#define BUF_SIZE        1400
#define NO_OF_PACKETS   1000

cc3100 _cc3100(p9, p10, p8, SPI(p11, p12, p13));//LPC1768  irq, nHib, cs, mosi, miso, sck
Serial pc(USBTX, USBRX);//lpc1768
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);


//GLOBAL VARIABLES -- Start
int32_t demo = 0;
//GLOBAL VARIABLES -- End


static void displayBanner()
{
     printf("\n\r\n\r");
     printf("***********Getting started with station application***********");
     printf("\n\r*******************************************************************************\n\r");
}


//******************************STATION_MODE***********************************************
//*****************************************************************************************
void station_app()
{
    int32_t retVal = -1;
    
    /* Connecting to WLAN AP */
    retVal = _cc3100.establishConnectionWithAP();
    //printf("retVal 000: %d\n\r",retVal);   
    if(retVal < 0)
    {
        printf(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }    
    printf(" Connection established w/ AP and IP is acquired \n\r");
    
    printf(" Pinging...! \n\r");
    retVal = _cc3100.checkLanConnection();
    //printf("retVal 001: %d\n\r",retVal);
    if(retVal < 0)
    {
        printf(" Device couldn't connect to LAN \n\r");
        LOOP_FOREVER();
    }
    printf(" Device successfully connected to the LAN\n\r");
    /*
    retVal = _cc3100.checkInternetConnection();
    if(retVal < 0)
    {
        printf(" Device couldn't connect to the internet \n\r");
        LOOP_FOREVER();
    }

    printf(" Device successfully connected to the internet \n\r");*/
}
//****************************END_STATION_MODE*********************************************
//*****************************************************************************************
void led(int result){
    
    if (result > 0)
      { 
            myled1 = 1;
            wait(0.05);
            myled1 = 0;
            wait(0.05);
                    myled2 = 1;
            wait(0.05);
            myled2 = 0;
            wait(0.05);
                    myled3 = 1;
            wait(0.05);
            myled3 = 0;
            wait(0.05);
                    myled4 = 1;
            wait(0.05);
            myled4 = 0;
            wait(0.05);
        }
    else 
    { myled1 = 0;
      myled2 = 0;
      myled3 = 0;
      myled4 = 0;
    }

 }



static int32_t BsdTcpServer(uint16_t Port)
{
    SlSockAddrIn_t  Addr;
    SlSockAddrIn_t  LocalAddr;
    char stringa[100];
    //int16_t         idx = 0;
    int16_t          AddrSize = 0;
    int16_t          SockID = 0;
    int32_t          Status = 0;
    int16_t          newSockID = 0;
    int16_t          LoopCount = 0;
    //int16_t          SlSockAddrIn_t = 0;
    int16_t          recvSize = 0;
       
/*
    for (idx=0 ; idx<BUF_SIZE ; idx++)
    {
        uBuf.BsdBuf[idx] = (_u8)(idx % 10);
    }
*/

    LocalAddr.sin_family = SL_AF_INET;
    LocalAddr.sin_port = _cc3100._socket.sl_Htons(PORT_NUM);
    LocalAddr.sin_addr.s_addr = 0;
    
    
    
        SockID = _cc3100._socket.sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
        if( SockID < 0 )
        {
            printf(" \n[TCP Server] Create socket Error \n\r");
            ASSERT_ON_ERROR(SockID);
        }
        else 
            printf("\nCreate socket \n\r");
        
    
        AddrSize = sizeof(SlSockAddrIn_t);
        Status = _cc3100._socket.sl_Bind(SockID, (SlSockAddr_t *)&LocalAddr, AddrSize);
        if( Status < 0 )
        {
            _cc3100._socket.sl_Close(SockID);
            printf(" \n[TCP Server] Socket address assignment Error \n\r");
            ASSERT_ON_ERROR(Status);
        }
        else
            printf("\nSocket address assignment \n\r");
    
        Status = _cc3100._socket.sl_Listen(SockID, 0);
        if( Status < 0 )
        {
            _cc3100._socket.sl_Close(SockID);
            printf("\n [TCP Server] Listen Error \n\r");
            ASSERT_ON_ERROR(Status);
        }
        else
            printf("\nListen for a Connection \n\r");
    
        newSockID = _cc3100._socket.sl_Accept(SockID, (SlSockAddr_t *)&Addr,(SlSocklen_t*)&AddrSize);
        if( newSockID < 0 )
        {
            _cc3100._socket.sl_Close(SockID);
            printf("\n [TCP Server] Accept connection Error \n\r");
            ASSERT_ON_ERROR(newSockID);
        }
        else 
            printf("\nAccept connection \n\r");

    while (LoopCount < NO_OF_PACKETS)
        {   
            //TODO : sarebbe da chiamare la funzione led() con un altro thread così da farla girare in background
            led(1);
            recvSize = BUF_SIZE;
            //printf("RecvSize = %d \n\r",recvSize);
            do
            {
                Status = _cc3100._socket.sl_Recv(newSockID, &(stringa), strlen(stringa), 0);
                if( Status <= 0 )
                {
                    _cc3100._socket.sl_Close(newSockID);
                    _cc3100._socket.sl_Close(SockID);
                    led(1);
                    printf(" \n[TCP Server] Data recv Error. Close socket %d \n\n\r",SockID);
                    //ASSERT_ON_ERROR(TCP_RECV_ERROR);
                    
                    /*-----Accept new connection----------*/
                    SockID = _cc3100._socket.sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
                    printf("----------Create new socket---------- \n\n\r");
                    AddrSize = sizeof(SlSockAddrIn_t);
                    Status = _cc3100._socket.sl_Bind(SockID, (SlSockAddr_t *)&LocalAddr, AddrSize);
                    printf("----------Socket address assignment---------- \n\n\r");
                    Status = _cc3100._socket.sl_Listen(SockID, 0);
                    printf("----------Listen for a new Connection---------- \n\n\r");
                    newSockID = _cc3100._socket.sl_Accept(SockID, (SlSockAddr_t *)&Addr,(SlSocklen_t*)&AddrSize);
                    printf("----------Accept new connection---------- \n\n\r");
                    led(1);
                }
                
                //printf("Status = %d \n\r",Status);
                recvSize -= Status;
                //printf("RecvSize = %d \n\r",recvSize);
                printf("%s",stringa);
                led(1);
                char thanks [] = "Data received";
                _cc3100._socket.sl_Send(SockID, &(thanks), strlen(thanks), 0);
                
            }
            while(recvSize > 0);
    
            LoopCount++;
        }
        

    
    Status = _cc3100._socket.sl_Close(newSockID);
    ASSERT_ON_ERROR(Status);

    Status = _cc3100._socket.sl_Close(SockID);
    ASSERT_ON_ERROR(Status);

    return SUCCESS;
    
}



//**************************_START_APPLICATION_*********************************
int main(void) 
{
    pc.baud(115200);
    
    
    
    int32_t retVal = -1;
    
    retVal = _cc3100.initializeAppVariables();
    
    displayBanner();

    _cc3100.CLR_STATUS_BIT(g_Status, STATUS_BIT_PING_DONE);
    g_PingPacketsRecv = 0;
    //printf("\n\r FUNZIONE CLR_STATUS_BIT ESEGUITA CON SUCCESSO\n\r");

    //retVal = _cc3100.configureSimpleLinkToDefaultState();
    
    //printf("\n\rRetval 2: %d \n\r ",retVal);
    
    if(retVal < 0) {
        if (DEVICE_NOT_IN_STATION_MODE == retVal)
            printf(" Failed to configure the device to its default state \n\r");

        LOOP_FOREVER();
    }

    printf("\n\rDevice is configured in it's default state \n\r");
    /*printf("\n\r******************************************************************************************************\n\r");
    printf("\n\r*****_DA_QUI_IN_POI_SCELTA_TIPO_FUNZIONE_IN_BASE_AL_VALORE_demo_*****\n\r");
    printf("\n\r******************************************************************************************************\n\r");*/
    
    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    /* Initializing the CC3100 device */
    
    retVal = _cc3100.sl_Start(0, 0, 0);
    
    if ((retVal < 0) || (ROLE_STA != retVal) )
    {
        printf(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    printf("\n\r Device started as STATION \n\r");
    
    
    station_app();
    
    printf("Establishing connection with TCP server \n\r");
    /*Before proceeding, please make sure to have a server waiting on PORT_NUM*/
    retVal = BsdTcpServer(PORT_NUM);
    if(retVal < 0)
        printf(" Failed to establishing connection with TCP server \n\r");
    else
        printf(" Connection with TCP server established successfully \n\r");
    
    return 0;
    
    
    
    
}    

