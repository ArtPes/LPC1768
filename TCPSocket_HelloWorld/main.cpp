#include "mbed.h"
#include "EthernetInterface.h"
#include "rtos.h"
Serial pc(USBTX,USBRX);

int main() {
    EthernetInterface eth;
    eth.init(); //Use DHCP
    int i = eth.connect();
    pc.printf("Connection = %d\n", i);
    
    pc.printf("IP Address is %s\n", eth.getIPAddress());
    pc.printf("Gateway is %s\n", eth.getGateway());
    pc.printf("MAC Address is %s\n", eth.getMACAddress());
    
    TCPSocketConnection sock;
    sock.connect("mbed.org", 80);
    
    char http_cmd[] = "GET /www.google.com \n\n";
    sock.send_all(http_cmd, sizeof(http_cmd)-1);
    
    char buffer[300];
    int ret;
    while (true) {
        ret = sock.receive(buffer, sizeof(buffer)-1);
        if (ret <= 0)
            break;
        buffer[ret] = '\0';
        pc.printf("Received %d chars from server:\n%s\n", ret, buffer);
    }
      
    sock.close();
    
    eth.disconnect();
    
    while(1) {}
}
