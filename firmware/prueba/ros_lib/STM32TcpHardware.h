#ifndef ROS_STM32_TCP_HARDWARE_H_
#define ROS_STM32_TCP_HARDWARE_H_

//#include <Ethernet.h>

class STM32Hardware {
public:
  STM32Hardware()
  {
  }

  void setConnection(IPAddress &server, int port = 11411)
  {
    server_ = server;
    serverPort_ = port;
  }

  IPAddress getLocalIP()
  {
    return Ethernet.localIP();
  }

  void init()
  {
    if(tcp_.connected())
    {
      tcp_.stop();
    }
    tcp_.connect(server_, serverPort_);
  }

  int read(){
    if (tcp_.connected())
    {
        return tcp_.read();
    }
    else
    {
      tcp_.stop();
      tcp_.connect(server_, serverPort_);
    }
    return -1;
  }

  void write(const uint8_t* data, int length)
  {
    tcp_.write(data, length);
  }

  unsigned long time()
  {
    return millis();
  }

  bool connected()
  {
    return tcp_.connected();
  }

protected:
  EthernetClient tcp_;
  IPAddress server_;
  uint16_t serverPort_ = 11411;
};

#endif
