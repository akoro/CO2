#pragma once

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <console.h>


class Telnet: public Console
{
  private:
  bool _flag;

  public:
  WiFiServer  Server;
  WiFiClient  Client;

  Telnet(): Console(&Client, false), Server(23), Client() 
  {
    _flag = false;
  }
  void exit(void){Client.stop();}
  bool begin(void) override; 
  void run(void) override;
};
