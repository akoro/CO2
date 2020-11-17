#include "TelnetServer.h"

bool Telnet::begin(void)
{
  Server.begin();
  Server.setNoDelay(true);
  return true;
}

void Telnet::run(void)
{
  if (!_flag)                             // если ещё нет клиента
  {
    Client = Server.available();         // создание нового клиента
    if (Client.connected())
    {
      _flag = true;
      Serial.println(F("Telnet client connected"));
      start();
      // соединение только что установилось
    }
    // нет соединения
  }
  else
  {
    if (Client.connected())
    {
      Server.available().stop(); // каждая попытка нового соединения блокируется
      Console::run();
      // соединение активно
    }
    else
    {
      _flag = false;
      Client.stop();
      Serial.println(F("Telnet client disconnected"));
      // соединение только что прекратилось
    }
  }
}