#ifndef NET_SIGNALK_WS_H
#define NET_SIGNALK_WS_H
#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include "State.h"
using namespace websockets;

class NetSignalkWS
{

protected:
  WebsocketsClient *client = nullptr;
  const char *host;
  int port;
  tState *state;
  unsigned long lastMillis = 0;
  unsigned long timeout = 10000;
  bool started = false;

  char buff[300];

  void onWsEventsCallback(WebsocketsEvent event, String data);
  void onWsMessageCallback(WebsocketsMessage message);

public:
  NetSignalkWS(const char *host, int port, tState* state);
  bool connect();
  void subscribe();
  void greet();
  void begin();
  void run();
};

#endif
