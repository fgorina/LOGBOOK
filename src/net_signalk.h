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

  void onWsEventsCallback(WebsocketsEvent event, String data);
  void onWsMessageCallback(WebsocketsMessage message);

public:
  NetSignalkWS(const char *host, int port, tState* state);
  bool connect();
  void ws_signalk_subscribe();
  void begin();
};

#endif
