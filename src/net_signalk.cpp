#include <Arduino.h>
#include "net_signalk.h"

void NetSignalkWS::onWsEventsCallback(WebsocketsEvent event, String data)
{
    if (event == WebsocketsEvent::ConnectionOpened)
    {
        Serial.println("Wss Connnection Opened");
        lastMillis = millis();
    }
    else if (event == WebsocketsEvent::ConnectionClosed)
    {

        Serial.print("Wss Connnection Closed: ");
        CloseReason reason = client->getCloseReason();
        Serial.print("Close Reason:");
        Serial.println(reason);
        Serial.println(data);
    }
    else if (event == WebsocketsEvent::GotPing)
    {
        Serial.println("Wss Got a Ping!");
    }
    else if (event == WebsocketsEvent::GotPong)
    {
        Serial.println("Wss Got a Pong!");
    }
    else
    {
        Serial.println("Received unindentified WebSockets event");
    }
}

void NetSignalkWS::onWsMessageCallback(WebsocketsMessage message)
{

    // Serial.print("Got Message: ");
    // Serial.println(message.data());
    // wsskClient.lastActivity = millis();
    lastMillis = millis();
    bool found = state->signalk_parse_ws(message.data());
    if (!found)
    {
        Serial.println(message.data());
    }
}

NetSignalkWS::NetSignalkWS(const char *host, int port, tState *state)
{
    this->state = state;
    this->host = host;
    this->port = port;
}

/*
void ws_signalk_greet(WiFiClient& client) {
  String dataFeed = client.readStringUntil('\n');
  const char* data = "{\"context\": \"*\",\"subscribe\": [{\"path\": \"*\", \"policy\":\"instant\"}]}";
  client.println(data);
  client.flush();

  Serial.println("Sent Subscribe all");
}
*/

bool NetSignalkWS::connect()
{
    if (strlen(host) > 0 && port > 0)
    {
        if (client == nullptr || !client->available())
        {
            client = new WebsocketsClient();
            client->onMessage([this](WebsocketsMessage message)
                              { this->onWsMessageCallback(message); });
            client->onEvent([this](WebsocketsEvent event, String data)
                            { this->onWsEventsCallback(event, data); });
            sprintf(buff, "ws://%s:%d/signalk/v1/stream", host, port);
            Serial.print("Reconnecting to ");
            Serial.println(buff);
            if (client->connect(String(buff)))
            {
                Serial.println("Ws conection opened");
                lastMillis = millis();
                return true;
                // signalk_greet(client.c);
            }
            else
            {

                Serial.println("Cannot connect");
                return false;
            }
        }
        else
        {
            return true;
        }
    }
};

void NetSignalkWS::subscribe()
{
}

void NetSignalkWS::begin()
{

    bool ok = connect();

    Serial.print(" Status ");
    Serial.println(ok);
}

void NetSignalkWS::run()
{

    if (client != nullptr)
    {
        if (millis() - lastMillis > timeout && false)
        {
            Serial.println("Timeout");
            client->close();
            delete client;
            client = nullptr;
            lastMillis = millis();
            connect();
        }
        else
        {
            client->poll();
        }
    }
    else
    {
        connect();
    }
}