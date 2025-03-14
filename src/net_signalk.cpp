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
        started = false;
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
    if (!started)
    {
        subscribe();
        started = true;
    }
    // Serial.print("Got Message: ");
    // Serial.println(message.data());
    // wsskClient.lastActivity = millis();
    lastMillis = millis();
    // Serial.println(message.data());
    bool found = state->signalk_parse_ws(message.data());
    if (!found)
    {
        Serial.print("Not Processed: ");
        Serial.println(message.data());
    }
}

NetSignalkWS::NetSignalkWS(const char *host, int port, tState *state)
{
    this->state = state;
    this->host = host;
    this->port = port;
}

void NetSignalkWS::greet()
{
}

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
            sprintf(buff, "ws://%s:%d/signalk/v1/stream?subscribe=none", host, port);
            Serial.print("Reconnecting to ");
            Serial.println(buff);
            if (client->connect(String(buff)))
            {
                Serial.println("Ws conection opened");
                lastMillis = millis();
                greet();
                return true;
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
    return false;
};

void NetSignalkWS::subscribe()
{
    const char *data = "{\"context\": \"\",\"subscribe\": [{\"path\": \"environment.wind.angleApparent\", \"policy\":\"instant\"},"
                       "{\"path\": \"environment.wind.speedApparent\", \"policy\":\"instant\"},"
                       "{\"path\": \"environment.wind.speedTrue\", \"policy\":\"instant\"},"
                       "{\"path\": \"environment.wind.directionTrue\", \"policy\":\"instant\"},"
                       "{\"path\": \"environment.depth.belowTransducer\", \"policy\":\"instant\"},"
                       "{\"path\": \"propulsion.p0.revolutions\", \"policy\":\"instant\"},"
                       "{\"path\": \"propulsion.p0.temperature\", \"policy\":\"instant\"},"
                       "{\"path\": \"navigation.headingMagnetic\", \"policy\":\"instant\"},"
                       "{\"path\": \"navigation.headingTrue\", \"policy\":\"instant\"},"
                       "{\"path\": \"navigation.position\", \"policy\":\"instant\"},"
                       "{\"path\": \"navigation.speedOverGround\", \"policy\":\"instant\"},"
                       "{\"path\": \"navigation.courseOverGroundTrue\", \"policy\":\"instant\"}"
                       "]}\n";
    client->send(data);
    Serial.println("Sent Subscribe");
}

void NetSignalkWS::begin()
{

    bool ok = connect();

    Serial.print(" Status ");
    Serial.println(ok);
}

void NetSignalkWS::run()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        if (client != nullptr)
        {
            if (millis() - lastMillis > timeout)
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
            Serial.println("Connecting to SignalK");
            connect();
        }
    }
}