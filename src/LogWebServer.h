#pragma once

// Configuration web server (see LogWebServer.cpp)

void startWebServer();  // Registers the handlers and starts listening
void handleWebServer(); // Serves pending requests, call from the network task
