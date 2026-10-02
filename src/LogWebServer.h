#pragma once

// Configuration web server (see LogWebServer.cpp)

void startWebServer();  // Registers the handlers and starts listening
void handleWebServer(); // Runs deferred work (restart), call from the network task
