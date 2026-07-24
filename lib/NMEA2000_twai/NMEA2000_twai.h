#pragma once
#include "NMEA2000.h"
#include "driver/twai.h"

#ifndef ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN GPIO_NUM_16
#endif

#ifndef ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#endif

class tNMEA2000_twai : public tNMEA2000 {
public:
  tNMEA2000_twai(gpio_num_t txPin = ESP32_CAN_TX_PIN, gpio_num_t rxPin = ESP32_CAN_RX_PIN);

protected:
  void InitCANFrameBuffers() override;
  bool CANOpen() override;
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent = false) override;
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override;

private:
  gpio_num_t _txPin;
  gpio_num_t _rxPin;
  bool _isOpen;

  void RecoverFromBusOff(twai_state_t state);
};
