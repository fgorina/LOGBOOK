#include "NMEA2000_twai.h"
#include <string.h>

tNMEA2000_twai::tNMEA2000_twai(gpio_num_t txPin, gpio_num_t rxPin)
  : tNMEA2000(), _txPin(txPin), _rxPin(rxPin), _isOpen(false) {}

void tNMEA2000_twai::InitCANFrameBuffers() {
  if (MaxCANReceiveFrames < 10) MaxCANReceiveFrames = 50;
  if (MaxCANSendFrames < 10) MaxCANSendFrames = 40;
  MaxCANSendFrames = 4; // TWAI driver maintains its own internal TX queue
  tNMEA2000::InitCANFrameBuffers();
}

bool tNMEA2000_twai::CANOpen() {
  if (_isOpen) return true;

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 50;
  g_config.tx_queue_len = 40;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }

  _isOpen = true;
  return true;
}

bool tNMEA2000_twai::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.data_length_code = len > 8 ? 8 : len;
  msg.extd = 1;
  memcpy(msg.data, buf, msg.data_length_code);
  return twai_transmit(&msg, wait_sent ? portMAX_DELAY : 0) == ESP_OK;
}

bool tNMEA2000_twai::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  twai_message_t msg;
  if (twai_receive(&msg, 0) != ESP_OK) return false;
  if (!msg.extd) return false; // NMEA2000 uses only extended (29-bit) frames
  id = msg.identifier;
  len = msg.data_length_code;
  memcpy(buf, msg.data, len);
  return true;
}
