#include "NMEA2000_twai.h"
#include "TwaiLog.h"
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
  TwaiLogger.printf("tNMEA2000_twai::CANOpen called, txPin=%d rxPin=%d\n", _txPin, _rxPin);
  if (_isOpen) {
    TwaiLogger.println("tNMEA2000_twai::CANOpen: already open");
    return true;
  }

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 50;
  g_config.tx_queue_len = 40;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t installErr = twai_driver_install(&g_config, &t_config, &f_config);
  TwaiLogger.printf("tNMEA2000_twai::CANOpen: twai_driver_install=%d\n", installErr);
  if (installErr != ESP_OK) return false;

  esp_err_t startErr = twai_start();
  TwaiLogger.printf("tNMEA2000_twai::CANOpen: twai_start=%d\n", startErr);
  if (startErr != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }

  _isOpen = true;
  TwaiLogger.println("tNMEA2000_twai::CANOpen: success");
  return true;
}

void tNMEA2000_twai::RecoverFromBusOff(twai_state_t state) {
  // Bus-off doesn't clear itself: the driver requires an explicit
  // twai_initiate_recovery(), which (once 128 occurrences of 11 consecutive
  // recessive bits are observed) drops the controller to STOPPED - it then
  // requires an explicit twai_start() to resume normal operation. Without
  // this, a single bus-off is permanent for the rest of the boot.
  if (state == TWAI_STATE_BUS_OFF) {
    TwaiLogger.println("TWAI bus-off - initiating recovery");
    twai_initiate_recovery();
  } else if (state == TWAI_STATE_STOPPED) {
    TwaiLogger.println("TWAI recovered from bus-off - restarting");
    twai_start();
  }
}

bool tNMEA2000_twai::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.data_length_code = len > 8 ? 8 : len;
  msg.extd = 1;
  memcpy(msg.data, buf, msg.data_length_code);
  esp_err_t err = twai_transmit(&msg, wait_sent ? portMAX_DELAY : 0);
  if (err != ESP_OK) {
    twai_status_info_t status;
    twai_get_status_info(&status);
    TwaiLogger.printf("twai_transmit failed: err=%d state=%d tec=%u rec=%u tx_failed=%u bus_err=%u arb_lost=%u\n",
                  err, status.state, status.tx_error_counter, status.rx_error_counter,
                  status.tx_failed_count, status.bus_error_count, status.arb_lost_count);
    RecoverFromBusOff(status.state);
  }
  return err == ESP_OK;
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
