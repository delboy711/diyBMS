#include "Arduino.h"

#ifndef config_settings_H_
#define config_settings_H_


//Where in EEPROM do we store the configuration
#define EEPROM_storageSize 1024
#define EEPROM_WIFI_CHECKSUM_ADDRESS 0
#define EEPROM_WIFI_CONFIG_ADDRESS EEPROM_WIFI_CHECKSUM_ADDRESS+sizeof(uint32_t)

#define EEPROM_CHECKSUM_ADDRESS 256
#define EEPROM_CONFIG_ADDRESS EEPROM_CHECKSUM_ADDRESS+sizeof(uint32_t)

// Timeout in secs of hotspot mode after which ESP8266 will reboot
#define AP_TIMEOUT 600
// WiFi connect timeout in secs after which ESP8266 will enter hotspot mode
#define WIFI_TIMEOUT 120
// MQTT topic to send updates to
#define MQTT_TOPIC "diybms"
// MQTT topic to  listen for commands from
#define MQTT_COMMAND_TOPIC "diybms_command"

//URL of location to download  browser files
#define FILES_URL "https://delboy711.github.io/diyBMS/"
//#define FILES_URL "http://192.168.1.4/emoncms/"

//How many bad i2c polls before declaring communication lost
#define BAD_I2C_POLLS 10

struct wifi_eeprom_settings {
  char wifi_ssid[32 + 1];
  char wifi_passphrase[63 + 1];
};

//We have allowed space for 1024-256 bytes of EEPROM for settings (768 bytes)
struct eeprom_settings { 
  bool emoncms_enabled;
  uint8_t emoncms_node_offset;
  int emoncms_httpPort;
  char emoncms_host[64 + 1];
  char emoncms_apikey[32 + 1];
  char emoncms_url[64 + 1];
  bool autobalance_enabled;
  float max_voltage;
  float balance_voltage;
  float balance_dev;
  char mqtt_host[64 + 1];
  bool mqtt_enabled;
  };
extern wifi_eeprom_settings myConfig_WIFI;
extern eeprom_settings myConfig;

void WriteConfigToEEPROM();
bool LoadConfigFromEEPROM();
void WriteWIFIConfigToEEPROM();
bool LoadWIFIConfigFromEEPROM();
void FactoryResetSettings();

#endif

