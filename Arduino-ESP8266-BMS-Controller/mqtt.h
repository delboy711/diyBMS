#ifndef mqtt_H_
#define mqtt_H_
#include <WiFiClient.h>
#include <PubSubClient.h>
extern uint8_t DEFAULT_SLAVE_ADDR;
extern uint8_t DEFAULT_SLAVE_ADDR_START_RANGE;
extern uint8_t DEFAULT_SLAVE_ADDR_END_RANGE;

WiFiClient espClient;
PubSubClient mqttclient(espClient);

void mqttprint ( String );
void report_configuration();
void mqttreconnect();
void updatemqtt( struct   cell_module *module);
void updatebus();




#endif
