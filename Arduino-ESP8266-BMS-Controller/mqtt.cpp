
#include "bms_values.h"
#include "settings.h"
#include "bms_values.h"
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <INA.h>


#define DEBUG

extern PubSubClient mqttclient;
extern INA_Class INA;
extern bool charging;
extern bool discharging;

const float ina_correction = 2.13;   //Correction factor for voltage divider

// Print debug message to MQTT console
void mqttprint ( String message ) {
  mqttclient.beginPublish("diybmsdebug",message.length(), false);    //New function in PubSubClient to publish long messages
  mqttclient.print(message);
  mqttclient.endPublish();
}


//Report Configuration to MQTT
void report_configuration(){
  mqttprint("Report Configuration");
}


void mqttreconnect() {
  //int count=8;    //How many attempts shall we make?
  // Loop until we're reconnected
  //while (!mqttclient.connected() && count ) {
    //Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "diyBMS-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttclient.connect(clientId.c_str())) {
      //Serial.println("connected");
      mqttclient.subscribe(MQTT_COMMAND_TOPIC);
    } else {
      //Serial.print("failed, rc=");
      //Serial.println(mqttclient.state());
  }
}


void updatemqtt( struct  cell_module *module ) {
  DynamicJsonBuffer jsonBuffer(400);                              //Allocate buffer for JSON
  JsonObject& root = jsonBuffer.createObject();  
  root["address"] = String(module->address);
  if (module->valid_values == true) {                     //Do not send junk data
    root["volts"] = String(module->valid_values ? module->voltage : 0);
    root["temp"] = String(module->temperature);
  } 
  root["bypass"] = String(module->bypass_status);
  char jsonout[256];
  root.printTo(jsonout);
  mqttclient.publish(MQTT_TOPIC, jsonout, root.measureLength());   //Publish to MQTT
  return;
}

//Send bus voltage/current to MQTT
void updatebus() {
  DynamicJsonBuffer jsonBuffer(192);
  JsonObject& root = jsonBuffer.createObject();
  float bus_voltage = ina_correction * INA.getBusMilliVolts()/1000.0;
  float bus_amps = (float)INA.getBusMicroAmps()/1000000.0;
  float bus_watts = bus_amps*bus_voltage;

#ifdef DEBUG
  String bus_readings="Bus "+String(bus_voltage)+" V  ";
  bus_readings += String(bus_amps)+" A  ";
  bus_readings += String(bus_watts) + " W";
  mqttprint(bus_readings);
#endif
  root["busvolts"] = String(bus_voltage);
  root["busamps"] = String(bus_amps);
  root["buswatts"] = String(bus_watts);
  root["charging"] = String(charging);  
  root["discharging"] = String(discharging);    //Update charger status
  char jsonout[128];
  root.printTo(jsonout);
  mqttclient.publish(MQTT_TOPIC, jsonout, root.measureLength());   //Publish to MQTT
  return;
}

