
#include "bms_values.h"
#include "settings.h"
#include "bms_values.h"
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>


extern PubSubClient mqttclient;




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
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "diyBMS-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttclient.connect(clientId.c_str())) {
      Serial.println("connected");
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
  root["errors"] = String(module->error_count);
  root["comms_alarm"] = String(module->lost_communication);
  char jsonout[256];
  root.printTo(jsonout);
  mqttclient.publish(MQTT_TOPIC, jsonout, root.measureLength());   //Publish to MQTT
  return;
}



