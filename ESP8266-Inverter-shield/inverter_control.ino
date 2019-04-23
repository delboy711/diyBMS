

/*
    Inverter control contains an MQTT client which listen to commands from an MQTT broker running on Emoncms
    which informs the  controller what voltage to apply to a GTIL2-1000 inverter output power. Inverter control then modulates a PWM signal to an IGBT circuit to
    set the output voltage in proportion to the available power.
    The controller also reads the actual power consumed, and relays that back to Emoncms using MQTT
    Derek Jennings 19th April 2019

 */

#define MAXDISCHARGE  290     //Maximum setting for discharge PWM
#define HOSTNAME  "inverter"

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

unsigned long lastmessage = millis(); // time of last update
//PWM parameters
int pwm_frequency = 1010;
byte pwm_pin = 14;              // PWM pin number, D13 on Wemos D1
unsigned int dischargepower = 0;


ESP8266WiFiMulti wifiMulti;



IPAddress mqttServerIP;

WiFiClient espClient;
PubSubClient mqttclient(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
// name of MQTT broker
//const char* mqttServerName = "cloudserver.local";
const char* mqttServerName = "192.168.1.4";

// Print debug message to MQTT console
void mqttprint ( String message ) {
  mqttclient.beginPublish("diybmsdebug",message.length(), false);    //New function in PubSubClient to publish long messages
  mqttclient.print(message);
  mqttclient.endPublish();
}


void callback(char* topic, byte* payload, unsigned int length) {

  StaticJsonBuffer<200> jsonBuffer;     //Decode JSON
  JsonObject& root = jsonBuffer.parseObject(payload);
  // Test if parsing succeeds.
  if (!root.success()) {
    mqttprint("parseObject() failed");
    return;
  }

   uint8_t command = root["command"];
  switch(command) {
    case 1:                                     // Command 1 - resetESP
      mqttprint("Restarting Inverter Controller");
      delay(1000);
      ESP.restart();
      break;

    case 14:                     //Immediate shutdown - Turns off discharging without delay
    case 13:                     //Cancel Discharging
      dischargepower = 0;
      analogWrite(pwm_pin, dischargepower);
      break;
    case 12:                                            //Enable Inverter
      if( root.containsKey("dischargepower") == true ) dischargepower = root["dischargepower"];
      dischargepower = (dischargepower < MAXDISCHARGE) ? dischargepower : 0;    //Do not permit power output > MAX
      mqttprint("Inverter PWM " + String(dischargepower) );

      // Write duty cycle to PWM pin
      analogWrite(pwm_pin, dischargepower);
      lastmessage = millis();    // update time of last message
      default:
      break;
    
  }
}

void reconnect() {
  // Loop until we're reconnected
  //while (!mqttclient.connected()) {
    String clientId = "Inverter";
    // Attempt to connect
    if (mqttclient.connect(clientId.c_str())) {
      // Once connected, publish an announcement...
      mqttclient.publish("diybmsdebug", "Inverter connected");
      // ... and resubscribe
      mqttclient.subscribe("diybms_command");
    
    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  //}
}



void setup(void)
{ 
  pinMode(A0, INPUT);
  pinMode(pwm_pin,OUTPUT);
  analogWriteFreq(pwm_frequency); 
  Serial.begin(115200);
  analogWrite(pwm_pin, 0);      // Initialise PWM
  Serial.println("Welcome to GTIL2 InverterController");
  
  // Connect to WiFi network
  wifiMulti.addAP("ST780wl", "inknatch726!hope");
  wifiMulti.addAP("buffalo", "inknatch726!hope");
  wifiMulti.addAP("8848DBjen", "inknatch726!hope");
  Serial.println("");
  Serial.println("Connecting Wifi...");
    while (wifiMulti.run() != WL_CONNECTED) {
      delay(500);
    }
  Serial.println("");
  Serial.println("WiFi connected to ");
  Serial.println( WiFi.SSID());
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); 

  // Start OTA server.
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();
  //Set up mDNS responder:
  if (!MDNS.begin(HOSTNAME)) {
    //Serial.println("Error setting up MDNS responder!");
    //This will force a reboot of the ESP module by hanging the loop
    while (1) {
      delay(1000);
    }
  }
// Set up MQTT client

  mqttclient.setServer(mqttServerName, 1883);
  mqttclient.setCallback(callback);
  

  ArduinoOTA.onStart([]() {
  Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
  });

}

void loop(void)
{
  // Ensure we are connected to MQTT
  if (!mqttclient.connected()) {
  reconnect();
  }
  mqttclient.loop();

  if ( millis() > lastmessage + 300000 ) {
    analogWrite(pwm_pin, 0 );   // Stop PWM if no contact for 5 mins
    lastmessage = millis();
  }
  // Handle OTA server.
  ArduinoOTA.handle();
  yield();
  
}

