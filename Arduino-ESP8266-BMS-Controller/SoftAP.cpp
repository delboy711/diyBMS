
#include "bms_values.h"
#include "SoftAP.h"
#include "settings.h"
#include "bms_values.h"
#include "i2c_cmds.h"

#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>


ESP8266WebServer server(80);

const char* ssid = "DIY_BMS_CONTROLLER";

String networks;

bool manual_balance = false;
extern int balance_status;

void handleNotFound()
{
  String message = "File Not Found\n\n";
  server.send(404, "text/plain", message);
}
void sendHeaders()
{
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "private");
}


String htmlHeader() {
  return String(F("<!DOCTYPE HTML>\r\n<html><head><style>.page {width:300px;margin:0 auto 0 auto;background-color:cornsilk;font-family:sans-serif;padding:22px;} label {min-width:120px;display:inline-block;padding: 22px 0 22px 0;}</style></head><body><div class=\"page\"><h1>DIY BMS</h1>"));
}


String htmlManagementHeader() {
  String s=String(F("<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><title>DIY BMS Management Console</title><script type=\"text/javascript\" src="));
  s += F(FILES_URL);
  s += F("loader.js></script></head><body></body></html>\r\n\r\n");
  return s;
}


String htmlFooter() {
  return String(F("</div></body></html>\r\n\r\n"));
}

void handleRoot()
{
  String s;
  s = htmlHeader();
  //F Macro - http://arduino-esp8266.readthedocs.io/en/latest/PROGMEM.html
  s += F("<h2>WiFi Setup</h2><p>Select local WIFI to connect to:</p><form autocomplete=\"off\" method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"\\save\"><label for=\"ssid\">SSID:</label><select id=\"ssid\" name=\"ssid\">");
  s += networks;
  s += F("</select><label for=\"pass\">Password:</label><input type=\"password\" id=\"id\" name=\"pass\"><br/><input minlength=\"8\" maxlength=\"32\" type=\"submit\" value=\"Submit\"></form>");
  s += htmlFooter();

  sendHeaders();
  server.send(200, "text/html", s);
}

void handleRedirect() {
  sendHeaders();
  server.send(200, "text/html", htmlManagementHeader());
}

void handleProvision() {
  runProvisioning = true;
  server.send(200, "application/json", "[1]\r\n\r\n");
}

void CancelAverageBalance() {
  if (cell_array_max > 0) {
    for (int a = 0; a < cell_array_max; a++) {
      command_set_bypass_voltage(cell_array[a].address,0);
    }
  }
  manual_balance = false;
  balance_status = 0;
  
  Serial.println("Cancelling balancing");  
}
void handleCancelAverageBalance() {
  CancelAverageBalance();
  server.send(200, "application/json", "[1]\r\n\r\n");
}

uint16_t AboveAverageBalance() {
  uint16_t avgint = 0;
  if (cell_array_max > 0) {
    //Work out the average
    float avg = 0;
    for (int a = 0; a < cell_array_max; a++) {
      avg += 1.0 * cell_array[a].voltage;
    }
    avg = avg / cell_array_max;

    avgint = avg;

    for ( int a = 0; a < cell_array_max; a++) {
      if (cell_array[a].voltage > avgint) {
        cell_array[a].balance_target = avgint;
      }
    }
  }
  
  manual_balance = true;
  balance_status = 1;
  return avgint;
}

void handleAboveAverageBalance() {
  uint16_t avgint = AboveAverageBalance();
  server.send(200, "application/json", "[" + String(avgint) + "]\r\n\r\n");
}

void handleFactoryReset() {
  uint8_t module =  server.arg("module").toInt();
  float newValue = server.arg("value").toFloat();

  for ( int a = 0; a < cell_array_max; a++) {
    if (cell_array[a].address == module) {
      Serial.print("FactoryReset ");
      Serial.print(module);
      cell_array[a].factoryReset = true;
      cell_array[a].update_calibration = true;
      server.send(200, "text/plain", "");
      return;
    }
  }
  server.send(500, "text/plain", "");  
}

void handleSetLoadResistance() {
  uint8_t module =  server.arg("module").toInt();
  float newValue = server.arg("value").toFloat();

  Serial.print("SetLoadResistance ");
  Serial.print(module);
  Serial.print(" = ");
  Serial.println(newValue, 6);

  for ( int a = 0; a < cell_array_max; a++) {
    if (cell_array[a].address == module) {

      if (cell_array[a].loadResistance != newValue) {
        cell_array[a].loadResistance = newValue;
        cell_array[a].update_calibration = true;
      }
      server.send(200, "text/plain", "");
      return;
    }
  }
  server.send(500, "text/plain", "");
}


void handleSetEmonCMS() {
  /* Receives HTTP POST for configuring the emonCMS settings
    emoncms_enabled:1
    emoncms_host:192.168.0.26
    emoncms_httpPort:80
    emoncms_node_offset:26
    emoncms_url:/emoncms/input/bulk?data=
    emoncms_apikey:11111111111111111111111111111111
  */
  myConfig.autobalance_enabled = (server.arg("autobalance_enabled").toInt() == 1) ? true : false;
  myConfig.max_voltage = server.arg("max_voltage").toFloat();
  myConfig.balance_voltage = server.arg("balance_voltage").toFloat();
  myConfig.balance_dev = server.arg("balance_dev").toFloat();
      
  myConfig.emoncms_enabled = (server.arg("emoncms_enabled").toInt() == 1) ? true : false;
  myConfig.emoncms_node_offset = server.arg("emoncms_node_offset").toInt();
  myConfig.emoncms_httpPort = server.arg("emoncms_httpPort").toInt();

  server.arg("emoncms_host").toCharArray(myConfig.emoncms_host, sizeof(myConfig.emoncms_host));
  server.arg("emoncms_url").toCharArray(myConfig.emoncms_url, sizeof(myConfig.emoncms_url));
  server.arg("emoncms_apikey").toCharArray(myConfig.emoncms_apikey, sizeof(myConfig.emoncms_apikey));

  bool current_mqtt_enabled = myConfig.mqtt_enabled;
  myConfig.mqtt_enabled = (server.arg("mqtt_enabled").toInt() == 1) ? true : false;
  server.arg("mqtt_host").toCharArray(myConfig.mqtt_host, sizeof(myConfig.mqtt_host));


  WriteConfigToEEPROM();

  server.send(200, "text/plain", "");

  if (  myConfig.mqtt_enabled != current_mqtt_enabled ) {
    //MQTT config changed so reboot
    delay(1000);
    ESP.restart();
  }
}

bool SetVoltCalib(uint8_t module, float newValue) {
  Serial.print("SetVoltCalib ");
  Serial.print(module);
  Serial.print(" = ");
  Serial.println(newValue, 6);

  for ( int a = 0; a < cell_array_max; a++) {
    if (cell_array[a].address == module) {
      if (cell_array[a].voltage_calib != newValue) {
        cell_array[a].voltage_calib = newValue;
        cell_array[a].update_calibration = true;
        return true;
      }
    }
  }
  return false; 
}
void handleSetVoltCalib() {
  uint8_t module =  server.arg("module").toInt();
  float newValue = server.arg("value").toFloat();
  if ( SetVoltCalib(module, newValue) == true ) {
      server.send(200, "text/plain", "");
    } else server.send(500, "text/plain", "");
}

bool SetTempCalib(uint8_t module, float newValue) {
    
  Serial.print("SetTempCalib ");
  Serial.print(module);
  Serial.print(" = ");
  Serial.println(newValue, 6);
  for ( int a = 0; a < cell_array_max; a++) {
    if (cell_array[a].address == module) {
      if (cell_array[a].temperature_calib != newValue) {
        cell_array[a].temperature_calib = newValue;
        cell_array[a].update_calibration = true;
      }
      server.send(200, "text/plain", "");
      return true;
    }
  }
  return false;
}

void handleSetTempCalib() {
  uint8_t module =  server.arg("module").toInt();
  float newValue = server.arg("value").toFloat();
  if ( SetTempCalib(module, newValue) == true ) {
      server.send(200, "text/plain", "");
    } else server.send(500, "text/plain", "");
}

void handleCellConfigurationJSON() {
  String json1 = "";
  
  if (cell_array_max > 0) {
    for ( int a = 0; a < cell_array_max; a++) {
      json1 += "{\"address\":" + String(cell_array[a].address)
        +",\"volt\":" + String(cell_array[a].voltage)
        +",\"voltc\":" + String(cell_array[a].voltage_calib, 6)
        +",\"temp\":" + String(cell_array[a].temperature)
        +",\"bypass\":" + String(cell_array[a].bypass_status)
        +",\"tempc\":" + String(cell_array[a].temperature_calib, 6)
        +",\"resistance\":" + String( isnan(cell_array[a].loadResistance) ? 0:cell_array[a].loadResistance, 6) 
        + "}";
      if (a < cell_array_max - 1) {
        json1 += ",";
      }
    }
  }
  server.send(200, "application/json", "[" + json1 + "]\r\n\r\n");
}

void handleSettingsJSON() {
  String json1 =   "{\"emoncms_enabled\":" + (myConfig.emoncms_enabled ? String("true") : String("false"))
                   + ",\"emoncms_node_offset\":" + String(myConfig.emoncms_node_offset)
                   + ",\"emoncms_httpPort\":" + String(myConfig.emoncms_httpPort)
                   + ",\"emoncms_host\":\"" + String(myConfig.emoncms_host) + "\""
                   + ",\"emoncms_apikey\":\"" + String(myConfig.emoncms_apikey) + "\""
                   + ",\"emoncms_url\":\"" + String(myConfig.emoncms_url) + "\""
                   + ",\"mqtt_enabled\":" + (myConfig.mqtt_enabled ? String("true") : String("false"))
                   + ",\"mqtt_host\":\"" + String(myConfig.mqtt_host) + "\""
                   + ",\"autobalance_enabled\":" + (myConfig.autobalance_enabled ? String("true") : String("false"))
                   + ",\"max_voltage\":\"" + String(myConfig.max_voltage) + "\""
                   + ",\"balance_voltage\":\"" + String(myConfig.balance_voltage) + "\""
                   + ",\"balance_dev\":\"" + String(myConfig.balance_dev) + "\""
                   + "}\r\n\r\n";
  server.send(200, "application/json", json1 );
}

void handleCellJSONData() {
  //Voltage
  String json1 = "[";
  //Temperature
  String json2 = "[";
  //Min voltage (since reset)
  String json3 = "[";
  //Max voltage (since reset)
  String json4 = "[";

  //Out of balance from average
  String json5 = "[";

  if (cell_array_max > 0) {

    //Work out the average
    float avg = 0;
    for (int a = 0; a < cell_array_max; a++) {
      avg += 1.0 * cell_array[a].voltage;
    }
    avg = avg / cell_array_max;

    for ( int a = 0; a < cell_array_max; a++) {
      json1 += String(cell_array[a].voltage);
      json2 += String(cell_array[a].temperature);
      json3 += String(cell_array[a].min_voltage);
      json4 += String(cell_array[a].max_voltage);

      json5 += String(cell_array[a].voltage - avg);

      if (a < cell_array_max - 1) {
        json1 += ",";
        json2 += ",";
        json3 += ",";
        json4 += ",";
        json5 += ",";
      }
    }
  }

  json1 += "]";
  json2 += "]";
  json3 += "]";
  json4 += "]";
  json5 += "]";


  server.send(200, "application/json", "[" + json1 + "," + json2 + "," + json3 + "," + json4 + "," + json5 + "]\r\n\r\n");
}

void handleSave() {
  String s;
  String ssid = server.arg("ssid");
  String password = server.arg("pass");

  if ((ssid.length() <= sizeof(myConfig_WIFI.wifi_ssid)) && (password.length() <= sizeof(myConfig_WIFI.wifi_passphrase))) {

    memset(&myConfig_WIFI, 0, sizeof(wifi_eeprom_settings));

    ssid.toCharArray(myConfig_WIFI.wifi_ssid, sizeof(myConfig_WIFI.wifi_ssid));
    password.toCharArray(myConfig_WIFI.wifi_passphrase, sizeof(myConfig_WIFI.wifi_passphrase));

    WriteWIFIConfigToEEPROM();

    s = htmlHeader() + F("<p>WIFI settings saved, will reboot in a few seconds.</p>") + htmlFooter();
    sendHeaders();
    server.send(200, "text/html", s);

    for (int i = 0; i < 20; i++) {
      delay(250);
      yield();
    }
    ESP.restart();

  } else {
    s = htmlHeader() + F("<p>WIFI settings too long.</p>") + htmlFooter();
    sendHeaders();
    server.send(200, "text/html", s);
  }
}

//Enter initial hotspot mode
void startAccessPoint(void) {
  delay(100);
  int n = WiFi.scanNetworks();
  if (n == 0)
    networks = "no networks found";
  else
  {
    for (int i = 0; i < n; ++i)
    {
      if (WiFi.encryptionType(i) != ENC_TYPE_NONE) {
        // Only show encrypted networks
        networks += "<option>";
        networks += WiFi.SSID(i);
        networks += "</option>";
      }
      delay(10);
    }
  }

  handleRoot();
}

void setupAccessPoint(void) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);
  int n = WiFi.scanNetworks();

  if (n == 0)
    networks = "no networks found";
  else
  {
    for (int i = 0; i < n; ++i)
    {
      if (WiFi.encryptionType(i) != ENC_TYPE_NONE) {
        // Only show encrypted networks
        networks += "<option>";
        networks += WiFi.SSID(i);
        networks += "</option>";
      }
      delay(10);
    }
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);

  if (!MDNS.begin("diybms")) {
    Serial.println("Error setting up MDNS responder!");
    //This will force a reboot of the ESP module by hanging the loop
    while (1) {
      delay(1000);
    }
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound(handleNotFound);

  server.begin();
  MDNS.addService("http", "tcp", 80);

  Serial.println("Soft AP ready on 192.168.4.1");
  // Restart after 10 minutes in case a power cut brought us here
  long m = millis();  
  while (millis()< m+(AP_TIMEOUT * 1000)) {
    HandleWifiClient();
  }
  ESP.restart();
}

void SetupManagementRedirect() {
  server.on("/", HTTP_GET, handleRedirect);
  server.on("/celljson", HTTP_GET, handleCellJSONData);
  server.on("/provision", HTTP_GET, handleProvision);
  server.on("/aboveavgbalance", HTTP_GET, handleAboveAverageBalance);
  server.on("/cancelavgbalance", HTTP_GET, handleCancelAverageBalance);
  server.on("/getmoduleconfig", HTTP_GET, handleCellConfigurationJSON);
  server.on("/getsettings", HTTP_GET, handleSettingsJSON);
  server.on("/hotspot", HTTP_GET, startAccessPoint);  // Enter hotspot mode
  server.on("/save", HTTP_POST, handleSave);
 

  server.on("/factoryreset", HTTP_POST, handleFactoryReset);
  server.on("/setloadresistance", HTTP_POST, handleSetLoadResistance);
  server.on("/setvoltcalib", HTTP_POST, handleSetVoltCalib);
  server.on("/settempcalib", HTTP_POST, handleSetTempCalib);
  server.on("/setemoncms", HTTP_POST, handleSetEmonCMS);

  server.onNotFound(handleNotFound);

  server.begin();
  MDNS.addService("http", "tcp", 80);

  Serial.println("Management Redirect Ready");
}

void HandleWifiClient() {
  server.handleClient();
}

