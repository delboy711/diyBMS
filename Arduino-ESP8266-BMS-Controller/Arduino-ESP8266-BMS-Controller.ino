
/* ____  ____  _  _  ____  __  __  ___
  (  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
   )(_) )_)(_  \  /  ) _ < )    ( \__ \
  (____/(____) (__) (____/(_/\/\_)(___/

  (c) 2017/2018 Stuart Pittaway

  This is the code for the controller - it talks to the cell modules over isolated i2c bus

  This code runs on ESP-8266-12E (NODE MCU 1.0) and compiles with Arduino 1.8.5 environment

  Arduino settings
  NodeMCU 1.0 (ESP-8266-12E module), Flash 4M (3MSPIFF), CPU 80MHZ

  Setting up ESP-8266-12E (NODE MCU 1.0) on Arduino
  http://www.instructables.com/id/Programming-a-HTTP-Server-on-ESP-8266-12E/

  c:\Program Files (x86)\PuTTY\putty.exe -serial COM6 -sercfg 115200,8,n,1,N

  i2c CLOCK STRETCH
  https://github.com/esp8266/Arduino/issues/698
*/

extern "C"
{
#include "user_interface.h"
}

#define LED_ON digitalWrite(D4, LOW)
#define LED_OFF digitalWrite(D4, HIGH)

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#include "bms_values.h"
#include "i2c_cmds.h"
#include "settings.h"
#include "SoftAP.h"
#include "WebServiceSubmit.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

//Allow up to 24 modules
cell_module cell_array[24];
int cell_array_index = -1;
int cell_array_max = 0;
unsigned long next_submit;
bool runProvisioning;
uint8_t i2cstatus;

extern bool manual_balance;
bool max_enabled = false;
int balance_status = 0;
// 0=No balancing 1=Manual balancing started 2=Auto Balancing enabled 3=Auto Balancing enabled and bypass happening 4=A module is over max voltage

//Configuration for thermistor conversion
//use the datasheet to get this data.
//https://www.instructables.com/id/NTC-Temperature-Sensor-With-Arduino/



// Parameters to calculate NTC resistance and temperature.
// Figures are based on thermistor B57891M0103K000 from EPCOS (TDK)
const float Vin=3.3;     // [V] 
const float Vref=2.56;   // Reference voltage used by TinyAT ADC       
//const float R3=22000;  // Resistor R3 [ohm] (Schematic says 20K, but I only had 22k)
const float R3=12692;    // Resistor R3 [ohm]    //Readings seem out, so adjust R3 here to compensate
const float R4=10000;    // Resistor R4 [ohm]
const float R0=10000;    // Resistance of thermistor at 25deg C T0 [ohm]
const float T0=298.15;   // T0 in Kelvin [K] 25deg C
const float T1=273.15;   // [K] in datasheet 0º C
const float T2=373.15;   // [K] in datasheet 100° C
const float RT1=32014;   // [ohms]  resistance in T1 from datasheet
const float RT2=697.22;  // [ohms]   resistance in T2 from datasheet
float beta=3950;    // initial parameters [K]  from datasheet
float Rinf=0.0;    // initial parameters [ohm]   
float Vout=0.0;    // Vout in A0 
float Rntc=0.0;    // Current resistance of NTC
float Rp=0.0;      // value of R3 and Rntc in parallel at current temp
float TempK=0.0;   // variable output
float TempC=0.0;   // variable output


EmonCMS emoncms;

os_timer_t myTimer;

void avg_balance() {
  uint16_t avgint = 0;
  float avgintf = 0.0;
       
  if ((myConfig.autobalance_enabled == true) && (manual_balance == false)) {
    
    if (cell_array_max > 0) {
    //Work out the average 
    float avg = 0;
    for (int a = 0; a < cell_array_max; a++) {
      avgintf = cell_array[a].voltage/1000.0;
      avg += 1.0 * avgintf;
    }
    avg = avg / cell_array_max;

    avgint = avg;

    balance_status = 2;
    
    //Serial.println("Average cell voltage is currently : " + String(avg*1000));
    //Serial.println("Configured balance voltage : " + String(myConfig.balance_voltage*1000));

    if ( avg >= myConfig.balance_voltage )  {
      for ( int a = 0; a < cell_array_max; a++) {
        if (cell_array[a].voltage > avg*1000) {
          cell_array[a].balance_target = avg*1000;
          balance_status = 3;
        }
      } 
    }  else {
      for (int a = 0; a < cell_array_max; a++) {
        command_set_bypass_voltage(cell_array[a].address,0); }}
        } 
  } else  balance_status = 0;    
}



// Set up  MQTT
char* mqttServerName = myConfig.mqtt_host;

WiFiClient espClient;
PubSubClient mqttclient(espClient);

void print_module_details(struct  cell_module *module) {
  Serial.print("Mod: ");
  Serial.print(module->address);
  Serial.print(" V:");
  Serial.print(module->voltage);
  Serial.print(" VC:");
  Serial.print(module->voltage_calib);
  Serial.print(" T:");
  Serial.print(module->temperature);
  Serial.print(" TC:");
  Serial.print(module->temperature_calib);
  Serial.print(" R:");
  Serial.print(module->loadResistance);
  Serial.println("");
}

void check_module_quick(struct  cell_module *module) {
  uint16_t data = cell_read_voltage(module->address);
  if ( i2cstatus == 2 ) {      //Is the data good?
    module->voltage = data;
    module->error_count -= 1;   //Good poll so decrement error count
  } else module->error_count += 1;
  data = cell_read_board_temp(module->address);
  if ( i2cstatus == 2 ) {      //Is the data good?
    float y = tempconvert(data);
    if ( !isnan(y) ) module->temperature = y;  //Check it is a valid number
    module->error_count -= 1;
  } else module->error_count += 1;
  data = cell_read_bypass_enabled_state(module->address);
  if ( i2cstatus == 1 ) {
    module->bypass_status = cell_read_bypass_enabled_state(module->address);
    module->error_count -= 1;
  } else module->error_count += 1;

  if (module->voltage >= 0 && module->voltage <= 5000) {

    if ( module->voltage > module->max_voltage || module->valid_values == false) {
      module->max_voltage = module->voltage;
    }
    if ( module->voltage < module->min_voltage || module->valid_values == false) {
      module->min_voltage = module->voltage;
    }

    module->valid_values = true;
  } else {
    module->valid_values = false;
  }

// I2C errors
  if ( module->error_count < 0 ) {
    module->error_count = 0; //If a negative number then good polling so clear count
    module->lost_communication = false;    // Clear error status
  }
  if ( module->error_count >= BAD_I2C_POLLS ) {
    module->lost_communication = true;
  }
  if ( module->error_count > BAD_I2C_POLLS+6 ) module->error_count = BAD_I2C_POLLS+6;    //Limit counter    

  if(myConfig.mqtt_enabled == true) updatemqtt(module);   //Publish to MQTT
}

void check_module_full(struct  cell_module *module) {
  check_module_quick(module);
  module->voltage_calib = cell_read_voltage_calibration(module->address);
  module->temperature_calib = cell_read_temperature_calibration(module->address);
  module->loadResistance = cell_read_load_resistance(module->address);
}

void timerCallback(void *pArg) {
  LED_ON;

  if (runProvisioning) {
    Serial.println("runProvisioning");
    uint8_t newCellI2CAddress = provision();

    if (newCellI2CAddress > 0) {
      Serial.print("Found ");
      Serial.println(newCellI2CAddress);

      cell_module m2;
      m2.address = newCellI2CAddress;
      cell_array[cell_array_max] = m2;
      cell_array[cell_array_max].min_voltage = 0xFFFF;
      cell_array[cell_array_max].max_voltage = 0;
      cell_array[cell_array_max].balance_target = 0;
      cell_array[cell_array_max].valid_values = false;

      //Dont attempt to read here as the module will be rebooting
      //check_module_quick( &cell_array[cell_array_max] );
      cell_array_max++;
    }

    runProvisioning = false;
    return;
  }


  //Ensure we have some cell modules to check
  if (cell_array_max > 0 && cell_array_index >= 0) {


    if (cell_array[cell_array_index].update_calibration) {

      if (cell_array[cell_array_index].factoryReset) {
        command_factory_reset(cell_array[cell_array_index].address);
      } else {

        //Check to see if we need to configure the calibration data for this module
        command_set_voltage_calibration(cell_array[cell_array_index].address, cell_array[cell_array_index].voltage_calib);
        command_set_temperature_calibration(cell_array[cell_array_index].address, cell_array[cell_array_index].temperature_calib);

        command_set_load_resistance(cell_array[cell_array_index].address, cell_array[cell_array_index].loadResistance);
      }
      cell_array[cell_array_index].update_calibration = false;
    }

    check_module_quick( &cell_array[cell_array_index] );

    if (cell_array[cell_array_index].balance_target > 0) {
      command_set_bypass_voltage(cell_array[cell_array_index].address, cell_array[cell_array_index].balance_target);
      cell_array[cell_array_index].balance_target = 0;
    }

    cell_array_index++;
    if (cell_array_index >= cell_array_max) {
      cell_array_index = 0;
    }
  }


  LED_OFF;
} // End of timerCallback

void scani2cBus() {

  cell_array_index = 0;

  //We have 1 module
  cell_array_max = 0;
  //Scan the i2c bus looking for modules on start up
  for (uint8_t address = DEFAULT_SLAVE_ADDR_START_RANGE; address <= DEFAULT_SLAVE_ADDR_END_RANGE; address++ )
  {
    if (testModuleExists(address) == true) {
      //We have found a module
      cell_module m1;
      m1.address = address;
      //Default values
      m1.valid_values = false;
      m1.min_voltage = 0xFFFF;
      m1.max_voltage = 0;
      cell_array[cell_array_max] = m1;

      check_module_full( &cell_array[cell_array_max] );

      //Switch off bypass if its on
      command_set_bypass_voltage(address, 0);

      print_module_details( &cell_array[cell_array_max] );
      delay(50);
      cell_array_max++;;
    }
  }
}

// MQTT functions
void mqttcallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  StaticJsonBuffer<200> jsonBuffer;     //Decode JSON
  JsonObject& root = jsonBuffer.parseObject(payload);
  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }
// TODO - Do stuff
  uint8_t address;
  float value;
  if ( root.containsKey("address") ) address = root["address"];
  if ( root.containsKey("value") ) value = root["value"];
  uint8_t command = root["command"];
  switch(command) {
    case 1:                                     // Command 1 - resetESP
      Serial.println("Restarting Controller");
      delay(1000);
      ESP.restart();
      break;
    case 2:                                     // Command 2 - Start Avg balance
      Serial.println("Starting balancing");
      AboveAverageBalance();
      break;
    case 3:                                     // Command 3 - Cancel Avg balance
      CancelAverageBalance();
      break;
    case 4:                                     // Command 4 - Set Voltage Calibration
      value = root["value"];
      SetVoltCalib(address, value);
      break;
    case 5:                                     // Command 5 - Set Temp Calibration
      value = root["value"];
      SetTempCalib(address, value);
      break;
    case 6:                                     // Command 6 - General Settings
      if( root.containsKey("autobalance") == true ) myConfig.autobalance_enabled = root["autobalance"];
      if( root.containsKey("maxvolts") == true ) myConfig.max_voltage = root["maxvolts"];
      if( root.containsKey("balance_voltage") == true ) myConfig.balance_voltage = root["balance_voltage"];
      if( root.containsKey("balance_dev") == true ) myConfig.balance_dev = root["balance_dev"];      
      if( root.containsKey("emoncms_enabled") == true ) myConfig.emoncms_enabled = root["emoncms_enabled"];
      WriteConfigToEEPROM();
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
    case 10:
      break;
    default:
      break;
    
  }
}

void updatemqtt( struct  cell_module *module ) {
  DynamicJsonBuffer jsonBuffer(400);                              //Allocate buffer for JSON
  JsonObject& root = jsonBuffer.createObject();  
  root["address"] = String(module->address);
  if (module->valid_values == true) {                     //Do not send junk data
    root["voltage"] = String(module->valid_values ? module->voltage : 0);
    root["temperature"] = String(module->temperature);
  } 
  root["bypass_status"] = String(module->bypass_status);
  root["errors"] = String(module->error_count);
  root["comms_alarm"] = String(module->lost_communication);
  char jsonout[256];
  root.printTo(jsonout);
  mqttclient.publish(MQTT_TOPIC, jsonout, root.measureLength());   //Publish to MQTT
  return;
}

void mqttreconnect() {
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "diyBMS-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttclient.connect(clientId.c_str())) {
      Serial.println("connected");
      mqttclient.subscribe(MQTT_COMMAND_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

float tempconvert(float rawtemp) {
  Vout=Vref*((float)(rawtemp)/1024.0); // calc voltage at ADC pin
  Rp= R4*((Vin/Vout) - 1);    //calc value of Thermistor in parallel with R3
  Rntc=1/((1/Rp)-(1/R3));     //calc Thermistor resistance
  TempK=(beta/log(Rntc/Rinf)); // calc temperature in Kelvin
  TempC=TempK-273.15;
  /*
  Serial.print("rawtemp = ");
  Serial.print(rawtemp);
  Serial.print("   Vout = ");
  Serial.print(Vout);
  Serial.print("   Rp = ");
  Serial.print(Rp);
  Serial.print("   Rntc = ");
  Serial.println(Rntc);
*/
  return TempC;
}

void setup() {
  Serial.begin(19200);           // start serial for output
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  //D4 is LED
  pinMode(D4, OUTPUT);
  LED_OFF;
  
  //Thermistor setup
  beta=(log(RT1/RT2))/((1/T1)-(1/T2));
  Rinf=R0*exp(-beta/T0);
  
  Serial.println(F("DIY BMS Controller Startup"));

  initWire();

  if (LoadConfigFromEEPROM()) {
    Serial.println(F("Settings loaded from EEPROM"));
  } else {
    Serial.println(F("We are in initial power on mode (factory reset)"));
    FactoryResetSettings();
  }

  if (LoadWIFIConfigFromEEPROM()) {
    Serial.println(F("Connect to WIFI AP"));
    /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
      would try to act as both a client and an access-point and could cause
      network-issues with your other WiFi-devices on your WiFi-network. */
    WiFi.mode(WIFI_STA);
    WiFi.begin(myConfig_WIFI.wifi_ssid, myConfig_WIFI.wifi_passphrase);
  } else {
    //We are in initial power on mode (factory reset)
    setupAccessPoint();
  }

  scani2cBus();

  //Ensure we service the cell modules every 0.5 seconds
  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, 1000, true);

  //Check WIFI is working and connected
  Serial.print(F("WIFI Connecting"));

  //Attempt to connect to WIFI for timeout period and then start hotspot if unsuccessful
  for ( int i=0; i<=WIFI_TIMEOUT*4; i++ )
  {
    delay(250);
    Serial.println( WiFi.status() );
    if (WiFi.status() == WL_CONNECTED) break;
  }
  // If WiFi cannot connect, enter AP mode
  if (WiFi.status() != WL_CONNECTED) setupAccessPoint();
  Serial.print(F(". Connected IP:"));
  Serial.println(WiFi.localIP());
  //Setup MQTT
  if(myConfig.mqtt_enabled == true) {
    mqttclient.setServer(mqttServerName, 1883);
    mqttclient.setCallback(mqttcallback);
  }

  SetupManagementRedirect();
}

void loop() {
  // Ensure we are connected to MQTT
  if (myConfig.mqtt_enabled == true && !mqttclient.connected()) {
    mqttreconnect();
  }
  for ( int j=0; j<=3; j++) {
    HandleWifiClient();
    if(myConfig.mqtt_enabled == true) mqttclient.loop();
    yield();
    delay(250);
  }


   if (cell_array_max > 0) {

    
        for ( int a = 0; a < cell_array_max; a++) {
          Serial.print(cell_array[a].address);
          Serial.print(':');
          Serial.print(cell_array[a].voltage);
          Serial.print(':');
          Serial.print(cell_array[a].temperature);
          Serial.print(':');
          Serial.print(cell_array[a].bypass_status);
          Serial.print(':');
          Serial.print(cell_array[a].error_count);
          Serial.print(' ');
        }
        Serial.println();
    
    if ((millis() > next_submit) && (WiFi.status() == WL_CONNECTED)) {
      //Update emoncms every 30 seconds
      emoncms.postData(myConfig, cell_array, cell_array_max);
      
      Serial.println("Configured Maximum voltage : " + String(myConfig.max_voltage));
      
      for (int a = 0; a < cell_array_max; a++) {
        if (cell_array[a].voltage >= myConfig.max_voltage*1000) {
          cell_array[a].balance_target = myConfig.max_voltage*1000; 
           max_enabled = true;
           balance_status = 4;
           Serial.println("Cell voltage : " + String(cell_array[a].voltage) + " Balance Target = " +  cell_array[a].balance_target  );
        }
      }
      if (max_enabled!=true) avg_balance();
      max_enabled = false;
      next_submit = millis() + 30000;
    }
  }
}//end of loop

