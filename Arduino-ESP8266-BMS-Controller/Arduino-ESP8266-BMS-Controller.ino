

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
#include "mqtt.h"
#include "WebServiceSubmit.h"
#include "bst900.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <INA.h>      //INA current sense


#define DEBUG 1

//Allow up to 20 modules
cell_module cell_array[20];
int cell_array_index = -1;
int cell_array_max = 0;
unsigned long next_submit;
bool runProvisioning;
uint8_t i2cstatus;

extern bool manual_balance;
bool max_enabled = false;
int balance_status = 0;
// 0=No balancing 1=Manual balancing started 2=Auto Balancing enabled 3=Auto Balancing enabled and bypass happening 4=A module is over max voltage

bool charging= false;
bool discharging= false;
uint8_t chargecurrent = 0;    //Charge current in Digipot wiper steps for analogue control of boost converter.
uint16_t bstcurrent = 0;      //Charge current in mA for serial control of BST900 boost converter if fitted
uint8_t dischargepower = 0;
uint8_t nodered_timeout = 0;   //Turn off charger if this expires

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

String bstbuf = ""; //Input buffer for serial
bool inputready = false;


EmonCMS emoncms;

os_timer_t myTimer;

INA_Class INA;  
//INA226 INA;
uint8_t devicesFound = 0;


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

//WiFiClient espClient;
//PubSubClient mqttclient(espClient);

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
  //Start off assuming good data
  module->valid_values = true;
  
  module->voltage = cell_read_voltage(module->address);
  if ( i2cstatus != 2 ) {      //Is the data bad?          
    module->valid_values = false;
  }
  uint16_t data = cell_read_board_temp(module->address);
  if ( i2cstatus == 2 ) {      //Is the data good?
    float y = tempconvert(data);
    if ( !isnan(y) ) module->temperature = y;  //Check it is a valid number
  }
  data = cell_read_bypass_enabled_state(module->address);
  if ( i2cstatus == 1 ) {
    module->bypass_status = cell_read_bypass_enabled_state(module->address);
  }

  if (module->voltage >= 0 && module->voltage <= 5000) {

    if ( module->voltage > module->max_voltage && module->valid_values == true) {
      module->max_voltage = module->voltage;
    }
    if ( module->voltage < module->min_voltage && module->valid_values == true) {
      module->min_voltage = module->voltage;
    }

  } else {
    module->valid_values = false;
  }   

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
    mqttprint("runProvisioning");
    uint8_t newCellI2CAddress = provision();

    if (newCellI2CAddress > 0) {
      mqttprint("Found "+ String(newCellI2CAddress));

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
#ifdef DEBUG
  String buff="";
  for (int i = 0; i < length; i++) {
      buff += (char)payload[i];
  }
  mqttprint("Message " + buff);
#endif
  StaticJsonBuffer<200> jsonBuffer;     //Decode JSON
  JsonObject& root = jsonBuffer.parseObject(payload);
  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }
// TODO - Do stuff
  uint16_t Avg;
  uint8_t address;
  
  float value;
  if ( root.containsKey("address") ) address = root["address"];
  if ( root.containsKey("value") ) value = root["value"];
  uint8_t command = root["command"];
  switch(command) {
    case 1:                                     // Command 1 - resetESP
      mqttprint("Restarting Controller");
      delay(1000);
      ESP.restart();
      break;
    case 2:                                     // Command 2 - Start Avg balance
      Avg = AboveAverageBalance();
      mqttprint("Started balancing to  " + String(Avg) + " mV");
      break;
    case 3:                                     // Command 3 - Cancel Avg balance
      mqttprint("Stop balancing");
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
    case 7:                                     // Command 7 - Provision
      runProvisioning = true;
      break;
    case 8:                                 //Command 8 - Report configuration
    report_configuration();
    break;
    case 10:                                            //Close Charger Relay on D5
      digitalWrite(D5, HIGH);               //Turn on charger
      charging= true;
      discharging= false;
      nodered_timeout = 20;        // if no new command received in 10 mins turn off charger
      if( root.containsKey("chargecurrent") == true ) {
        chargecurrent = root["chargecurrent"];
        set_charge_current(chargecurrent);      // Set charger digipot
      }
      if( root.containsKey("bstcurrent") == true ) {
        bstcurrent = root["bstcurrent"];
        setcurrent_bst900(bstcurrent);      // Set boost converter current
      }      
      set_inverter_power(0);                //Disable inverter if active
      digitalWrite(D7, LOW);                  // Disable discharging relay
      break;
    case 14:                     //Immediate shutdown - Turns off charging and discharging without delay
      digitalWrite(D7, LOW);                  // Disable discharging relay
      digitalWrite(D5, LOW);                  // Disable charging relay
      nodered_timeout = 0;
    case 11: 
    case 13:    //Cancel Charging or Discharging
      charging= false;
      discharging= false;
      if (nodered_timeout > 10)  nodered_timeout = 10; //Reduce timeout on expiry relay releases
      chargecurrent = 0;
      dischargepower = 0;
      set_charge_current(0);    //Turn off digipots
      set_inverter_power(0);
      break;
    case 12:                                            //Enable Inverter
      charging= false;
      discharging= true;
      digitalWrite(D5, LOW);       // Cancel charging if present
      set_charge_current(0);
      nodered_timeout = 20;        // if no new command received in 10 mins turn off inverter
      if( root.containsKey("dischargepower") == true ) dischargepower = root["dischargepower"];
      set_inverter_power(dischargepower);      // Set inverter digipot
      if (dischargepower > 0 ) digitalWrite(D7, HIGH);       // Enable discharging relay
      break;
    case 18:                      //Pass text to serial interface to control boost converter
      if( root.containsKey("serial") == true ) {
        String text = root["serial"];
        bst_send_text(text);
      }
    default:
      break;
    
  }
}


float tempconvert(float rawtemp) {
  Vout=Vref*((float)(rawtemp)/1024.0); // calc voltage at ADC pin
  Rp= R4*((Vin/Vout) - 1);    //calc value of Thermistor in parallel with R3
  Rntc=1/((1/Rp)-(1/R3));     //calc Thermistor resistance
  TempK=(beta/log(Rntc/Rinf)); // calc temperature in Kelvin
  TempC=TempK-273.15;
  return TempC;
}

void setup() {
  Serial.begin(19200);           // start serial for output
  Serial.println();
  bstbuf.reserve(64);
  //D4 is LED
  pinMode(D4, OUTPUT);
  LED_OFF;
  //D5 is Charger Relay output
  pinMode(D5, OUTPUT);
  digitalWrite(D5, LOW);
  //D7 is Inverter Relay output
  pinMode(D7, OUTPUT);
  digitalWrite(D7, LOW); 
  //D0 is Smoke Detector input
  pinMode(D0, INPUT);
  
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

//Before scanning i2c bus wait for 3 seconds 
  delay(3000);
#ifdef DEBUG
  mqttprint("Scanning I2c bus");
#endif
  scani2cBus();

  //Ensure we service the cell modules every 0.5 seconds
  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, 1000, true);

  set_charge_current(0);    //Turn off charger digipot
  set_inverter_power(0);  //Turn off inverter digipot

  //Enable Digipots (removes Shutdown setting)
  enable_digipot();

  
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
    mqttreconnect();
  }
  
  mqttprint ("Configured Maximum voltage : " + String(myConfig.max_voltage)); // Report max pack voltage

  
  //Setup INA current sensor
  mqttprint(" - Searching & Initializing INA devices");
  devicesFound = INA.begin(20,SHUNT);                                         // Set expected 20 Amp max and resistor 1.5 mohm   //
  mqttprint(" - Detected "+String(devicesFound)+" INA devices on the I2C bus");
  INA.setBusConversion(10000);                                                 // Maximum conversion time 8.244ms  //
  INA.setShuntConversion(10000);                                               // Maximum conversion time 8.244ms  //
  INA.setAveraging(1024);                                                   // Average each reading n-times     //
  INA.setMode(INA_MODE_CONTINUOUS_BOTH);                                      // Bus/shunt measured continuously
  String bus_readings= "";
  for (uint8_t i=0;i<devicesFound;i++) // Loop through all devices
  {
    bus_readings = String(i)+ "  ";
    bus_readings += String(INA.getDeviceName(i)) + "  ";
    bus_readings += String((float)INA.getBusMilliVolts(i)/1000.0) + " V  ";    // convert mV to Volts
    bus_readings += String((float)INA.getBusMicroAmps(i)/1000.0) + " mA  ";     // convert uA to Milliamps
    bus_readings += String((float)INA.getBusMicroWatts(i) / 1000.0) + " mW  "; // convert uA to Milliwatts
    mqttprint(bus_readings);
} // for-next each INA device loop

  
  //Setup OTA Updates
  ArduinoOTA.setHostname("diyBMS");
    ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  

  SetupManagementRedirect();
}

void loop() {
  ArduinoOTA.handle();
  for ( int j=0; j<=3; j++) {
    HandleWifiClient();
    if(myConfig.mqtt_enabled == true) mqttclient.loop();
    yield();
    delay(250);
  }


   if (cell_array_max > 0) {
    
    if ((millis() > next_submit) && (WiFi.status() == WL_CONNECTED)) {
      // Ensure we are connected to MQTT
      if (myConfig.mqtt_enabled == true && !mqttclient.connected()) {
      mqttreconnect();
      }
      //Update emoncms every 30 seconds
      emoncms.postData(myConfig, cell_array, cell_array_max);

      //Check nodered timeout and turn off (dis)charging if it has expired
      if ( nodered_timeout > 0 ) {
        nodered_timeout -= 1;
        if ( nodered_timeout == 0 ) {
          digitalWrite(D5, LOW);  //Release charger relay
          digitalWrite(D7, LOW);  //Release inverter relay
          charging= false;
          discharging= false;
          nodered_timeout = 0;
          chargecurrent = 0;        //Turn off charger digipot
          dischargepower = 0;     //Turn off inverter digipot
        }
      }
      set_charge_current(chargecurrent);            // Refresh charger digipot just in case
      set_inverter_power(dischargepower);      // Refresh inverter digipot just in case
      
      //Read bus battery voltage and current
      updatebus();
      
#ifdef DEBUG
        //Debug messages to MQTT
      String debug_message="";
      for ( int a = 0; a < cell_array_max; a++) {
        debug_message += String(cell_array[a].address) + F(":") + String(cell_array[a].voltage) + F(":");;
        debug_message += String(cell_array[a].temperature) + F(":") + String(cell_array[a].bypass_status) + F(":");
      }
      mqttprint(debug_message);

#endif
      
      for (int a = 0; a < cell_array_max; a++) {
        if (cell_array[a].voltage >= myConfig.max_voltage*1000 && cell_array[a].voltage != 65535) {
          cell_array[a].balance_target = myConfig.max_voltage*1000; 
           max_enabled = true;
           balance_status = 4;
#ifdef DEBUG
           mqttprint("Cell voltage : " + String(cell_array[a].voltage) + " Balance Target = " +  cell_array[a].balance_target  );
#endif           
        }
      }
      if (max_enabled!=true) avg_balance();
      max_enabled = false;
      next_submit = millis() + 30000;
    }
  }
 
  if (Serial.available()) bst_process();    //Data has arrived on serial interface from boost converter
  if (inputready ) {
    inputready=false;
    mqttprint ("text:"+ bstbuf);
    bstbuf = "";
  }

  //if(digitalRead(D0) == true ) {    //Smoke detected, shutdown and sound alarms
  //  digitalWrite(D5, LOW);          //Shutdown charger
                                    //Shutdown inverter
  //  mqttclient.publish(MQTT_TOPIC, "{smoke_alarm:1}");    //Send alarm to MQTT
  //}
}//end of loop

