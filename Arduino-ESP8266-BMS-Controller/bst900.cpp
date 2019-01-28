/* *****************************
 *  Functions to interface with a BST900 boost converter
 *  over a serial interface
 *  
 *  Derek Jennings
 *  Jan 2019
 *  Licensed under GPL v3
 */

   
#include "Arduino.h"

extern String bstbuf;
extern bool inputready;
//extern void mqttprint(String);

// Empty input buffer
void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
  inputready = false;
  bstbuf="";
} 

//Process data arriving on serial interface
void bst_process() {
  while(Serial.available()) {
    char character = Serial.read();
    if(character == '\r') continue;
    if(character == '\n') {
      inputready = true;
      return;
    }
    bstbuf += character;
  } 
}

// Establish contact with BST900
boolean bst900_init() {
  if (!Serial) return false;
  serialFlush();
  Serial.write('\n');
}



//Set charger current in mA
boolean setcurrent_bst900 (uint16_t chargecurrent) {
  Serial.print("CURRENT " + String(chargecurrent) + "\n");
}

//Send text string to BST900
void bst_send_text(String text) {
  Serial.println(text);

}


