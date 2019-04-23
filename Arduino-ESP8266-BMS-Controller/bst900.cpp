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

bool bst_output_enabled = false;

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
      if ( bstbuf.endsWith("OK") || bstbuf.endsWith("E!") ) {
        inputready = true;
        return;
      }
      character = ' ';    //Replace LF with spaces
    }
    bstbuf += character;
    yield();
  } 
}

// Establish contact with BST900
boolean bst900_init() {
  if (!Serial) return false;
  serialFlush();
  Serial.write('\n');
}

/* could be issues if bst fails to respond to a command
 *  suggest putting sequence number on each message to assist with ensuring response.
 *  Or else reflect command back with OK.
 */

//Disable boost converter output
void stop_bst900() {
  //if (bst_output_enabled == true) Serial.print("OUTPUT 0\n");
  Serial.print("OUTPUT 0\n");
  delay(20);    
  bst_output_enabled = false;
}


//Set charger current in A
boolean setcurrent_bst900 (String chargecurrent) {
  if ( chargecurrent == "0" ) {
    stop_bst900();
    return true;
  }
  Serial.print("CURRENT " + chargecurrent + '\n');
  delay(20);
  //if (bst_output_enabled == false) Serial.print("OUTPUT 1\n");
  Serial.print("OUTPUT 1\n");
  bst_output_enabled = true; 
}

//Send text string to BST900
void bst_send_text(String text) {
  Serial.println(text);
  delay(5);

}





