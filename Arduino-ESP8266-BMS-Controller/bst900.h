/* *****************************
 *  Functions to interface with a BST900 boost converter
 *  over a serial interface
 *  
 *  Derek Jennings
 *  Jan 2019
 *  Licensed under GPL v3
 */

#ifndef bst900_H_
#define bst900_H_


void bst_process();
boolean bst900_init();
boolean setcurrent_bst900 (uint16_t chargecurrent);
void bst_send_text(String);
void serialFlush();


#endif
