/*
 * File:   debug.h
 * Author: Istvan
 *
 * Created on Oct 16, 2018, 15:43 PM
 */

#ifndef DEBUG_H
#define DEBUG_H

/*
 * Microchip Application Library includes
 * expects V2.9a of the USB library from "Microchip Solutions v2011-07-14"
 */
#include "./USB/usb.h"
#include "./USB/usb_function_hid.h"

// Set the size of the debug information buffer in characters
#define DEBUGBUFFERSIZE 128

// Function prototypes
void debugInitialise(void);
void debugOut(char*);
void copyDebugToSendBuffer(BYTE* sendDataBuffer);

#endif