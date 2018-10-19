/*
 * File:   main.c
 * Author: Istvan
 *
 * Created on Oct 16, 2018, 15:43 PM
 */

#ifndef MAIN_C
#define MAIN_C

#include <string.h>
#include "HardwareProfile.h"
#include "debug.h"

/*
 * Microchip Application Library includes
 * expects V2.9a of the USB library from "Microchip Solutions v2011-07-14"
 */
#include "./USB/usb.h"
#include "./USB/usb_function_hid.h"

// Ensure we have the correct target PIC device family
#if !defined(__18F4550)
    #error "This firmware only supports the PIC18F4550 microcontroller."
#endif

// Define the globals for the USB data in the USB RAM of the PIC18F4550
#pragma udata
#pragma udata USB_VARIABLES = 0x500
unsigned char ReceivedDataBuffer[64];
unsigned char ToSendDataBuffer[64];
USB_HANDLE USBOutHandle = 0;
USB_HANDLE USBInHandle  = 0;
BOOL blinkStatusValid   = FLAG_TRUE;

// PIC18F4550 configuration for PineRace
#pragma config PLLDIV   = 5         // 20Mhz external oscillator
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
#pragma config STVREN   = ON
#pragma config LVP      = OFF
#pragma config XINST    = OFF
#pragma config CP0      = OFF
#pragma config CP1      = OFF
#pragma config CPB      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
#pragma config WRTB     = OFF
#pragma config WRTC     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
#pragma config EBTRB    = OFF
// #pragma config CCP2MX   = ON
// #pragma config ICPRT    = OFF
// #pragma config CP2      = OFF
// #pragma config CP3      = OFF
// #pragma config CPD      = OFF
// #pragma config WRT2     = OFF
// #pragma config WRT3     = OFF
// #pragma config WRTD     = OFF
// #pragma config EBTR2    = OFF
// #pragma config EBTR3    = OFF

// Private function prototypes
static void initialisePic(void);
void processUsbCommands(void);
void applicationInit(void);
void USBCBSendResume(void);
void highPriorityISRCode();
void lowPriorityISRCode();
void delayADC();

// Remap vectors for compatibilty with Microchip USB boot loaders
#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
    #define REMAPPED_RESET_VECTOR_ADDRESS		0x1000
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
    #define REMAPPED_RESET_VECTOR_ADDRESS		0x800
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
#else
    #define REMAPPED_RESET_VECTOR_ADDRESS		0x00
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
#endif

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER) || defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
    extern void _startup (void);
    #pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
    void _reset (void) {
        _asm goto _startup _endasm
    }
#endif

#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR (void){
    _asm goto highPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR (void){
    _asm goto lowPriorityISRCode _endasm
}

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER) || defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
    #pragma code HIGH_INTERRUPT_VECTOR = 0x08
    void High_ISR (void) {
        _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
    }
    #pragma code LOW_INTERRUPT_VECTOR = 0x18
    void Low_ISR (void) {
        _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
    }
#endif

#pragma code
// High-priority ISR handling function
#pragma interrupt highPriorityISRCode
void highPriorityISRCode() {
    // Application specific high-priority ISR code goes here
    #if defined(USB_INTERRUPT)
        // Perform USB device tasks
        USBDeviceTasks();
    #endif
}

// Low-priority ISR handling function
#pragma interruptlow lowPriorityISRCode
void lowPriorityISRCode() {
    // Application specific low-priority ISR code goes here
}

// Application specific variables
char debugString[64];       // String for creating debug messages
UINT ticks = 0;              // Int for counting how many ticks have passed since last USB request

// Main program entry point
void main(void) {
    // Initialise and configure the PIC ready to go
    initialisePic();

    // If we are running in interrupt mode attempt to attach the USB device
    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif
        
    // Initialise the debug log functions
    debugInitialise();

    // Show that we are up and running
    sprintf(debugString, ((const rom far char *)"PineRace USB Device initialised."));
    debugOut(debugString);
    mStatusLED0_on();

    // Main processing loop
    while(1) {
        /*
         * If we are in polling mode, the USB device tasks must be processed
         * here, otherwise the interrupt is performing this task
         */
        #if defined(USB_POLLING)
            USBDeviceTasks();
        #endif

        processUsbCommands();
    }
}

// Initialise the PIC and configure the ports for the application
static void initialisePic(void) {
    // Default all pins to digital
    ADCON1 = 0x0F;
    // Configure ports as inputs (1) or outputs(0)
    TRISA = 0b00000000;
    TRISB = 0b00000000;
    TRISC = 0b00000000;
    TRISD = 0b00000000;
    TRISE = 0b00000000;
    // Clear all ports
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    PORTD = 0b00000000;
    PORTE = 0b00000000;

    /*
     * If you have a VBUS sense pin (for self-powered devices when you
     * want to detect if the USB host is connected) you have to specify
     * your input pin in HardwareProfile.h
     */
    #if defined(USE_USB_BUS_SENSE_IO)
    	tris_usb_bus_sense = INPUT_PIN;
    #endif

    /*
     * In the case of a device which can be both self-powered and bus-powered
     * the device must respond correctly to a GetStatus (device) request and
     * tell the host how it is currently powered.
     * To do this you must device a pin which is high when self powered and low
     * when bus powered and define this in HardwareProfile.h
     */
    #if defined(USE_SELF_POWER_SENSE_IO)
    	tris_self_power = INPUT_PIN;
    #endif

    // Application specific initialisation
    applicationInit();
    // Initialise the USB device
    USBDeviceInit();
}

// Application specific device initialisation
void applicationInit(void) {
    // Initialise the outputs for the status LEDs (red, green) and the solenoid
    mInitStatusLedsAndSolenoid();

    // Initialise the analog inputs and the ADC for the 4 phototransistors
    TRISA  = 0xFF;
    ADCON0 = 0x01;
    ADCON1 = 0x09;
    ADCON2 = 0x90;
    PORTC  = PORTC | 0x1;
    // Initialize the variable holding the USB handle for the last transmission
    USBOutHandle = 0;
    USBInHandle  = 0;
}

// Delay a few miliseconds so that the ADC can safely reset the momentary value
void delayADC() {
    int i;
    for(i=0;i<=100;i++){}
}

// Process USB commands from the PineRace application
void processUsbCommands(void) {
    // Check if we are in the configured state, otherwise just return
    if((USBSuspendControl == 1) || (USBDeviceState < CONFIGURED_STATE)) {
        if (mStatusLED0_Get() == 0)
            mStatusLED0_on();
	return;
    }

    /*
     * HIDRxHandleBusy will indicate if the previous transfer is complete or not
     * and if the USB module still owns the data. If it's not busy, we can
     * process fresh commands.
     * Different cases are treated:
     * - Request for sending debug data to host
     * - Request for toggling the solenoid on or off
     * - Request for sending the solenoid's state value (on/off)
     * - Request for the analog-to-digital value of the 4 phototransistors
     */
    if(!HIDRxHandleBusy(USBOutHandle)) {
        /*
         * Turn the red led off and the green one on, so it indicates correct
         * processing and communication with host application
         */
        switch(ReceivedDataBuffer[0]) {
            case 0x10:
                // Debug information request from host
                // Copy any waiting debug text to the send data buffer
                copyDebugToSendBuffer((BYTE*)&ToSendDataBuffer[0]);
                // Transmit the response to the host
                if(!HIDTxHandleBusy(USBInHandle))
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                break;
            case 0x80:
                // Toggle the solenoid
                mSolenoid_Toggle();
                sprintf(debugString, ((const rom far char *)"Received command 0x80 from host - Toggle Solenoid"));
                debugOut(debugString);
                break;
            case 0x81:
                // Read the solenoid status
                ToSendDataBuffer[0] = mSolenoid_Get();
                // Transmit the response to the host
                if(!HIDTxHandleBusy(USBInHandle))
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                break;
            case 0x82:
                /*
                 * Request for the 4 values of the phototransistors
                 * We configure separately for each of the 4 analog inputs the
                 * ADC (analog-to-digital converter)'s 3 registers (ADCON0,
                 * ADCON1, ADCON2) (See configuration tables and details
                 * in the PIC18F4550 datasheet from page 265), we then convert
                 * the value to a 10bit (0->1024) value quantized on a voltage
                 * scale from 0V (GND) to 5V, by using the internal reference
                 * rails.
                 * We use the big endian format of putting the 10bit value
                 * (stored on 2 bytes) onto the send data buffer from the
                 * registers ADRESL and ADRESH.
                 * A small delay is needed between the separate conversions of
                 * the analog input values, so that the ADC module has time
                 * to reset it's momentary value. Otherwise, it'll transmit the
                 * first analog value foreach of the 4 values.
                 */
                ADCON0 = 0x01;
                ADCON1 = 0x09;
                ADCON2 = 0x8D;
                ADCON0bits.GO_DONE =1;
                while((ADCON0 & 0x02)==1);
                delayADC();
                ToSendDataBuffer[1] = ADRESH;
                ToSendDataBuffer[2] = ADRESL;

                ADCON0 = 0x05;
                ADCON1 = 0x09;
                ADCON2 = 0x8D;
                ADCON0bits.GO_DONE =1;
                while((ADCON0 & 0x02)==1);
                delayADC();
                ToSendDataBuffer[3] = ADRESH;
                ToSendDataBuffer[4] = ADRESL;
                
                ADCON0 = 0x09;
                ADCON1 = 0x09;
                ADCON2 = 0x8D;
                ADCON0bits.GO_DONE =1;
                while((ADCON0 & 0x02)==1);
                delayADC();
                ToSendDataBuffer[5] = ADRESH;
                ToSendDataBuffer[6] = ADRESL;

                ADCON0 = 0x0D;
                ADCON1 = 0x09;
                ADCON2 = 0x8D;
                ADCON0bits.GO_DONE =1;
                while((ADCON0 & 0x02)==1);
                delayADC();
                ToSendDataBuffer[7] = ADRESH;
                ToSendDataBuffer[8] = ADRESL;
                // Transmit the values to the host
                if(!HIDTxHandleBusy(USBInHandle))
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                break;
            default:
                break; 
        }

        mStatusLED0_off();
        mStatusLED1_on();
        ticks = 0;
        // Re-arm the OUT endpoint for the next packet
        USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);
    } else {
        if (ticks > 60000){     // About 0.6 seconds...
            mStatusLED0_on();   // Set the red led on
            mStatusLED1_off();  // Set the green led off
            ticks = 0;
    }
    ticks++;

    }
}

// USB Callback handling routines --------------------------------------

// Call back that is invoked when a USB suspend is detected
void USBCBSuspend(void) { }

// This call back is invoked when a wakeup from USB suspend is detected.
void USBCBWakeFromSuspend(void) { }

// The USB host sends out a SOF packet to full-speed devices every 1 ms.
// No need to clear UIRbits.SOFIF to 0 here. Callback caller is already doing that.
void USBCB_SOF_Handler(void) { }

// The purpose of this callback is mainly for debugging during development.
// Check UEIR to see which error causes the interrupt.
// No need to clear UEIR to 0 here.
// Callback caller is already doing that.
void USBCBErrorHandler(void) { }

// Check other requests callback
void USBCBCheckOtherReq(void) {
    USBCheckHIDRequest();
}

// Callback function is called when a SETUP, bRequest: SET_DESCRIPTOR request arrives.
// You must claim session ownership if supporting this request
void USBCBStdSetDscHandler(void) { }

//This function is called when the device becomes initialized
void USBCBInitEP(void) {
    // Enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    // Re-arm the OUT endpoint for the next packet
    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);
}

// Send resume call-back
void USBCBSendResume(void) {
    static WORD delay_count;

    // Verify that the host has armed us to perform remote wakeup.
    if(USBGetRemoteWakeupStatus() == FLAG_TRUE) {
        // Verify that the USB bus is suspended (before we send remote wakeup signalling).
        if(USBIsBusSuspended() == FLAG_TRUE) {
            USBMaskInterrupts();

            // Bring the clock speed up to normal running state
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FLAG_FALSE;

            // Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            // device must continuously see 5ms+ of idle on the bus, before it sends
            // remote wakeup signalling.  One way to be certain that this parameter
            // gets met, is to add a 2ms+ blocking delay here (2ms plus at
            // least 3ms from bus idle to USBIsBusSuspended() == FLAG_TRUE, yeilds
            // 5ms+ total delay since start of idle).
            delay_count = 3600U;
            do {
                delay_count--;
            } while(delay_count);

            // Start RESUME signaling for 1-13 ms
            USBResumeControl = 1;
            delay_count = 1800U;
            do {
                delay_count--;
            } while(delay_count);
            USBResumeControl = 0;

            USBUnmaskInterrupts();
        }
    }
}

// USB callback function handler
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size) {
    switch(event)
    {
        case EVENT_TRANSFER:
            // Application callback tasks and functions go here
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        default:
            break;
    }
    return FLAG_TRUE;
}

#endif
