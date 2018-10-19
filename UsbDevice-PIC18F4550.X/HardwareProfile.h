/*
 * File:   HardwareProfile.h
 * Author: Istvan
 *
 * Created on May 27, 2014, 4:42 PM
 */

#ifndef HARDWAREPROFILE_H
#define	HARDWAREPROFILE_H

// USB stack hardware selection options
/*
 * This section is the set of definitions required by the MCHPFSUSB framework.
 * Uncomment the following define if you wish to use the self-power sense
 * feature and define the port, pin and tris for the power sense pin below:
 */
// #define USE_SELF_POWER_SENSE_IO
#define tris_self_power TRISAbits.TRISA2
#if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power PORTAbits.RA2
#else
    #define self_power 1
#endif

/*
 * Uncomment the following define if you wish to use the bus-power sense
 * feature and define the port, pin and tris for the power sense pin below:
 */
//#define USE_USB_BUS_SENSE_IO
#define tris_usb_bus_sense TRISAbits.TRISA1
#if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE PORTAbits.RA1
#else
    #define USB_BUS_SENSE 1
#endif

/*
 * Uncomment the following line to make the output HEX of this project
 * work with the MCHPUSB Bootloader
 */
//#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER

/*
 * Uncomment the following line to make the output HEX of this project
 * work with the HID Bootloader
 */
#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER

// Application specific hardware definitions

// Oscillator frequency (48Mhz with a 20Mhz external oscillator)
#define CLOCK_FREQ 48000000
// Device Vendor Indentifier (VID) (0x04D8 is Microchip's VID)
#define USB_VID	0x04D8
// Device Product Indentifier (PID) (0x0042)
#define USB_PID	0x0042
// Manufacturer string descriptor
#define MSDLENGTH 12
#define MSD 'K','ö','b','l','e',' ','I','s','t','v','á','n'
// Product String descriptor
#define PSDLENGTH 8
#define PSD 'P','i','n','e','R','a','c','e'
// Device serial number string descriptor
#define DSNLENGTH 7
#define DSN 'K','I','P','R','4','.','0'

// Common definitions
#define INPUT_PIN 1
#define OUTPUT_PIN 0
#define FLAG_TRUE 1
#define FLAG_FALSE 0

/*
 * Comment out the following line if you do not want the debug
 * feature of the firmware (saves code and RAM space when off)
 * Note: if you use this feature you must compile with the large
 * memory model on (for 24-bit pointers) so that the sprintf()
 * function will work correctly.  If you do not require debug it's
 * recommended that you compile with the small memory model and
 * remove any references to <strings.h> and sprintf().
 */
#define DEBUGON

// PIC to hardware pin mapping and control macros

// Led and solenoid control macros
#define mInitStatusLedsAndSolenoid()    LATB &= 0b00000110; TRISB &= 0b00000000;
#define mSolenoid                       LATBbits.LATB0
#define mStatusLED0	                LATBbits.LATB1
#define mStatusLED1	                LATBbits.LATB2

#define mSolenoid_on()                  mSolenoid = 1;
#define mSolenoid_off()                 mSolenoid = 0;
#define mSolenoid_Toggle()              mSolenoid = !mSolenoid;
#define mSolenoid_Get()                 mSolenoid

//Red led
#define mStatusLED0_on()                mStatusLED0 = 1;
#define mStatusLED0_off()               mStatusLED0 = 0;
#define mStatusLED0_Toggle()            mStatusLED0 = !mStatusLED0;
#define mStatusLED0_Get()               mStatusLED0

//Green led
#define mStatusLED1_on()                mStatusLED1 = 1;
#define mStatusLED1_off()               mStatusLED1 = 0;
#define mStatusLED1_Toggle()            mStatusLED1 = !mStatusLED1;
#define mStatusLED1_Get()               mStatusLED1

#endif	/* HARDWAREPROFILE_H */

