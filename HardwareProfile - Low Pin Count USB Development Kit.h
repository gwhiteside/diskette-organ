#ifndef HARDWARE_PROFILE_LOW_PIN_COUNT_USB_DEVELOPMENT_KIT_H
#define HARDWARE_PROFILE_LOW_PIN_COUNT_USB_DEVELOPMENT_KIT_H

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //#define USE_SELF_POWER_SENSE_IO	
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTAbits.RA2
    #else
    #define self_power          1
    #endif

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISAbits.TRISA1    // Input
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTAbits.RA1
    #else
    #define USB_BUS_SENSE       1
    #endif

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    //Uncomment the following line to make the output HEX of this 
    //  project work with the HID Bootloader
    #define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER	

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    
    #define DEMO_BOARD LOW_PIN_COUNT_USB_DEVELOPMENT_KIT
    #define LOW_PIN_COUNT_USB_DEVELOPMENT_KIT
    #define CLOCK_FREQ 48000000
    
    /** pins ***********************************************************/   

    #define DRIVE_0_SELECT      LATCbits.LATC5
    #define DRIVE_0_DIRECTION   LATCbits.LATC4
    #define DRIVE_0_STEP        LATCbits.LATC3

    #define DRIVE_1_SELECT      LATBbits.LATB7
    #define DRIVE_1_DIRECTION   LATCbits.LATC7
    #define DRIVE_1_STEP        LATCbits.LATC6
    
    #define DRIVE_2_SELECT      LATBbits.LATB4
    #define DRIVE_2_DIRECTION   LATBbits.LATB5
    #define DRIVE_2_STEP        LATBbits.LATB6

    #define DRIVE_3_SELECT      LATCbits.LATC0
    #define DRIVE_3_DIRECTION   LATCbits.LATC1
    #define DRIVE_3_STEP        LATCbits.LATC2

    
    /** SWITCH *********************************************************/
    #define sw2                 PORTAbits.RA3
    #define sw3                 PORTAbits.RA3


    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0

#endif  //HARDWARE_PROFILE_LOW_PIN_COUNT_USB_DEVELOPMENT_KIT_H
