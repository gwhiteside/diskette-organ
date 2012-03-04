#ifndef MAIN_C
#define MAIN_C

/** INCLUDES *******************************************************/

#include "./USB/usb.h"
#include "HardwareProfile.h"
#include "./USB/usb_function_midi.h"
#include <delays.h>
#include <math.h>

/** CONFIGURATION **************************************************/

#pragma config CPUDIV = NOCLKDIV
#pragma config USBDIV = OFF
#pragma config FOSC   = HS
#pragma config PLLEN  = ON
#pragma config FCMEN  = OFF
#pragma config IESO   = OFF
#pragma config PWRTEN = OFF
#pragma config BOREN  = OFF
#pragma config BORV   = 30
#pragma config WDTEN  = OFF
#pragma config WDTPS  = 32768
#pragma config MCLRE  = OFF
#pragma config HFOFST = OFF
#pragma config STVREN = ON
#pragma config LVP    = OFF
#pragma config XINST  = OFF
#pragma config BBSIZ  = OFF
#pragma config CP0    = OFF
#pragma config CP1    = OFF
#pragma config CPB    = OFF
#pragma config WRT0   = OFF
#pragma config WRT1   = OFF
#pragma config WRTB   = OFF
#pragma config WRTC   = OFF
#pragma config EBTR0  = OFF
#pragma config EBTR1  = OFF
#pragma config EBTRB  = OFF                                                  // CONFIG7H

/** VARIABLES ******************************************************/
#pragma udata

#if defined(__18F14K50) || defined(__18F13K50) || defined(__18LF14K50) || defined(__18LF13K50)
    #pragma udata usbram2
#elif defined(__18F2455) || defined(__18F2550) || defined(__18F4455) || defined(__18F4550)\
    || defined(__18F2458) || defined(__18F2453) || defined(__18F4558) || defined(__18F4553)
    #pragma udata USB_VARIABLES=0x500
#elif defined(__18F4450) || defined(__18F2450)
    #pragma udata USB_VARIABLES=0x480
#else
    #pragma udata
#endif

unsigned char ReceivedDataBuffer[64];
//unsigned char ToSendDataBuffer[64];
USB_AUDIO_MIDI_EVENT_PACKET midiData;

#pragma udata

USB_HANDLE USBTxHandle = 0;
USB_HANDLE USBRxHandle = 0;

struct {
	unsigned char MidiNote;
	unsigned int Timer;
	unsigned int Period;
	unsigned long StealTimer;
	BOOL Active;
} Generator[4];

const unsigned char CHANNELS = 4;
const unsigned char PITCH_BEND_RANGE = 2;

#pragma idata freqdata

// This is a table of all 128 MIDI note frequencies. It has been precomputed
// as the period, T, in tens of microseconds. It would have been simpler to
// calculate this at startup, but ROM is far more plentiful than RAM.

// Carrying the full 128 notes is completely unnecessary (you literally can't
// hear half of them played on a floppy drive), but it's not exactly hurting
// anything at this point.

rom unsigned int NotePeriod[128] = { 12231, 11545, 10897, 10285, 9708, 9163, 8649, 8163, 7705, 7273, 6865, 6479, 6116, 5772, 5448, 5143, 4854, 4582, 4324, 4082, 3853, 3636, 3432, 3240, 3058, 2886, 2724, 2571, 2427, 2291, 2162, 2041, 1926, 1818, 1716, 1620, 1529, 1443, 1362, 1286, 1213, 1145, 1081, 1020, 963, 909, 858, 810, 764, 722, 681, 643, 607, 573, 541, 510, 482, 455, 429, 405, 382, 361, 341, 321, 303, 286, 270, 255, 241, 227, 215, 202, 191, 180, 170, 161, 152, 143, 135, 128, 120, 114, 107, 101, 96, 90, 85, 80, 76, 72, 68, 64, 60, 57, 54, 51, 48, 45, 43, 40, 38, 36, 34, 32, 30, 28, 27, 25, 24, 23, 21, 20, 19, 18, 17, 16, 15, 14, 13, 13, 12, 11, 11, 10, 9, 9, 8, 8 };
rom float BendCoeff[32] = {0.99819656, 0.99639637, 0.99459942, 0.99280572, 0.99101525, 0.98922801, 0.98744400, 0.98566320, 0.98388561, 0.98211123, 0.98034005, 0.97857206, 0.97680726, 0.97504565, 0.97328721, 0.97153194, 0.96977984, 0.96803090, 0.96628511, 0.96454247, 0.96280297, 0.96106661, 0.95933338, 0.95760328, 0.95587630, 0.95415243, 0.95243167, 0.95071402, 0.94899946, 0.94728799, 0.94557961, 0.94387431};

/** PRIVATE PROTOTYPES *********************************************/
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
WORD_VAL ReadPOT(void);

/** VECTOR REMAPPING ***********************************************/

//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
//the reset, high priority interrupt, and low priority interrupt
//vectors.  However, the current Microchip USB bootloader
//examples are intended to occupy addresses 0x00-0x7FF or
//0x00-0xFFF depending on which bootloader is used.  Therefore,
//the bootloader code remaps these vectors to new locations
//as indicated below.  This remapping is only necessary if you
//wish to program the hex file generated from this project with
//the USB bootloader.  If no bootloader is used, edit the
//usb_config.h file and comment out the following defines:
//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
	#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
	#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
	#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
	#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
	#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
#else
	#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
	#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
	#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
#endif

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
extern void _startup (void);        // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset (void)
{
    _asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR (void)
{
     _asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR (void)
{
     _asm goto YourLowPriorityISRCode _endasm
}

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
//Note: If this project is built while one of the bootloaders has
//been defined, but then the output hex file is not programmed with
//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
//As a result, if an actual interrupt was enabled and occured, the PC would jump
//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
//would effective reset the application.

//To fix this situation, we should always deliberately place a
//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
//hex file of this project is programmed with the bootloader, these sections do not
//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
//programmed using the bootloader, then the below goto instructions do get programmed,
//and the hex file still works like normal.  The below section is only required to fix this
//scenario.
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR (void)
{
     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR (void)
{
     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

#pragma code





#define MIDI_MESSAGE_MASK	0xF0
#define MIDI_CHANNEL_MASK	0x0F

#define MIDI_NOTE_OFF		0x80
#define MIDI_NOTE_ON		0x90
#define MIDI_PITCH_BEND		0xE0


#define DELAY_1_US	Delay10TCYx(1); Delay1TCY(); Delay1TCY()

//These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode
void YourHighPriorityISRCode()
{
	//Check which interrupt flag caused the interrupt.
	//Service the interrupt
	//Clear the interrupt flag
	//Etc.
       #if defined(USB_INTERRUPT)
        USBDeviceTasks();
       #endif

}	//This return will be a "retfie fast", since this is in a #pragma interrupt section

#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode()
{
	// Service Timer2 interrupt (10us resolution note generator)

	if( PIR1bits.TMR2IF == 1 )
	{
		PIR1bits.TMR2IF = 0;	// Clear the interrupt flag

		Generator[0].Timer++;
		Generator[1].Timer++;
		Generator[2].Timer++;
		Generator[3].Timer++;



		// NOTE! In the following sections, I have Delay1TCY(); statements littered
		// about in an ugly fashion. I was conducting some timing experiments on my
		// floppy hardware... some drives are more tolerant than others when it comes
		// to timing. I'll put some easily adjustable knobs in later, but feel
		// free to mess around with it in the meantime.



		if( Generator[0].Active == TRUE )
		{
			if( Generator[0].Timer >= Generator[0].Period )
			{
				Generator[0].Timer = 0;

				DRIVE_0_SELECT = 0; // light on

				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				DRIVE_0_STEP = 0;
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				DRIVE_0_STEP = 1;
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				DRIVE_0_DIRECTION = ~DRIVE_0_DIRECTION;
			}
		}
		else
		{
			DRIVE_0_SELECT = 1;	// light off
		}










		if( Generator[1].Active == TRUE )
		{
			if( Generator[1].Timer >= Generator[1].Period )
			{
				Generator[1].Timer = 0;

				DRIVE_1_SELECT = 0; // light on

				Delay1TCY(); Delay1TCY();
				DRIVE_1_STEP = 0;
				Delay1TCY(); Delay1TCY(); 
				Delay1TCY(); Delay1TCY();
				DRIVE_1_STEP = 1;
				Delay1TCY();
				DRIVE_1_DIRECTION = ~DRIVE_1_DIRECTION;
			}
		}
		else
		{
			DRIVE_1_SELECT = 1;	// light off
		}




		if( Generator[2].Active == TRUE )
		{
			if( Generator[2].Timer >= Generator[2].Period )
			{
				static unsigned char count = 0;
				
				Generator[2].Timer = 0;

				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY(); // test block
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();

				DRIVE_2_SELECT = 0; // light on

				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY(); // test block
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();


				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				DRIVE_2_STEP = 0;
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();



				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY(); // test block
				Delay1TCY(); Delay1TCY();



				DRIVE_2_STEP = 1;
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();

				//count++;
				//if( count >= 80 )
				{
					DRIVE_2_DIRECTION = ~DRIVE_2_DIRECTION;
					//count = 0;
				}
			}
		}
		else
		{
			DRIVE_2_SELECT = 1;	// light off
		}





		if( Generator[3].Active == TRUE )
		{
			if( Generator[3].Timer >= Generator[3].Period )
			{
				Generator[3].Timer = 0;

				DRIVE_3_SELECT = 0; // light on

				Delay1TCY(); Delay1TCY();
				DRIVE_3_STEP = 0;
				Delay1TCY(); Delay1TCY();
				Delay1TCY(); Delay1TCY();
				DRIVE_3_STEP = 1;
				Delay1TCY();
				DRIVE_3_DIRECTION = ~DRIVE_3_DIRECTION;
			}
		}
		else
		{
			DRIVE_3_SELECT = 1;	// light off
		}







		
		
	}

}	//This return will be a "retfie", since this is in a #pragma interruptlow section



/** DECLARATIONS ***************************************************/
#pragma code



/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    while(1)
    {
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // regularly (such as once every 1.8ms or faster** [see 
        				  // inline code comments in usb_device.c for explanation when
        				  // "or faster" applies])  In most cases, the USBDeviceTasks() 
        				  // function does not take very long to execute (ex: <100 
        				  // instruction cycles) before it returns.
        #endif


		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    ADCON1 |= 0x0F;                 // Default all pins to digital

	// Configure global interrupts

	RCONbits.IPEN = 1;				// Enable interrupt priority discrimination
	INTCONbits.GIEH = 1;			// Globally enable high-priority interrupts
	INTCONbits.GIEL = 1;			// Globally enable low-priority interrupts
	IPR2bits.USBIP = 1;				// USB interrupt events are high priority

	// Configure and enable Timer2 @ 100 kHz... see PIC18F14k50 docs for details

	IPR1bits.TMR2IP = 0;	// Select low-priority interrupts
	T2CON = 0;				// 1:1 postscale, prescale = 1, timer disabled
	PIE1bits.TMR2IE = 1;	// Generate interrupts
	PR2 = 120 - 1;			// Timer2 period = 120; FOSC/4 * 120 = 10us
	PIR1bits.TMR2IF = 0;	// Clear interrupt flag

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif

//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif

    UserInit();

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.

	T2CONbits.TMR2ON = 1;	// Timer2 enable

}//end InitializeSystem



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:
 *
 *****************************************************************************/
void UserInit(void)
{
	int i;

	// Configure output pins

	TRISC &= 0x00;	// RC0:RC7 output
	TRISB &= 0x0F;	// RB4:RB7 output

	// Initial state of floppy drives (active low)

	LATC = 0xFF;
	LATB = 0xFF;

	i = 4;
	while(i--)
	{
		Generator[i].Timer = 0;
		Generator[i].StealTimer = 0;
		Generator[i].Active = FALSE;
	}



    //initialize the variable holding the handle for the last
    // transmission
    USBTxHandle = NULL;
    USBRxHandle = NULL;
}//end UserInit

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{

    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

	

    if(!USBHandleBusy(USBRxHandle))
    {
        //We have received a MIDI packet from the host, process it and then
        //  prepare to receive the next packet

        //INSERT MIDI PROCESSING CODE HERE

		BOOL note_stealing = FALSE;
		static BYTE note;
		int channel;
		BYTE oldest_channel;
		unsigned long lowest_counter;
		BOOL duplicate_note = FALSE;
		float bent_note;
		unsigned char bend_value;

		USB_AUDIO_MIDI_EVENT_PACKET *packet = ReceivedDataBuffer; // suspicious pointer conversion, yada yada
	
		// run through the complete buffer (16 packets x 4 bytes) and process MIDI messages
		int buffer = 16; while(buffer--)
		{
			switch(packet->DATA_0 & MIDI_MESSAGE_MASK)
			{
				case MIDI_NOTE_ON:

					if(note_stealing)
					{
						// Note stealing logic: find the channel with the lowest steal counter,
						// and reassign it to the newest note on command. In the event of a
						// tie, channel order decides.
	
						oldest_channel = 0;
						lowest_counter = 0xFFFFFFFF;
	
						note = packet->DATA_1;
						
						channel = CHANNELS;
						while(channel--)
						{
							if(Generator[channel].Active == TRUE)
								Generator[channel].StealTimer--;
	
							if(Generator[channel].MidiNote == note && Generator[channel].Active == TRUE)
								duplicate_note = TRUE;
	
							if(Generator[channel].StealTimer < lowest_counter)
							{
								lowest_counter = Generator[channel].StealTimer;
								oldest_channel = channel;
							}
						}
	
						if(!duplicate_note)
						{
							Generator[oldest_channel].MidiNote = note;
							Generator[oldest_channel].Period = NotePeriod[note];
							Generator[oldest_channel].StealTimer = 0xFFFFFFFF;
							Generator[oldest_channel].Active = TRUE;
						}
					}
					else
					{
						note = packet->DATA_1;
						channel = packet->DATA_0 & MIDI_CHANNEL_MASK;

						if(channel >= 0 && channel < CHANNELS)
						{
							Generator[channel].MidiNote = note;
							Generator[channel].Period = NotePeriod[note];
							Generator[channel].Active = TRUE;
						}
					}

					break;
				
				case MIDI_NOTE_OFF:

					if(note_stealing)
					{
						channel = CHANNELS;
		
						note = packet->DATA_1;
		
						// find the active channel for this note, if it exists
						while(channel--)
						{
							if(Generator[channel].MidiNote == note)
							{
								//kill it
								Generator[channel].Active = FALSE;
								Generator[channel].StealTimer = 0;
								break;
							}
						}
					}
					else
					{
						note = packet->DATA_1;
						channel = packet->DATA_0 & MIDI_CHANNEL_MASK;

						if(channel >= 0 && channel < CHANNELS)
						{
							if(Generator[channel].MidiNote == note)
							{
								Generator[channel].Active = FALSE;
							}
						}
					}

					break;

				// Please excuse this atrocious bend code. It was rushed to be used, only to not
				// actually be used at all due to a change in song choice. It did work "kind of"
				// when I originally implemented it, but I never ironed the bugs out. In fact it
				// may not work at all in its present state due to changes elsewhere. I definitely
				// did have it working at one point in time for a song demo I decided not to make.

				case MIDI_PITCH_BEND:
					{
						channel = packet->DATA_0 & MIDI_CHANNEL_MASK;
						

						if(channel >= 0 && channel < CHANNELS)
						{
							note = Generator[channel].MidiNote;
							//bent_note = NotePeriod[note];

							// 100 cents per semitone, PITCH_BEND_RANGE max semitones

							// 1. calculate how many cents to bend pitch ...

							// we can use a piecewise linear interpolation between semitones
							// if we restrict the max bend range to +/- 2 semitones, we can
							// just check if the bend is between 0 and 1 semitones and linearly interpolate,
							// or if it's between 1 and 2 semitones, start at the next highest note, and linearly
							// interpolate to the one above that (or so on for successively higher ranges)

							// 2 ^ 1/12 = 1.05946309435929526456...
							// 2 ^-1/12 = 0.94387431268169349664...

							// for PITCH_BEND_RANGE = 2
							// max cents = PITCH_BEND_RANGE * 100 = 200 cents
							// 0x2000 - 0x3FFF = 200 cents; 0x2000 - 0x0000 = 200 cents

							// 0x2000 = no bend, 0x0000 = full bend down, 0x3FFF = full bend up


							// 1. assemble the 14-bit bend value
							//bend_value = ((int)(Generator[channel].DATA_2 & 0x7F) << 7) | (int)(Generator[channel].DATA_1 & 0x7F);

							// actually, we can probably get away with using only the most significant byte because the floppy-stepper-oscillator is such a crude sound in the first place
							bend_value = packet->DATA_2;

							
							
							// 2. determine if the bend is up or down
							if(bend_value > 0x40)
							{
								bend_value -= 0x40; // value between 0x00 and 0x3F

								if(bend_value >= 0x20) // bend past +1 semitone
								{
									bend_value -= 0x20; // value between 0x00 and 0x1F;
									bent_note = NotePeriod[note+1];
								}
								else // bend between 0 and +1 semitone
								{
									bent_note = NotePeriod[note];
								}

								// !! FIX THIS
								// !! we need to bend the cycle period down to bend frequency up, and vice-versa

								//bend_value = 0x20 - bend_value;
								bent_note *= BendCoeff[bend_value];
							}
							else if(bend_value < 0x40)
							{
								bend_value = bend_value - 0x39;
							}

							Generator[channel].Period = bent_note;
						}
					}

					break;

				default:
					break;
			}
	
			packet++;
		}

        //Get ready for next packet (this will overwrite the old data)
        USBRxHandle = USBRxOnePacket(MIDI_EP,(BYTE*)&ReceivedDataBuffer,64);
    }

}//end ProcessIO




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:

	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
	//things to not work as intended.


    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;

            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *
 *					This call back is invoked when a wakeup from USB suspend
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.

//    if(msCounter != 0)
//    {
//        msCounter--;
//    }
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.

	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This
 *					callback function should initialize the endpoints
 *					for the device's usage according to the current
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(MIDI_EP,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBRxHandle = USBRxOnePacket(MIDI_EP,(BYTE*)&ReceivedDataBuffer,64);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
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
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }
    return TRUE;
}

/** EOF main.c *************************************************/
#endif
