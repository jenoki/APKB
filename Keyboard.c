/********************************************************************
 FileName:		Keyboard.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18 or PIC24 USB Microcontrollers
 Hardware:		The code is natively intended to be used on the following
 				hardware platforms: PICDEM?EFS USB Demo Board, 
 				PIC18F87J50 FS USB Plug-In Module, or
 				Explorer 16 + PIC24 USB PIM.  The firmware may be
 				modified for use on other USB platforms by editing the
 				HardwareProfile.h file.
 Complier:  	Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the Company for its PIC? Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN *AS IS* CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date		 Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
					 coding style
********************************************************************/

#ifndef KEYBOARD_C
#define KEYBOARD_C

/** INCLUDES *******************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb.h"

#include "HardwareProfile.h"

#include "usb_function_hid.h"

//User added
#include "Keyboard.h"

/** CONFIGURATION **************************************************/
//CONFIG1L
	#pragma config PLLDIV	= 4		// (16 MHz crystal on USB board)
	#pragma config CPUDIV	= OSC1_PLL2   
	#pragma config USBDIV	= 2		// Clock source from 96MHz PLL/2
//CONFIG1H
	#pragma config FOSC	= HSPLL_HS
	#pragma config FCMEN	= OFF
	#pragma config IESO	= OFF
//CONFIG2L
	#pragma config PWRT	= ON
	#pragma config BOR		= OFF
	#pragma config BORV	= 3
	#pragma config VREGEN	= ON		//USB Voltage Regulator
//CONFIG2H
	#pragma config WDT	= OFF
	#pragma config WDTPS	= 256		// approx 1(sec)
//CONFIG3H
	#pragma config MCLRE	= OFF
	#pragma config LPT1OSC	= OFF
	#pragma config PBADEN	= OFF
	#pragma config CCP2MX	= ON

//CONFIG4L
	#pragma config STVREN	= ON
	#pragma config LVP	= OFF
#if 0 
	#pragma config ICPRT	= OFF	// Dedicated In-Circuit Debug/Programming
#endif
	#pragma config XINST	= OFF	// Extended Instruction Set
	#pragma config DEBUG	= ON
//CONFIG5L
	#pragma config CP0	  = OFF
	#pragma config CP1	  = OFF
	#pragma config CP2	  = OFF
	#pragma config CP3	  = OFF
//CONFIG5H
	#pragma config CPB	  = OFF
	#pragma config CPD	  = OFF
//CONFIG6L
	#pragma config WRT0	 = OFF
	#pragma config WRT1	 = OFF
	#pragma config WRT2	 = OFF
	#pragma config WRT3	 = OFF
//CONFIG6H
	#pragma config WRTB	 = OFF	   // Boot Block Write Protection
	#pragma config WRTC	 = OFF
	#pragma config WRTD	 = OFF      // EEPROM Write Protection
//CONFIG7L
	#pragma config EBTR0	= OFF
	#pragma config EBTR1	= OFF
	#pragma config EBTR2	= OFF
	#pragma config EBTR3	= OFF
//CONFIG7H
	#pragma config EBTRB	= OFF
#pragma romdata
// Key Definitions
#if defined(APKB)
#warning used Aperture keymaps.
char keytable[]={
    0x00,   0x00,HID_1,0x00,
    0x00,   0x00,HID_2,0x00,
    0x00,   0x00,HID_9,0x00,
    0x00,   0x00,HID_0,0x00,
    0x00,   0x00,HID_3,0x00,
    0x00,   0x00,HIDUP,0x00,
    0x00,   0x00,HIDTAB,0x00,
    0x00,   0x00,HID_SL,0x00,
    0x00,   0x00,HIDLEFT,0x00,
    0x00,   0x00,HIDDOWN,0x00,
    0x00,   0x00,HIDRIGHT,0x00,
    HIDALTM,0x00,HID_1,0x00,
    0x00,   0x00,HID_w,0x00,
    0x00,   0x00,HID_v,0x00,
    0x00,   0x00,HID_RB,0x00,
    HIDSFTM,0x00,HID_u,0x00,
//Key definiton in Mouse Mode
    0x00,   0x00,HID_1,0x00,
    0x00,   0x00,HID_2,0x00,
    0x00,   0x00,HID_9,0x00,
    0x00,   0x00,HID_0,0x00,
    0x00,   0x00,HID_3,0x00,
    0x00,   0x00,HIDUP,0x00,
    0x00,   0x00,HID_p,0x00,
    HIDALTM,0x00,HID_4,0x00,
    0x00,   0x00,HIDLEFT,0x00,
    0x00,   0x00,HIDDOWN,0x00,
    0x00,   0x00,HIDRIGHT,0x00,
    HIDALTM,0x00,HID_1,0x00,
    0x00,   0x00,HID_w,0x00,
    0x00,   0x00,HID_v,0x00,
    0x00,   0x00,HIDLEFT,0x00,
    0x00,   0x00,HIDRIGHT,0x00
};
#elif defined(LRKB)
#warning used Lightroom keymaps
const char keytable[]={
    0x00,   0x00,HID_1,0x00,
    0x00,   0x00,HID_2,0x00,
    0x00,   0x00,HID_x,0x00,
    0x00,   0x00,HID_u,0x00,
    0x00,   0x00,HID_3,0x00,
    0x00,   0x00,HIDUP,0x00,
    0x00,   0x00,0x00,0x00,
    0x00,   0x00,HID_a,0x00,
    0x00,   0x00,HIDLEFT,0x00,
    0x00,   0x00,HIDDOWN,0x00,
    0x00,   0x00,HIDRIGHT,0x00,
    HIDALTM,0x00,HID_1,0x00,
    0x00,   0x00,0x00,0x00,
    0x00,   0x00,0x00,0x00,
    HIDGUIM,0x00,HID_RB,0x00,
    HIDSFTM,0x00,HID_u,0x00,
//Key definiton in Mouse Mode
    0x00,   0x00,HID_1,0x00,
    0x00,   0x00,HID_2,0x00,
    0x00,   0x00,HID_x,0x00,
    0x00,   0x00,HID_u,0x00,
    (HIDGUIM+HIDSFTM),   0x00,HID_DT ,0x00,
    0x00,   0x00,HIDUP,0x00,
    (HIDGUIM+HIDSFTM),   0x00,HID_CM ,0x00,
    HIDALTM,0x00,HID_4,0x00,
    0x00,   0x00,HIDLEFT,0x00,
    0x00,   0x00,HIDDOWN,0x00,
    0x00,   0x00,HIDRIGHT,0x00,
    HIDALTM,0x00,HID_1,0x00,
    0x00,   0x00,0x00,0x00,
    0x00,   0x00,0x00,0x00,
    0x00,   0x00,HIDLEFT,0x00,
    0x00,   0x00,HIDRIGHT,0x00
};
#endif
/** FILE SCOPIC VARIABLES *******************************************/
#pragma udata

// User added 
WORD HIDButtons;
WORD HIDnewButtons;
WORD HIDprevButtons;
WORD HIDoldButtons;

BOOL HIDSideSw;
BOOL HIDnewSideSw;
BOOL HIDprevSideSw;
BOOL HIDoldSideSw;
BOOL HitSideSw;
BOOL BrkSideSw;
BOOL SideSwStat;

BOOL KeyMouse = SIDE_SW_KEYBOARD;
USB_HANDLE lastTransmissionM;
USB_HANDLE lastTransmissionK;
BOOL Hit;
WORD MouseBitMask;
BYTE Timer0Count = 0;
BYTE Timer0CountLimit;

char modifiers[128]; //EEP_ELEM_COUNT * EEP_ELEM_COUNT * 2

/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void ClearReport(void);

// User added
int Keyboard(void);
int Mouse(void);
WORD read_buttons(void);
BOOL read_side_sw(void);
void KeyConfig(void);
int SideSw(void);
void Sendmodifiers(BYTE);
void SendModifierElement(BYTE);
BYTE bitcount8(BYTE);
BYTE bitcount16(WORD);
BYTE ntz(WORD);

//Approx. 20ms periodic timer
#define TMR0VALH	(0xFB)	//(HIGH)0xFFFF-0x0460
#define TMR0VALL	(0x9F)	//(LOW )0xFFFF-0x0460

#define	EEP_ELEM_SIZE	4
#define	EEP_ELEM_COUNT	16


/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
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

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
extern void _startup (void);		// See c018i.c in your C18 compiler dir
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
    if (INTCONbits.TMR0IF){
        INTCONbits.TMR0IE = 0;	//Interrupt Disable
        TMR0H = TMR0VALH;	//Reload Timer Value
        TMR0L = TMR0VALL;
        INTCONbits.TMR0IF = 0;	//Clear Interrupt Flag
        // Do your own Task Invoked by Timer0

        Timer0Count++;
        if(Timer0Count > Timer0CountLimit) {
            Timer0Count = 0;
            
            if(KeyMouse==SIDE_SW_MOUSE) //at MOUSE mode
                PORTBbits.RB7 = (~PORTBbits.RB7);   //flash LED.
            
        }

        HIDnewButtons = read_buttons();
        if(HIDnewButtons == HIDprevButtons){
            WORD dif = HIDnewButtons ^ HIDoldButtons;
            if (dif != 0) {
                HIDoldButtons = HIDnewButtons;
                HIDButtons = HIDnewButtons;
                Hit = TRUE;
            }else{
                if(MouseBitMask & HIDnewButtons){
                    Hit=TRUE;
                }
            }
        }
        HIDprevButtons = HIDnewButtons;

        HIDnewSideSw = read_side_sw();
        if(HIDnewSideSw == HIDprevSideSw){
            BOOL dif = HIDnewSideSw ^ HIDoldSideSw;
            if(dif != FALSE) {
                HIDoldSideSw = HIDnewSideSw;
                HIDSideSw = HIDnewSideSw;
                HitSideSw = TRUE;
            }
        }

        if(SideSwStat==TRUE){
            BrkSideSw = TRUE;
            SideSwStat = FALSE;
        }

        HIDprevSideSw = HIDnewSideSw;

        INTCONbits.TMR0IE = 1;	//Interrupt Enable
    }
}	//This return will be a "retfie fast", since this is in a #pragma interrupt section
#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode()
{
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.

}	//This return will be a "retfie", since this is in a #pragma interruptlow section
#endif

/** DECLARATIONS ***************************************************/
#pragma code

/********************************************************************
 * Function:		void main(void)
 * PreCondition:	None
 * Input:			None
 * Output:			None
 * Side Effects:	None
 * Overview:		Main program entry point.
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
        ClrWdt();
        #if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
                          // this function periodically.  This function will take care
                          // of processing and responding to SETUP transactions
                          // (such as during the enumeration process when you first
                          // plug in).  USB hosts require that USB devices should accept
                          // and process SETUP packets in a timely fashion.  Therefore,
                          // when using polling, this function should be called
                          // frequently (such as once about every 100 microseconds) at any
                          // time that a SETUP packet might reasonably be expected to
                          // be sent by the host to your device.  In most cases, the
                          // USBDeviceTasks() function does not take very long to
                          // execute (~50 instruction cycles) before it returns.
        #endif

        // Application-specific tasks.
        // Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();
    }//end while
}//end main

/********************************************************************
 * Function:		static void InitializeSystem(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		InitializeSystem is a centralize initialization
 *				routine. All required USB initialization routines
 *				are called from here.
 *
 *				User application initialization routine should
 *				also be called from here.				  
 *******************************************************************/
static void InitializeSystem(void)
{
    #if (defined(__18CXX))
        ADCON1 |= 0x0F;				 // Default all pins to digital
    #endif

    UserInit();
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
			//variables to known states.
}//end InitializeSystem

/******************************************************************************
 * Function:		void UserInit(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		This routine should take care of all of the demo code
 *					initialization that is required.
 *****************************************************************************/
void UserInit(void)
{
    //Set 18F2550 I/O Ports
    TRISA = 0xff;	//PORTA all input
    TRISC = 0xff;	//PORTC all input

    TRISB = 0x7f;	//PORTB7 output / 6-0 input
    PORTBbits.RB7 = 0;

    //Initialize Timer0
    //RCONbits.IPEN=0;      //Default Value
    T0CONbits.T0CS = 0;     // use Fosc/4 for Timer0
    T0CONbits.T0SE = 0;
    T0CONbits.T08BIT = 0;   // 16bit timer
    T0CONbits.T0PS = 7;     // 1/256 pre-scaler
    T0CONbits.PSA = 0;      // use pre-scaler

    TMR0H = TMR0VALH;       //load Timer Value
    TMR0L = TMR0VALL;

    T0CONbits.TMR0ON = 1;   // Timer 0 On
    INTCONbits.TMR0IE = 1;  //Timer0 Interrupt Enable

    //initialize the variable holding the handle for the last
    // transmission
    lastTransmissionK = 0;
    lastTransmissionM = 0;

    MouseBitMask = MOUSE_BIT_MASK;

    //Initialize Variables
    HIDoldButtons= HIDButtons= HIDnewButtons= HIDprevButtons= read_buttons();
    HIDSideSw= HIDnewSideSw= HIDprevSideSw= HIDoldSideSw= read_side_sw();
    Hit = FALSE;
    HitSideSw = FALSE;
    SideSwStat = FALSE;
    BrkSideSw = FALSE;

    Timer0CountLimit = 20;
    KeyConfig();    //
}//end UserInit

/********************************************************************
 * Function:		void ProcessIO(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		This function is a place holder for other user
 *				  routines. It is a mixture of both USB and
 *				  non-USB tasks.
 *******************************************************************/
void ProcessIO(void)
{
    // User Application USB tasks
    // If not configured do nothing
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl == 1)) return;

    if (Hit==TRUE){
        //Clear Buffer Contents;
        ClearReport();

        if(Keyboard() == 0)
            Mouse();
        Hit = FALSE;
        return;
    }

    if(HitSideSw==TRUE){
        //Clear Buffer Contents;
        ClearReport();
        if(HIDSideSw==SIDE_SW_ON){
#if defined(APKB)
                KeyMouse=(KeyMouse==SIDE_SW_KEYBOARD)? SIDE_SW_MOUSE:SIDE_SW_KEYBOARD;
                hid_report_in[2] = HID_z;
                SideSw();
#elif defined(LRKB)
                KeyMouse=(KeyMouse==SIDE_SW_KEYBOARD)? SIDE_SW_MOUSE:SIDE_SW_KEYBOARD;
#endif
                PORTBbits.RB7 = 0;	//LED On
        }
        HitSideSw = FALSE;
        SideSwStat = TRUE;
        return;
    }

    if(BrkSideSw==TRUE){
        ClearReport();
        SideSw();
        BrkSideSw = FALSE;
        return;
    }

}//end ProcessIO

void ClearReport(void){
    hid_report_in[0] = hid_report_in[1] = hid_report_in[2] \
    = hid_report_in[3] = hid_report_in[4] = 0;
}

int SideSw(void){
    int ret = 0;
    if(!HIDTxHandleBusy(lastTransmissionK))
    {
        lastTransmissionK = HIDTxPacket(HID_EPK, (BYTE*)hid_report_in, KEYBOARD_LEN);
        ret = KEYBOARD_LEN;
    }
    return ret;
}

void SendModifierElement(BYTE mod){
    hid_report_in[0] = mod;
    hid_report_in[1] = hid_report_in[2] = hid_report_in[3] = 0;
    
    if(!HIDTxHandleBusy(lastTransmissionK))
    {
        lastTransmissionK = HIDTxPacket(HID_EPK, (BYTE*)hid_report_in, KEYBOARD_LEN);
    }
    while(HIDTxHandleBusy(lastTransmissionK)) ;//wait until free to next transmit
}

void SendModifiers(BYTE eeptop){
    int i;
    for (i = 0;i < EEP_ELEM_SIZE;i++){
        BYTE mod = modifiers[eeptop+i];
        if(mod == 0x00) break;
        SendModifierElement(mod);
    }
}

int Keyboard(void)
{
    int ret = 0;
    BYTE bitpos;
    WORD mask = 1;
    if(!HIDTxHandleBusy(lastTransmissionK))
    {
        if ( (KeyMouse != SIDE_SW_KEYBOARD) && ((HIDButtons & MouseBitMask) != 0))
            return 0;	//examine Keyboard task.

#if 0
        for(bitpos = 0 ; bitpos < 16 ; bitpos++){
            if((HIDButtons & mask) != 0) break;
            mask = mask << 1;
        }
#else
        bitpos = ntz(HIDButtons);
#endif
        //Sets up buffer
        if(bitpos != 16)	//if any bits are found
        {
            BYTE eeptop = (EEP_ELEM_SIZE * bitpos);
            if(KeyMouse==SIDE_SW_MOUSE) eeptop += (EEP_ELEM_SIZE * EEP_ELEM_COUNT);

            if (modifiers[eeptop] != 0){
                SendModifiers(eeptop);
            }
            hid_report_in[KEYBOARD_RPT_IDX_M] = keytable[eeptop];
            hid_report_in[1] = keytable[eeptop+1];
            hid_report_in[2] = keytable[eeptop+2];
            hid_report_in[3] = keytable[eeptop+3];
        }

        lastTransmissionK = HIDTxPacket(HID_EPK, (BYTE*)hid_report_in, KEYBOARD_LEN);
        ret = KEYBOARD_LEN;
    }
    return ret;
}//end keyboard()

int Mouse(void){
    int ret = 0;
    if (KeyMouse==SIDE_SW_MOUSE && HIDButtons != 0){
        if( !HIDTxHandleBusy(lastTransmissionM) ){	// Is the IN2BUF available,

            if (HIDButtons & AKB05){	// [up]
                hid_report_in[MOUSE_RPT_IDX_ROTATE] = WHEEL_DOWN;
            }else if (HIDButtons & AKB09){	// [down]
                hid_report_in[MOUSE_RPT_IDX_ROTATE] = WHEEL_UP;
            }else if (HIDButtons & AKB08){	// [left]
                hid_report_in[MOUSE_RPT_IDX_TILT] = WHEEL_RIGHT;
            }else if (HIDButtons & AKB10){	// [right]
                hid_report_in[MOUSE_RPT_IDX_TILT] = WHEEL_LEFT;
            }else if (HIDButtons & AKB12){
                hid_report_in[MOUSE_RPT_IDX_MOVE] = MOUSE_MOVE_LEFT;
            }else if (HIDButtons & AKB13){
                hid_report_in[MOUSE_RPT_IDX_MOVE] = MOUSE_MOVE_RIGHT;
#if 0 //experimental
            }else if (HIDButtons & AKB04){
                hid_report_in[MOUSE_RPT_IDX_BTN] = MOUSE_BUTTON_LEFT;
            }else if (HIDButtons & AKB06){
                hid_report_in[MOUSE_RPT_IDX_BTN] = MOUSE_BUTTON_RIGHT;
#endif                
            }

            lastTransmissionM = HIDTxPacket(HID_EPM, (BYTE*)hid_report_in, MOUSE_LEN);
            ret = MOUSE_LEN;
        }
    }
    return ret;
}//end Mouse()

/***********************************************************************
Function:	WORD read_buttons(void)
parameter:	none
return:		switch condition by bit-pattern;
***********************************************************************/
WORD read_buttons()
{
    WORD pa = (PORTA & 0x3f);
    WORD pb = (PORTB & 0x1f);
    WORD pc = (PORTC & 0xc7);
    return ~( (pb)<<11|(pc&0xc0)<<3|((pc&0x7)<<6)|(pa) );
}

/***********************************************************************
Function:	BOOL read_side_sw(void)
parameter:	none
return:		side-switch condition;
************************************************************************/
BOOL read_side_sw()
{
    return (PORTBbits.RB5)? FALSE:TRUE;
}
BYTE bitcount8(BYTE b8){
	BYTE count = ( ((b8 & 0xAA) >> 1) + (b8 & 0x55) );
	count = ( ((count & 0xCC) >> 2) + (count & 0x33) );
	return ( ((count & 0xF0) >> 4) + (count & 0x0F) );

}//bitcount8

BYTE bitcount16(WORD b16) {
    WORD count = (b16 & 0x5555) + ((b16 >> 1) & 0x5555);
    count = (count & 0x3333) + ((count >> 2) & 0x3333);
    count = (count & 0x0f0f) + ((count >> 4) & 0x0f0f);
    return (count & 0x00ff) + ((count >> 8) & 0x00ff);
}//bitcount 16

BYTE ntz(WORD v){
    WORD w=(v&(-v))-1;
    return bitcount16(w);
}

void KeyConfig(){//expand modifier key table
    int mod_index;
    memset(modifiers,0,sizeof(modifiers));
    for (mod_index = 0 ; mod_index < sizeof(modifiers) ; mod_index += EEP_ELEM_SIZE){
        BYTE modifier = keytable[mod_index];
        BYTE bitcount = bitcount8(modifier);
        if (modifier != 0x00 && bitcount > 1){
            BYTE dst_index = mod_index;
            BYTE pattern = 0;
            if(modifier & HIDGUIM) {
                pattern += HIDGUIM;
                modifiers[dst_index] = pattern;
                dst_index += 1;
            }
            if(modifier & HIDALTM) {
                pattern += HIDALTM;
                modifiers[dst_index] = pattern;
                dst_index +=1;
            }
            if(modifier & HIDSFTM) {
                pattern += HIDSFTM;
                modifiers[dst_index] = pattern;
                dst_index += 1;
            }
            if(modifier & HIDCTLM) {
                pattern += HIDCTLM;
                modifiers[dst_index] = pattern;
            }
        }
    }
}
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
 * Function:		void USBCBSuspend(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		Call back that is invoked when a USB suspend is detected
zz *****************************************************************************/
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
    //RestoreIOPinsToNormal();					//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

    //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
    //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
    //things to not work as intended.

}


/******************************************************************************
 * Function:		void _USB1Interrupt(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
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
 * Function:		void USBCBWakeFromSuspend(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
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
 * Function:		void USBCB_SOF_Handler(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:	The USB host sends out a SOF packet to full-speed
 *				devices every 1 ms. This interrupt may be useful
 *				for isochronous pipes. End designers should
 *				implement callback routine as necessary.
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:		void USBCBErrorHandler(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		The purpose of this callback is mainly for
 *					debugging during development. Check UEIR to see
 *					which error causes the interrupt.
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
 * Function:		void USBCBCheckOtherReq(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		When SETUP packets arrive from the host, some
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
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}//end


/*******************************************************************
 * Function:		void USBCBStdSetDscHandler(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
	// Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:		void USBCBInitEP(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		This function is called when the device becomes
 *					initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoints
    USBEnableEndpoint(HID_EPK,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(HID_EPM,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
}

/********************************************************************
 * Function:		void USBCBSendResume(void)
 * PreCondition:	None
 * Input:			None
 * Output:		None
 * Side Effects:	None
 * Overview:		The USB specifications allow some types of USB
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
 *					function should only be called when:
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
 *					This callback should send a RESUME signal that
 *				  has the period of 1-15ms.
 *
 * Note:			Interrupt vs. Polling
 *				  -Primary clock
 *				  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *				   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *				  The modifiable section in this routine should be changed
 *				  to meet the application needs. Current implementation
 *				  temporary blocks other functions from executing for a
 *				  period of 1-13 ms depending on the core frequency.
 *
 *				  According to USB 2.0 specification section 7.1.7.7,
 *				  "The remote wakeup device must hold the resume signaling
 *				  for at lest 1 ms but for no more than 15 ms."
 *				  The idea here is to use a delay counter loop, using a
 *				  common value that would work over a wide range of core
 *				  frequencies.
 *				  That value selected is 1800. See table below:
 *				  ==========================================================
 *				  Core Freq(MHz)	  MIP		 RESUME Signal Period (ms)
 *				  ==========================================================
 *					  48			  12				1.05
 *					   4			  1				12.6
 *				  ==========================================================
 *				  * These timing could be incorrect when using code
 *					optimization or extended instruction mode,
 *					or when having other interrupts enabled.
 *					Make sure to verify using the MPLAB SIM's Stopwatch
 *					and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;

    USBResumeControl = 1;				// Start RESUME signaling

    delay_count = 1800U;				// Set RESUME line for 1-13 ms
    do {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:		BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *						USB_EVENT event, void *pdata, WORD size)
 * PreCondition:	None
 * Input:		   USB_EVENT event - the type of event
 *				  void *pdata - pointer to the event data
 *				  WORD size - size of the event data
 *
 * Output:		  None
 * Side Effects:	None
 * Overview:		This function is called from the USB stack to
 *				  notify a user application that a USB event
 *				  occured.  This callback is in interrupt context
 *				  when the USB_INTERRUPT option is selected.
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED:
                USBCBInitEP();
                break;
        case EVENT_SET_DESCRIPTOR:
                USBCBStdSetDscHandler();
                break;
        case EVENT_EP0_REQUEST:
                USBCBCheckOtherReq();
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
        case EVENT_BUS_ERROR:
                USBCBErrorHandler();
                break;
        case EVENT_TRANSFER:
                Nop();
                break;
        default:
                break;
    }
    return TRUE;
}

/** EOF Keyboard.c **********************************************/
#endif
