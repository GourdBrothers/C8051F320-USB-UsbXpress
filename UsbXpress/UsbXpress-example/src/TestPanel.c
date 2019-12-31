/*--[ USB_MAIN.c ]--------------------------------------------------------------
Copyright 2002 Silicon Laboratories, Inc.
AUTH: JS
DATE: 22 FEB 02

Target: C8051F34x
Tool chain: IAR Systems EW8051 version 7.4x

REVISIONS:
25/01/08 - Modified for IAR Embedded Workbench for 8051 [IHEF]
10/11/06 - PKC: Changed port I/O and ADC Mux settings to accommodate
           potentiometer at P2.5 on the 'F340 Target Board; Changed
           USBXpress API interrupt to 17.
1/24/06  - PKC: Changed 'F320.h to 'F340.h
11/22/02 - DM:  Added support for switches and sample USB
           interrupt application.
4/4/03   - DM:   Ported code to use USB_API.lib instead of custom solution.
5/6/03   - DM: Made changes to use new driver with better throughput.
------------------------------------------------------------------------------*/

/*--[ Includes ]--------------------------------------------------------------*/

#include <stddef.h>
#include "MCU_SELECT.h"
#include "USB_API.h"

/*--[ General definitions ]---------------------------------------------------*/

/* LED defines */
#define Led1 P2_bit.P22
#define Led2 P2_bit.P23

/* Port 2 bits for SW1 and SW2 on the board */
#define Sw1 0x01
#define Sw2 0x02

/*--[ 16-bit SFR Declarations for 'F32x ]-------------------------------------*/

__sfr __no_init volatile unsigned short TMR2RL @ 0xCA; /* Timer2 reload value */
__sfr __no_init volatile unsigned short TMR2   @ 0xCC; /* Timer2 counter */
__sfr __no_init volatile unsigned short ADC0   @ 0xBE;

/*--[ Constants ]-------------------------------------------------------------*/

BYTE Switch1State = 0;                  /* Indicate status of switch */
BYTE Switch2State = 0;                  /* starting at 0 == off */

BYTE Toggle1 = 0;                       /* Variable to make sure each button */
BYTE Toggle2 = 0;                       /* press and release toggles switch */

BYTE Potentiometer = 0x00;              /* Last read potentiometer value */
BYTE Temperature   = 0x00;              /* Last read temperature sensor value */

BYTE Out_Packet[8] = {0,0,0,0,0,0,0,0}; /* Last packet received from host */
BYTE In_Packet[8]  = {0,0,0,0,0,0,0,0}; /* Next packet to sent to host */

__code const BYTE TEMP_ADD = 112;    /* This constant is added to Temperature */

/*--[ USB Descriptor Information ]--------------------------------------------*/

__code const UINT USB_VID          = 0x10C4;
__code const UINT USB_PID          = 0xEA61;
__code const BYTE USB_MaxPower     = 15;      /* Max current = 30 mA (15 * 2) */
__code const UINT USB_bcdDevice    = 0x0100;  /* Device release number 1.00 */
__code const BYTE USB_PwAttributes = 0x80;    /* Bus-powered,
                                                 remote wakeup not supported */

__xdata BYTE USB_SerialStr[]  = {0x0A, 0x03, '1', 0, '2', 0, '3', 0, '4', 0};
__xdata BYTE USB_ProductStr[] =
  {0x10, 0x03, 'U', 0, 'S', 0, 'B', 0, ' ', 0, 'A', 0, 'P', 0, 'I', 0};

__xdata BYTE USB_MfrStr[] =
  {0x1A, 0x03, 'S', 0, 'i', 0, 'l', 0, 'i', 0, 'c', 0, 'o', 0, 'n', 0,
   ' ', 0, 'L', 0, 'a', 0, 'b', 0, 's', 0};

/*--[ Forward declarations ]--------------------------------------------------*/

void Timer_Init(void);
void Adc_Init(void);
void Port_Init(void);
void Suspend_Device(void);
void Initialize(void);

/*--[ Code starts here ]------------------------------------------------------*/

void main(void){

  USB_Clock_Start();    /* Init USB clock *before* calling USB_Init */
  USB_Init( USB_VID, USB_PID, USB_MfrStr, USB_ProductStr, USB_SerialStr,
            USB_MaxPower, USB_PwAttributes, USB_bcdDevice);

  Initialize();
  USB_Int_Enable();

  while (1)  {

    /**************************************************************************
       It is possible that the contents of the following packets can change
       while being updated.  This doesn't cause a problem in the sample
       application because the bytes are all independent.  If data is NOT
       independent, packet update routines should be moved to an interrupt
       service routine, or interrupts should be disabled during data updates.
    ***************************************************************************/

    /* Update LED Status */
    if (Out_Packet[0] == 1) 
      Led1 = 1;
    else 
      Led1 = 0;

    if(Out_Packet[1] == 1)
      Led2 = 1;
    else 
      Led2 = 0;

    P1 = (Out_Packet[2] & 0x0F);  /* Set Port 1 pins */

    In_Packet[0] = Switch1State;  /* Send status of switch 1 */
    In_Packet[1] = Switch2State;  /* and switch 2 to host */
    In_Packet[2] = (P0 & 0x0F);   /* Port 0 [0-3] (make sure j9/j10 jumpered) */
    In_Packet[3] = Potentiometer; /* Potentiometer value */
    In_Packet[4] = Temperature;   /* Temperature sensor value */
  }
}

void Port_Init(void){

/* Configure the Crossbar and GPIO ports */

#if defined(_C8051F34X_)
   P2MDIN   = 0xDF;              /* Port 2 pin 5 set as analog input */
#elif defined(_C8051F320_1_)
   P1MDIN   = 0x7F;              /* Port 1 pin 7 set as analog input */
#endif

   P0MDOUT |= 0x0F;              /* Port 0 pins 0-3 set high impedence */
   P1MDOUT |= 0x0F;              /* Port 1 pins 0-3 set high impedence */
   P2MDOUT |= 0x0C;              /* Port 2 pins 0,1 set high impedence */

#if defined(_C8051F34X_)
   P2SKIP   = 0x20;              /* Port 1 pin 7 skipped by crossbar */
#elif defined(_C8051F320_1_)
   P1SKIP   = 0x80;              /* Port 1 pin 7 skipped by crossbar */
#endif

   XBR0     = 0x00;
   XBR1     = 0x40;              /* Enable Crossbar */
}

void Timer_Init(void){

/* 1 mhz timer 2 reload, used to check if switch pressed on overflow and
 * used for ADC continuous conversion
 */

  TMR2CN  = 0x00;              /* Stop Timer2; Clear TF2; */

  CKCON  &= ~0xF0;             /* Timer2 clocked based on T2XCLK; */
  TMR2RL  = -(24000000 / 12);  /* Initialize reload value */
  TMR2    = 0xffff;            /* Set to reload immediately */

  IE_bit.ET2 = 1;              /* Enable Timer2 interrupts */
  TMR2CN_bit.TR2 = 1;          /* Start Timer2 */
}

void Adc_Init(void){

/* Configures ADC for single ended continuous conversion or Timer2 */

  REF0CN = 0x0E;               /* Enable voltage reference VREF */
  AMX0P  = 0x1E;               /* Positive input starts as temp sensor */
  AMX0N  = 0x1F;               /* Single ended mode(negative input = gnd) */

  ADC0CF = 0xF8;               /* SAR Period 0x1F, Right adjusted output */
  ADC0CN = 0xC2;               /* Continuous converion on timer 2 overflow
                                  with low power tracking mode on */

  EIE1   |= 0x08;              /* Enable conversion complete interrupt */
}

void Suspend_Device(void){

/* Called when a DEV_SUSPEND interrupt is received.
 * - Disables all unnecessary peripherals
 * - Calls USB_Suspend()
 * - Enables peripherals once device leaves suspend state
 */

  /* Disable peripherals before calling USB_Suspend() */

  P0MDIN = 0x00;    /* Port 0-3 configured as analog input */
  P1MDIN = 0x00;
  P2MDIN = 0x00;
  P3MDIN = 0x00;

  ADC0CN &= ~0x80;  /* Disable ADC0 */
  IE_bit.ET2 = 0;   /* Disable Timer 2 Interrupts */


  USB_Suspend();    /* Put the device in suspend state */

  /* Reenable peripherals once the device leaves the suspend state */

  ADC0CN |= 0x80;   /* Enable ADC0 */

  P0MDIN = 0xFF;
  P1MDIN = 0x7F;    /* Port 1 pin 7 set as analog input */
  P2MDIN = 0xFF;
  P3MDIN = 0x01;

  IE_bit.ET2 = 1;   /* Enable Timer 2 Interrupts */
}

void Initialize(void){

/* Called when a DEV_CONFIGURED interrupt is received.
 * - Enables all peripherals needed for the application
 */

  Port_Init();    /* Initialize crossbar and GPIO */
  Timer_Init();   /* Initialize timer2 */
  Adc_Init();     /* Initialize ADC */
}

#pragma vector=0x2b
__interrupt void Timer2_ISR(void){

/* Called when timer 2 overflows, check to see if switch is pressed,
 * then watch for release.
 */

  if (!(P2 & Sw1)){       /* Check for switch #1 pressed */
    if (Toggle1 == 0){

      /* Toggle is used to debounce switch
       *  so that one press and release will
       *  toggle the state of the switch sent
       *  to the host
       */

        Switch1State = ~Switch1State;
        Toggle1 = 1;
    }
  }
  else
    Toggle1 = 0;          /* Reset toggle variable */

  if (!(P2 & Sw2)){       /* Same as above, but for Switch2 */
    if (Toggle2 == 0){

      Switch2State = ~Switch2State;
      Toggle2 = 1;
    }
  }
  else
    Toggle2 = 0;

  TMR2CN_bit.TF2H = 0;    /* Clear Timer2 interrupt flag */
}

#pragma vector=0x53
__interrupt void Adc_ConvComplete_ISR(void){

/* Called after a conversion of the ADC has finished
 * - Updates the appropriate variable for either
 *   potentiometer or temperature sensor
 * - Switches the Adc multiplexor value to switch between
 *   the potentiometer and temp sensor
 */

  if(AMX0P == 0x1E){

    /* This switches the AMUX between
     * the temperature sensor and the
     * potentiometer pin
     */

    Temperature  = ADC0L;
    Temperature += TEMP_ADD;
    AMX0P        = 0x04;      /* switch to potentiometer ('F340 - P2.5) */
    ADC0CF       = 0xFC;      /* place ADC0 in left-adjusted mode */
  }
  else {
    Potentiometer = ADC0H;
    AMX0P         = 0x1E;     /* switch to temperature sensor */
    ADC0CF        = 0xF8;     /* place ADC0 in right-adjusted mode */
  }

  ADC0CN_bit.AD0INT = 0;
  Block_Write(In_Packet, 8);
}

#if defined(_C8051F34X_)
  #pragma vector=0x8b          // interrupt 17
#else
  #pragma vector=0x83          // interrupt 16
#endif
__interrupt void  USB_API_TEST_ISR(void){

  /* ISR called by USB_API */

  BYTE INTVAL = Get_Interrupt_Source();

  if (INTVAL & RX_COMPLETE){
    Block_Read(Out_Packet, 8);
  }

  if (INTVAL & DEV_SUSPEND){
    Suspend_Device();
  }

  if (INTVAL & DEV_CONFIGURED){
    Initialize();
  }
}
