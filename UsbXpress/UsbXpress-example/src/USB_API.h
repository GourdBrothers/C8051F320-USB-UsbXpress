/*--[ USB_API.h ]--------------------------------------------------------------
(c) 2005 Silicon Laboratories, Inc.
http://www.silabs.com/mcu

Filename  : USB_API.h
Project   : USBXpress Firmware Library
Target    : C8051F320/1/6/7, 'F340/1/2/3/4/5/6/7
Tool Chain: IAR Embedded Workbench 7.x
Version   : 3.0
Build Date: 06FEB2008

Revision History:

Rev3.0 (PKC-20APR2006)
 -No changes; version number incremented to match project version
Rev2.4 (PKC-06DEC2005)
 -Added function USB_Get_Library_Version
Rev2.3 (PKC)
 -No changes; version number incremented to match project version
Rev2.2 (PKC-17MAR2005)
 -Added file description header
 -Added new function declaration - USB_Clock_Start()

Description:
 Header file for the USBXpress firmware library. Includes function
 prototypes, type definitions, and function return value macro definitions.
-----------------------------------------------------------------------------*/

#ifndef  _USB_API_H_
#define  _USB_API_H_

/* UINT type definition */
#ifndef _UINT_DEF_
  #define _UINT_DEF_
  typedef unsigned int UINT;
#endif

/* BYTE type definition */
#ifndef _BYTE_DEF_
  #define _BYTE_DEF_
  typedef unsigned char BYTE;
#endif

/* Get_Interrupt_Source() return value bit masks
   Note: More than one bit can be set at the same time. */
#define USB_RESET      0x01  /* USB Reset Interrupt has occurred */
#define TX_COMPLETE    0x02  /* Transmit Complete Interrupt has occurred */
#define RX_COMPLETE    0x04  /* Receive Complete Interrupt has occurred */
#define FIFO_PURGE     0x08  /* Command received (and serviced) from the host
                                to purge the USB buffers */
#define DEVICE_OPEN    0x10  /* Device Instance Opened on host side */
#define DEVICE_CLOSE   0x20  /* Device Instance Closed on host side */
#define DEV_CONFIGURED 0x40  /* Device has entered configured state */
#define DEV_SUSPEND    0x80  /* USB suspend signaling present on bus */

/* Function prototypes */
void USB_Clock_Start(void);
void USB_Init(UINT,UINT,BYTE*,BYTE*,BYTE*,BYTE,BYTE,UINT);
UINT Block_Write(BYTE __generic *, UINT);
BYTE Block_Read(BYTE __generic *, BYTE);
BYTE Get_Interrupt_Source(void);
void USB_Int_Enable(void);
void USB_Int_Disable(void);
void USB_Disable(void);
void USB_Suspend(void);
UINT USB_Get_Library_Version(void);

#endif
