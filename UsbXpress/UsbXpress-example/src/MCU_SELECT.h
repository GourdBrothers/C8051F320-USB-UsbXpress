/*--[MCU_SELECT.h ]------------------------------------------------------------
(c) 2005 Silicon Laboratories, Inc.
http://www.silabs.com/mcu

Filename  : MCU_SELECT.h
Project   : USBXpress Firmware Library
Target    : C8051F320/1/6/7, 'F340/1/2/3/4/5/6/7
Tool Chain: IAR Embedded Workbench 7.x
Version   : 3.0
Build Date: 06FEB2008

Revision History:
Rev3.0 (PKC-20APR2006)
 -Added condition for 'F34x devices. Removed macro definitions from this
  file -- one of the valid macros is defined during compilation via the
  command line.
Rev2.4 (PKC-06DEC2005)
 -No changes; version number incremented to match project version
Rev2.3 (PKC)
 -No changes; version number incremented to match project version
Rev2.2 (PKC-05MAY2005)
 -Original Revision.

Description:
 Header file for selecting which MCU the USBXpress code will be compiled for.
------------------------------------------------------------------------------*/

#ifndef _MCU_SELECT_H_
#define _MCU_SELECT_H_

/* Only one of the following should be defined during compile-time.
 * This is done via the compiler command line option DEFINE().
 * See one of the build batch files (*.bat) for details.
 */

  #if defined(_C8051F320_1_)
    #include <ioC8051F320.h>
  #elif defined(_C8051F326_7_)
    #include <ioC8051F326.h>
  #elif defined(_C8051F34X_)
    #include <ioC8051F340.h>
  #else
    #error Invalid microcontroller choice. See MCU_SELECT.h for details.
  #endif

#endif
