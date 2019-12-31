/*--[ FileTransfer.c ]---------------------------------------------------------
Author   DM  DATE 4-4-03
Modified CS  DATE 8-25-03
Modified PKC DATE 1-24-06  (changed 'F320.h to 'F340.h)
Modified PKC DATE 10-11-06 (changed USBXpress API interrupt to 17)

This example illustrates usage of the USB_API.lib

DO NOT locate code segments at 0x1400 to 0x4000
These are used for non-voltile storage for transmitted data.
------------------------------------------------------------------------------*/

/*--[ Include files ]---------------------------------------------------------*/

#include <stddef.h>         /* Used for NULL pointer definition */
#include "MCU_SELECT.h" /* Header file for SiLabs c8051f34x */
#include "USB_API.h"    /* Header file for USB_API.lib */

/*--[ Bit Definitions ]-------------------------------------------------------*/

#define Led1 P2_bit.P22   /* LED='1' means ON */
#define Led2 P2_bit.P23   /* These blink to indicate data transmission */

/*--[ Constants Definitions ]-------------------------------------------------*/

/* Total number of flash pages to be used for file storage */
#define NUM_STG_PAGES 20

/* Use the maximum read block size of 64 bytes */
#define MAX_BLOCK_SIZE_READ 64

/* Use the maximum write block size of 4096 bytes */
#define MAX_BLOCK_SIZE_WRITE 4096

/* Size of each flash page */
#define FLASH_PAGE_SIZE 512

#define BLOCKS_PR_PAGE FLASH_PAGE_SIZE/MAX_BLOCK_SIZE_READ
#define MAX_NUM_BYTES   FLASH_PAGE_SIZE*NUM_STG_PAGES
#define MAX_NUM_BLOCKS  BLOCKS_PR_PAGE*NUM_STG_PAGES

/* Message Types for communication with host */
#define READ_MSG  0x00
#define WRITE_MSG 0x01
#define SIZE_MSG  0x02
#define DELAYED_READ_MSG 0x05

/* Machine States */
#define ST_WAIT_DEV 0x01 /* Wait for application to open a device instance */
#define ST_IDLE_DEV 0x02 /* Device is open, wait for Setup Message from host */
#define ST_RX_SETUP 0x04 /* Received Setup Message, decode and wait for data */
#define ST_RX_FILE  0x08 /* Receive file data from host */
#define ST_TX_FILE  0x10 /* Transmit file data to host */
#define ST_TX_ACK   0x20 /* Transmit ACK 0xFF back to host after every 8
                            packets */
#define ST_ERROR    0x80 /* Error state */

/* Structure definition of a block of data */
typedef struct {
   BYTE  Piece[MAX_BLOCK_SIZE_READ];
}  BLOCK;

/* Structure definition of a flash memory page */
typedef struct {
   BYTE  FlashPage[FLASH_PAGE_SIZE];
}  PAGE;

/*--[ Constants ]-------------------------------------------------------------*/

/* Temporary storage of between flash writes */
__xdata BLOCK TempStorage[BLOCKS_PR_PAGE];

__data BYTE __code *PageIndices[20]={
  (BYTE __code *)0x2200, (BYTE __code *)0x2400, (BYTE __code *)0x2600,
  (BYTE __code *)0x2800, (BYTE __code *)0x2A00, (BYTE __code *)0x2C00,
  (BYTE __code *)0x2E00, (BYTE __code *)0x3000, (BYTE __code *)0x3200,
  (BYTE __code *)0x3400, (BYTE __code *)0x3600, (BYTE __code *)0x3800,
  (BYTE __code *)0x3A00, (BYTE __code *)0x3C00, (BYTE __code *)0x3E00,
  (BYTE __code *)0x4000, (BYTE __code *)0x4200, (BYTE __code *)0x4400,
  (BYTE __code *)0x4600
};

__data BYTE  Buffer[3];         /* buffer for setup messages */
__data BYTE *ReadIndex;
__data UINT  BytesToRead, WriteStageLength, ReadStageLength, NumBytes,
             BytesRead, BytesToWrite, BytesWrote;
__data BYTE  M_State, BlockIndex, PageIndex, BlocksWrote, NumBlocks;

__code const BYTE LengthFile[3] @ 0x2000;

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

void Port_Init(void);       /* Initialize Ports Pins and Enable Crossbar */
void State_Machine(void);   /* Determine new state and act on current state */
void Receive_Setup(void);   /* Receive and decode setup packet from host */
void Receive_File(void);    /* Receive file data from host */

void Page_Erase(BYTE __code *);    /* Erase a flash page */
void Page_Write(BYTE __code *);    /* Write a flash page */

/*--[ Code starts here ]------------------------------------------------------*/

void main(void){

  USB_Clock_Start();    /* Init USB clock *before* calling USB_Init */
  USB_Init(USB_VID, USB_PID, USB_MfrStr, USB_ProductStr, USB_SerialStr,
           USB_MaxPower, USB_PwAttributes, USB_bcdDevice);

  CLKSEL |= 0x02;
  RSTSRC   |= 0x02;

  Port_Init();          /* Initialize crossbar and GPIO */
  USB_Int_Enable();     /* Enable USB_API Interrupts */

  while (1);
}

void  Port_Init(void){

  P2MDOUT |= 0x0C;      /* Port 2 pins 0,1 set high impedence */
#ifndef _C8051F326_7_
  XBR0     = 0x00;
  XBR1     = 0x40;      /* Enable Crossbar */
#endif
}

void  Page_Erase(BYTE __code *Page_Address){
  BYTE EA_Save;
  BYTE __xdata *pwrite;

  EA_Save   = IE_bit.EA;   /* Save current EA */
  IE_bit.EA = 0;           /* Turn off interrupts */

  /* Set write pointer to Page_Address */
  pwrite = (BYTE __xdata *)(Page_Address);

  PSCTL = 0x03;            /* Enable flash erase and writes */
  FLKEY = 0xA5;            /* Write flash key sequence to FLKEY */
  FLKEY = 0xF1;

  *pwrite  =  0x00;        /* Erase flash page using a write command */
  PSCTL =  0x00;           /* Disable flash erase and writes */

  IE_bit.EA =  EA_Save;    /* Restore state of EA */
}

void  Page_Write(BYTE __code *PageAddress){

  BYTE  EA_Save;
  BYTE  __xdata *pwrite, *pread;
  UINT  x;

  pread = (BYTE __xdata *)(TempStorage);

  /* Save EA & disable interrupts */
  EA_Save = IE_bit.EA;
  IE_bit.EA = 0;

  pwrite = (BYTE __xdata *)(PageAddress);

  PSCTL  =  0x01;             // Enable flash writes

  for(x = 0;  x<FLASH_PAGE_SIZE;   x++){

     FLKEY =  0xA5;         /* Write flash key sequence */
     FLKEY =  0xF1;
     *pwrite  =  *pread;    /* Write data byte to flash */

     pread++;
     pwrite++;
  }

  /* Disable flash writes & restore EA */
  PSCTL =  0x00;
  IE_bit.EA =  EA_Save;
}

void  State_Machine(void){

  switch   (M_State){
    case  ST_RX_SETUP:    /* Receive and decode host Setup Message */
      Receive_Setup();
      break;
    case  ST_RX_FILE:     /* Receive File data from host */
      Receive_File();
      break;
    case  ST_TX_ACK:      /* Ack Transmit complete, continue RX data */
      M_State = ST_RX_FILE;
      break;
    case  ST_TX_FILE:     /* Send file data to host */
      WriteStageLength = ((BytesToWrite - BytesWrote) > MAX_BLOCK_SIZE_WRITE) ?
                           MAX_BLOCK_SIZE_WRITE : (BytesToWrite - BytesWrote);

      BytesWrote += Block_Write((BYTE __code *)(ReadIndex), WriteStageLength);
      ReadIndex  += WriteStageLength;

      if ((BlocksWrote%8) == 0)
        Led2 = ~Led2;
      if (BytesWrote == NumBytes)
        Led2 = 0;
      break;
    default:
      break;
  }
}

#if defined(_C8051F320_1_)
  #pragma vector=0x83
#elif defined(_C8051F326_7_)
  #pragma vector=0x83
#elif defined(_C8051F34X_)
  #pragma vector=0x8b
#else
  #error Invalid microcontroller choice. See MCU_SELECT.h for details.
#endif

__interrupt void  USB_API_TEST_ISR(void){

  BYTE INTVAL = Get_Interrupt_Source(); /* Determine type of API interrupts */

  if (INTVAL  &  USB_RESET)    /* Bus Reset Event, go to Wait State */
    M_State  =  ST_WAIT_DEV;

  if (INTVAL  &  DEVICE_OPEN)  /* Device opened on host, go to Idle */
    M_State  =  ST_IDLE_DEV;

  if (INTVAL  &  TX_COMPLETE){

    if (M_State == ST_RX_FILE)   /* Ack Transmit complete, go to RX state */
      M_State  =  (ST_TX_ACK);

    /* Transmit complete? Transmit the file, Idle when done */
    if (M_State == ST_TX_FILE)
      M_State  =  (BytesWrote == BytesToWrite) ? ST_IDLE_DEV :ST_TX_FILE;
  }

  if (INTVAL  &  RX_COMPLETE)  /* RX Complete? RX Setup or RX file state */
    M_State  =  (M_State == ST_IDLE_DEV) ? ST_RX_SETUP : ST_RX_FILE;

  if (INTVAL  &  DEVICE_CLOSE)  /* Device closed, wait for re-open */
    M_State  =  ST_WAIT_DEV;

  if (INTVAL  &  FIFO_PURGE)   /* Fifo purged, go to Idle State */
    M_State  =  ST_IDLE_DEV;

  State_Machine();    /* Call state machine routine */
}

void  Receive_Setup(void){

  BytesRead = Block_Read(Buffer, 3);  /* Read Setup Message */

  if (Buffer[0] == READ_MSG){         /* Check See if Read File Setup */

     PageIndex = 0;                   /* Reset Index */
     NumBlocks = LengthFile[2];       /* Read NumBlocks from flash stg */
     NumBlocks = (NumBlocks > MAX_NUM_BLOCKS) ? /* only write as many bytes */
                  MAX_NUM_BLOCKS : NumBlocks;   /* as we have space available */

     Buffer[0]    = SIZE_MSG;         /* Send host size of transfer message */
     Buffer[1]    = LengthFile[1];
     Buffer[2]    = LengthFile[0];
     BytesToWrite = Buffer[1] + 256*Buffer[2];
     BytesWrote   = Block_Write((BYTE*)Buffer, 3);
     M_State      = ST_TX_FILE;       /* Go to TX data state */
     BytesWrote   = 0;
     ReadIndex    = (BYTE *)PageIndices[0];
     Led2         = 1;
  }
  else{                               /* Otherwise assume Write Setup Packet */

    BytesToRead = Buffer[1] + 256*Buffer[2];
    NumBlocks   = (BYTE)(BytesToRead/MAX_BLOCK_SIZE_READ);  /* Find NumBlocks */

    if (BytesToRead > MAX_NUM_BYTES)  /* State Error if transfer too big */
      M_State = ST_ERROR;
    else{

      if (BytesToRead%MAX_BLOCK_SIZE_READ)
        NumBlocks++;    /* Increment NumBlocks for last partial block */

      TempStorage->Piece[0] = Buffer[2];
      TempStorage->Piece[1] = Buffer[1];
      TempStorage->Piece[2] = NumBlocks;

      /* Write Values to Flash */
      Page_Erase((BYTE __code *)0x2000);
      Page_Write((BYTE __code *)0x2000);

      PageIndex   =  0;          /* Reset Index */
      BlockIndex  =  0;
      BytesRead   =  0;
      Led1        =  1;
      M_State     = ST_RX_FILE;  /* Go to RX data state */
    }
  }
}

void Receive_File(void){

  ReadStageLength = ((BytesToRead - BytesRead) > MAX_BLOCK_SIZE_READ) ?
                      MAX_BLOCK_SIZE_READ : (BytesToRead - BytesRead);

  /* Read Block */
  BytesRead += Block_Read((BYTE*)(&TempStorage[BlockIndex]), ReadStageLength);

  BlockIndex++;

  /* If device has received as many bytes as fit on one FLASH page,
   * disable interrupts, write page to flash, reset packet index,
   * enable interrupts. Send handshake packet 0xFF to host after FLASH write
   */
  if ((BlockIndex == (BLOCKS_PR_PAGE)) || (BytesRead == BytesToRead)){
    Page_Erase((BYTE __code *)(PageIndices[PageIndex]));
    Page_Write((BYTE __code *)(PageIndices[PageIndex]));
    PageIndex++;
    Led1 = ~Led1;
    BlockIndex = 0;
    Buffer[0] = 0xFF;
    Block_Write(Buffer, 1);  /* Send handshake Acknowledge to host */
  }

  /* Go to Idle state if last packet has been received */
  if (BytesRead == BytesToRead){
    M_State = ST_IDLE_DEV;
    Led1 = 0;
  }
}
