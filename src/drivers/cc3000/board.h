/**************************************************************************
*
*  This file is part of the ArduinoCC3000 library.

*  Version 1.0.1b
*
*  Copyright (C) 2013 Chris Magagna - cmagagna@yahoo.com
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  Don't sue me if my code blows up your board and burns down your house
*
*
*  This file is the main module for the Arduino CC3000 library.
*  Your program must call CC3000_Init() before any other API calls.
*
****************************************************************************/

/*
	Some things are different for the Teensy 3.0, so set a flag if we're using
	that hardware.
*/

#include <stdint.h>

































































































































//#define MAC_ADDR_LEN	6



#define DISABLE	(0)

#define ENABLE	(1)

//AES key "smartconfigAES16"
//const uint8_t smartconfigkey[] = {0x73,0x6d,0x61,0x72,0x74,0x63,0x6f,0x6e,0x66,0x69,0x67,0x41,0x45,0x53,0x31,0x36};





/* If you uncomment the line below the library will leave out a lot of the
   higher level functions but use a lot less memory. From:

   http://processors.wiki.ti.com/index.php/Tiny_Driver_Support

  CC3000's new driver has flexible memory compile options.

  This feature comes in handy when we want to use a limited RAM size MCU.

  Using The Tiny Driver Compilation option will create a tiny version of our
  host driver with lower data, stack and code consumption.

  By enabling this feature, host driver's RAM consumption can be reduced to
  minimum of 251 bytes.

  The Tiny host driver version will limit the host driver API to the most
  essential ones.

  Code size depends on actual APIs used.

  RAM size depends on the largest packet sent and received.

  CC3000 can now be used with ultra low cost MCUs, consuming 251 byte of RAM
  and 2K to 6K byte of code size, depending on the API usage. */

//#define CC3000_TINY_DRIVER	1





extern uint8_t asyncNotificationWaiting;
extern long lastAsyncEvent;
extern uint8_t dhcpIPAddress[];



extern void CC3000_Init(void);


extern volatile unsigned long ulSmartConfigFinished,
	ulCC3000Connected,
	ulCC3000DHCP,
	OkToDoShutDown,
	ulCC3000DHCP_configured;

extern volatile uint8_t ucStopSmartConfig;
