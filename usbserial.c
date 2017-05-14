#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"
#include "leds.h"

//
// LEDS
#define LED_PORT 	GPIO_PORTE_BASE
#define LED_ON 		GPIO_PIN_0
#define LED_1 		GPIO_PIN_1
#define LED_2 		GPIO_PIN_2
//
//
//
static unsigned char mydata;
extern char usb_date[7];
extern char usb_time[7];
int usb_counter = 0;
extern int USBDataloggingFlag;
extern int USBProcessDataFlag;
extern unsigned char RxDataFrame[9];

//*****************************************************************************
//
// This module manages the USB serial device function. It is used when the
// eval board is connected to a host PC as a serial device, and can transmit
// data log records to the host PC through a virtual serial port.
//
//*****************************************************************************

//*****************************************************************************
//
// Global flag indicating that a USB device configuration is made.
//
//*****************************************************************************
static volatile bool g_bUSBDevConnected = false;

//*****************************************************************************
//
// The line coding parameters for the virtual serial port.  Since there is
// no physical port this does not have any real effect, but we have a default
// set of values to report if asked, and will remember whatever the host
// configures.
//
//*****************************************************************************
static tLineCoding g_sLineCoding =
{
		115200, USB_CDC_STOP_BITS_1, USB_CDC_PARITY_NONE, 8
};

bool isCDCConnected(void)
{
	return g_bUSBDevConnected;
}
//*****************************************************************************
//
// Set the communication parameters for the virtual serial port.
//
//*****************************************************************************
static bool
SetLineCoding(tLineCoding *psLineCoding)
{
	//
	// Copy whatever the host passes into our copy of line parameters.
	//
	memcpy(&g_sLineCoding, psLineCoding, sizeof(tLineCoding));

	//
	// ALways return success
	//
	return(true);
}

//*****************************************************************************
//
// Get the communication parameters in use on the UART.
//
//*****************************************************************************
static void
GetLineCoding(tLineCoding *psLineCoding)
{
	//
	// Copy whatever we have stored as the line parameter to the host.
	//
	memcpy(psLineCoding, &g_sLineCoding, sizeof(tLineCoding));
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
		uint32_t ui32MsgValue, void *pvMsgData)
{
	//
	// Which event are we being asked to process?
	//
	switch(ui32Event)
	{
	//
	// We are connected to a host and communication is now possible.
	//
	case USB_EVENT_CONNECTED:
	{
		g_bUSBDevConnected = true;
		//led_on(VERDE);
		GPIOPinWrite(LED_PORT, LED_1, LED_1);
		// Flush our buffers.
		//
		USBBufferFlush(&g_sTxBuffer);
		USBBufferFlush(&g_sRxBuffer);
		break;
	}

	//
	// The host has disconnected.
	//
	case USB_EVENT_DISCONNECTED:
	{
		g_bUSBDevConnected = false;
		//led_off(VERDE);
		GPIOPinWrite(LED_PORT, LED_ON, 0);
		break;
	}

	//
	// Return the current serial communication parameters.
	//
	case USBD_CDC_EVENT_GET_LINE_CODING:
	{
		GetLineCoding(pvMsgData);
		break;
	}

	//
	// Set the current serial communication parameters.
	//
	case USBD_CDC_EVENT_SET_LINE_CODING:
	{
		SetLineCoding(pvMsgData);
		break;
	}

	//
	// The following line control events can be ignored because there is
	// no physical serial port to manage.
	//
	case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
	case USBD_CDC_EVENT_SEND_BREAK:
	case USBD_CDC_EVENT_CLEAR_BREAK:
	{
		break;
	}

	//
	// Ignore SUSPEND and RESUME for now.
	//
	case USB_EVENT_SUSPEND:
	case USB_EVENT_RESUME:
	{
		break;
	}

	//
	// An unknown event occurred.
	//
	default:
	{
		break;
	}
	}

	//
	// Return control to USB stack
	//
	return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
		void *pvMsgData)
{
	//
	// Which event have we been sent?
	//
	switch(ui32Event)
	{
	case USB_EVENT_TX_COMPLETE:
	{
		//
		// Since we are using the USBBuffer, we don't need to do anything
		// here.
		//
		break;
	}

	//
	// We don't expect to receive any other events.
	//
	default:
	{
		break;
	}
	}
	return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this
// channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
		void *pvMsgData)
{
	//
	// Which event are we being sent?
	//
	switch(ui32Event)
	{
	//
	// A new packet has been received.
	//
	case USB_EVENT_RX_AVAILABLE:
	{
		//
		// We do not ever expect to receive serial data, so just flush
		// the RX buffer if any data actually comes in.
		//
		//USBBufferFlush(&g_sRxBuffer);
		// Si se desea leer deber�a utilizarce algo como:
		//
		//USBBufferRead(&g_sRxBuffer, &data, 1);
		// TODO Receive
		unsigned char mystring[9];
		uint32_t cnt;
		//
		cnt = USBBufferRead(&g_sRxBuffer, RxDataFrame, 9);
		//USBSerialSend("1\n\r", 3);
		//
		if(RxDataFrame[0] == 0x30)
		{
			//USBSerialSend("2\n\r", 3);
			USBProcessDataFlag = 1;
		}
		//
		//USBBufferFlush(&g_sRxBuffer);
		//
		break;
	}

	//
	// We are being asked how much unprocessed data we have still to
	// process.  Since there is no actual serial port and we are not
	// processing any RX data, just return 0.
	//
	case USB_EVENT_DATA_REMAINING:
	{
		return(0);
	}

	//
	// We are being asked to provide a buffer into which the next packet
	// can be read. We do not support this mode of receiving data so let
	// the driver know by returning 0. The CDC driver should not be sending
	// this message but this is included just for illustration and
	// completeness.
	//
	case USB_EVENT_REQUEST_BUFFER:
	{
		return(0);
	}

	//
	// We don't expect to receive any other events.
	//
	default:
	{
		break;
	}
	}
	return(0);
}

//*****************************************************************************
//
// Initializes the USB serial device.
//
//*****************************************************************************
extern void USB0DeviceIntHandler(void);

void USBSerialInit(void)
{
	IntRegister(INT_USB0, USB0DeviceIntHandler);
	IntEnable(INT_USB0);
	IntMasterEnable();
	//USBStackModeSet(0, eUSBModeDevice, 0);
	USBStackModeSet(0, eUSBModeForceDevice, 0);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);
	USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
	USBBufferInit((tUSBBuffer *)&g_sRxBuffer);
	USBDCDCInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);
}

//*****************************************************************************
//
// Finishes the USB serial device.
//
//*****************************************************************************
void USBSerialKaput(void){
	//
	IntDisable(INT_USB0);
	USBDCDCTerm( (tUSBDCDCDevice *)&g_sCDCDevice );
}

//*****************************************************************************
//
// This is called by the application main loop to perform regular processing.
// This is just a stub here because everything is event or interrupt driven.
//
//*****************************************************************************
void
USBSerialRun(void)
{
}

void USBSerialSend(unsigned char *toSend, unsigned int size)
{
	USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, toSend, size);
}
