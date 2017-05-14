/*-------------------------------------------------------------------------------------------
 ********************************************************************************************
 *-------------------------------------------------------------------------------------------
 *
 *				DATA LOGGER DE VARIABLES AMBIENTALES INTERNAS
 *							CIMEC CONICET - UTN FRP
 *								     2016
 *
 *						Polo, Franco		fjpolo@frp.utn.edu.ar
 *						Burgos, Sergio		sergioburgos@frp.utn.edu.ar
 *						Bre, Facundo		facubre@cimec.santafe-conicet.gov.ar
 *
 *	main.c
 *
 *	Descripción:
 *
 *  Desarrollo del firmware de la placa base del data logger, constando de:
 *
 *  - Periféricos I2C:
 *  	a) HR y Tbs		HIH9131		0b0100111		0x27
 *  	b) Ev			TSL2563		0b0101001		0x29
 *  	c) Va			ADS			0b1001000		0x48
 *  	d) Tg			LM92		0b1001011		0x51
 *  	e) RTC			DS1703		0b1101000		0x68
 *
 *  - Periféricos OneWire@PD6
 *  	a) Ts01			MAX31850	ROM_Addr		0x3B184D8803DC4C8C
 *  	b) Ts02			MAX31850	ROM_Addr		0x3B0D4D8803DC4C3C
 *  	c) Ts03			MAX31850	ROM_Addr		0x3B4D4D8803DC4C49
 *  	d) Ts04			MAX31850	ROM_Addr		0x3B234D8803DC4C99
 *  	e) Ts05			MAX31850	ROM_Addr		0x3B374D8803DC4C1E
 *  	f) Ts06			MAX31850	ROM_Addr
 *
 *  	//3
 *		//1
 *   	//2
 *		//4
 *		//0
 *
 *  - IHM
 *  	a) RESET		!RST
 *  	b) SW_SD		PC6
 *  	c) SW_ON		PC5
 *  	d) SW_1			PC7
 *  	e) WAKE			!Wake
 *  	f) LEDON		PE0
 *  	g) LED1			PE1
 *  	h) LED2			PE2
 *
 *  - SD
 *  	a) SD_IN		PA6
 *  	b) SD_RX		PA4
 *  	c) SD_TX		PA5
 *  	d) SD_CLK		PA2
 *  	e) SD_FSS		PA3
 *
 *--------------------------------------------------------------------------------------------
 *********************************************************************************************
 *-------------------------------------------------------------------------------------------*/

// standard C
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
// inc
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
// driverlib
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/hibernate.h"
#include "driverlib/i2c.h"
#include "driverlib/eeprom.h"
#include "driverlib/flash.h"
// datalogger
#include "datalogger/datalogger.h"
#include "datalogger/delay.h"
// third_party
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"
// usbserial
#include "usbserial.h"
//#include "usbcdc.h"
// leds
#include "leds.h"
// OneWire
#include "oneWire.h"

/*****************************************************************************************************
 * Defines
 ****************************************************************************************************/
// SWITCHES
#define SW_PORT 	GPIO_PORTC_BASE
#define SW_ON 		GPIO_PIN_5
#define SW_SD 		GPIO_PIN_6
#define SW_1 		GPIO_PIN_7
// LEDS
#define LED_PORT 	GPIO_PORTE_BASE
#define LED_ON 		GPIO_PIN_0
#define LED_1 		GPIO_PIN_1
#define LED_2 		GPIO_PIN_2
// SD Card
#define SD_PORT		GPIO_PORTA_BASE
#define SD_IN		GPIO_PIN_6
#define SD_RX		GPIO_PIN_4
#define SD_TX		GPIO_PIN_5
#define SD_CLK		GPIO_PIN_2
#define SD_FSS		GPIO_PIN_3
// Timer0
#define TOGGLE_FREQUENCY 1
// Tg
#define SLAVE_ADDRESS_TG 0x4B
#define TG_REG_READ 0x00
#define TG_REG_LOWPOW 0x01
// Ev
#define SLAVE_ADDRESS_EV 0x29
// RTC
#define SLAVE_ADDR_RTC 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
// Va ADC slave address
#define SLAVE_ADDR_VA 0x48
#define VA_REG_READ 0x00
// I2C3
#define GPIO_PD0_I2C3SCL        0x00030003
#define GPIO_PD1_I2C3SDA        0x00030403
// SSI1

/*********************************************************************************************
 * Global variables
 * ******************************************************************************************/
//
int SWRead;
//
int SWRead, SD_Read;
// Estado del modulo hibernacion
unsigned long ulStatus;
//unsigned long ulPeriod;
//Datos a mantener durante la hibernacion
unsigned long HibReg[13] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//0: sensor flag
//1: hibernate flag
//2-3: year
//4-5: month
//6-7: day
//8-9: hour
//10-11: time
//12: period
//
//
// SD Card variables
//
FATFS FatFs;    	/* Work area (file system object) for logical drive */
FIL fil;        	/* File object */
//
FRESULT fr;     	/* FatFs return code */
FILINFO fno;
UINT br;    		/* File read count */
//
DIR *dir;
//static FILINFO fileInfo;
//
int i=0;
const char null[]="0";
const char newline[] = "\r\n";
const char comma[] = ";";
char line[512];
int filesize;
char dir_date[9];
char dir_time[6];
//
//
GPIOIntState;
//
int USBDataloggingFlag = 0;
int USBProcessDataFlag;
unsigned char RxDataFrame[9];
unsigned char PeriodBytes[4];
int j=0;
//
uint32_t hibPeriod;
uint32_t flashTest[2];
uint32_t flashPeriod[2];
int ui8Index;
uint32_t read_byte[1];
//
unsigned char sec,min,hour,day,date,month,year;

/*********************************************************************************************
 * Main loop
 *
 * - Final code take 5
 *
 *  Pulsando SW1 comienza la adquisición. Crea un directorio con la fecha, adentro otro con la
 * hora y adentro el archivo DATA.csv. Comienza la hibernación y en cada interrupción se
 * agregan los datos de las nuevas mediciones al mismo archivo. Pulsando SW_ON se cierra el
 * archivo y deja de adquirir.
 *
 * Agregando USB. Si el USB esta conectado, se deshabilita la funcion de hibernacion, y el mecanismo
 * de funcionamiento es distinto.
 *
 * Trama USB
 * ___________________________
 * 0x30 | Comando | ... | CRC|
 * ---------------------------
 *
 * Comandos:
 *
 * Set T&D:			0x33
 * Set Period:		0x35
 * Get Tbs:			0x41
 * Get HR:
 * Get Tg:
 * Get Ev:
 * Get Ts:
 *
 *
 ********************************************************************************************/
int main(void) {
	// Initialize clock, GPIO, I2C, GPIOInt, SDCard, SPI
	Initialize();
	// Enable SW_1 interrupt
	IntEnable(INT_GPIOC);
	//
	// Check battery
	//
	sec = GetClock(SEC);
	min = GetClock(MIN);
	hour = GetClock(HRS);
	date = GetClock(DATE);
	month = GetClock(MONTH);
	year = GetClock(YEAR);
	if(sec == 165  && min == 165)
	{
		while(1)
		{
			// Bateria baja
			GPIOPinWrite(LED_PORT,LED_1, LED_1);
			GPIOPinWrite(LED_PORT,LED_2, LED_2);
			GPIOPinWrite(LED_PORT,LED_ON, LED_ON);
		}
	}
	//
	//
	//
	//FlashErase(0x39000);
	//FlashProgram(flashPeriod, 0x39000, sizeof(flashPeriod));
	for(ui8Index=0; ui8Index<10; ui8Index++)
	{
	read_byte[ui8Index] = HWREG(0x39000+(ui8Index*0x4));
	}
	hibPeriod = read_byte[0];
	/*
	if(read_byte[0] == flashPeriod[0])
	{
		delayMS(1);
	}
	*/
	//
	// Check for hibernation module to be active and the reason of the reset
	//
	if(HibernateIsActive())
	{
		//Leo el estado y determino la causa de despertarse
		ulStatus = HibernateIntStatus(false);
		//
		//Controlo la razon de haberse despertado
		//
		// RTC wake
		if(ulStatus & HIBERNATE_INT_RTC_MATCH_0)
		{
			//Se despierta de una hibernación
			//
			//Recupero mis flags y path
			HibernateDataGet(HibReg, 13);
			// Date directory
			dir_date[0] = HibReg[2];
			dir_date[1] = HibReg[3];
			dir_date[2] = '-';
			dir_date[3] = HibReg[4];
			dir_date[4] = HibReg[5];
			dir_date[5] = '-';
			dir_date[6] = HibReg[6];
			dir_date[7] = HibReg[7];
			dir_date[8] = '\0';
			// Time directory
			dir_time[0] = HibReg[8];
			dir_time[1] = HibReg[9];
			dir_time[2] = '_';
			dir_time[3] = HibReg[10];
			dir_time[4] = HibReg[11];
			dir_time[5] = '\0';
			//
			//
			//
			hibPeriod = HibReg[12];
			//
			// Implementacion del datalogger
			//
			//
			GetNewLine(line);
			//
			// Append data
			//
			if( f_mount(0,&FatFs) == FR_OK )
			{
				// Date path
				if (f_stat(dir_date, &fno) == FR_OK)
				{
					f_chdir(dir_date);
					// Time path
					if (f_stat(dir_time, &fno) == FR_OK)
					{
						f_chdir(dir_time);
						// Open te file
						if(f_open(&fil, "Data.csv", FA_READ | FA_WRITE | FA_OPEN_ALWAYS) == FR_OK)
						{
							// Check size
							//filesize=f_size(&fil);
							// Search for the end of the file
							if(f_lseek(&fil, f_size(&fil)) == FR_OK)
							{
								// Begin flag
								GPIOPinWrite(LED_PORT,LED_1, LED_1);
								// Write line and new line
								fr = f_write(&fil, line, strlen(line), &br);
								fr = f_write(&fil, newline, strlen(newline), &br);
								//end of f_seek
							}
							//end of f_open
							f_close(&fil);
						}
					}
				}
				//end of f_mount
				f_mount(0, NULL);
			}
			// End flag
			GPIOPinWrite(LED_PORT,LED_1, 0);
			// Save values
			HibernateDataSet(HibReg,13);
			//
		}
		// Waking because of SW_ON
		if (ulStatus & HIBERNATE_INT_PIN_WAKE){
			HibernateIntClear(HIBERNATE_INT_PIN_WAKE | HIBERNATE_INT_LOW_BAT | HIBERNATE_INT_RTC_MATCH_0);
			GPIOPinWrite(LED_PORT,LED_ON, LED_ON);//Red
			delayMS(500);
			GPIOPinWrite(LED_PORT,LED_ON, 0);
			HibReg[1]=0;
			//
			DataLoggingOFF();
			//

		}
		//En caso contrario, es un reset o power-on
	}
	//
	//
	//
	while(1)
	{
		//
		//readHIH(&myHR, &myTbs);
		// Wait for hibernation module
		if(HibReg[1] == 1)
		{
			// Save path for hibernation
			HibReg[2] = dir_date[0];
			HibReg[3] = dir_date[1];
			HibReg[4] = dir_date[3];
			HibReg[5] = dir_date[4];
			HibReg[6] = dir_date[6];
			HibReg[7] = dir_date[7];
			HibReg[8] = dir_time[0];
			HibReg[9] = dir_time[1];
			HibReg[10] = dir_time[3];
			HibReg[11] = dir_time[4];
			// Empiezo a hibernar y tomar datos
			DataLoggingON();
		}
		//End datalogging configuration

		//
		// Enable USB
		//
		if(USBDataloggingFlag == 1)
		{
			// Configure and initialize USB
			USBSerialInit();
			// Flag to process USB Rx data
			USBDataloggingFlag = 2;
			//
			// Initialize OneWire module
			InitOneWire(1);
			// Get OneWire ROM addresses
			OneWiregetROMs();
			delayMS(5);
		}
		//End enable USB

		//
		// Receive USB Data
		//
		if((USBDataloggingFlag == 2) && (USBProcessDataFlag == 1))
		{
			// Set flag to down, process Rx data one time
			USBProcessDataFlag = 0;
			switch (RxDataFrame[1])
			{
			case 0x33:
				// Configure clock
				SetTimeDate(RxDataFrame[7], RxDataFrame[6], RxDataFrame[5],1,RxDataFrame[4],RxDataFrame[3],RxDataFrame[2]);
				//
				break;
				// end case 0x33
			case 0x34:
				// Get period from USB variable
				for(j=0;j<=3;j++)
				{
					PeriodBytes[j] = RxDataFrame[j+2];
				}
				hibPeriod = PeriodBytes[0] << 24;
				hibPeriod += PeriodBytes[1] << 16;
				hibPeriod += PeriodBytes[2] << 8;
				hibPeriod += PeriodBytes[3];
				HibReg[12] = hibPeriod;
				//EEPROMProgram(hibPeriod, 0x0, sizeof(hibPeriod));
				flashPeriod[0] = hibPeriod;
				flashPeriod[1] = 0;
				FlashErase(0x39000);
				FlashProgram(flashPeriod, 0x39000, sizeof(flashPeriod));
				// TODO write period in FLASH
				// end case 0x34
				//pui32Test = hibPeriod;
				break;
				// end case 0x34
			case 0x35:
				//USBStartLoggingFlag = 1;
				USBSendLine();
				break;
				// end case 0x35
			}
			// end switch

		}
		// End Rx data
	}
	//End while loop
}

/*********************************************************************************************
 * GPIOPortC_IRQHandler
 * ******************************************************************************************/
void GPIOPortC_IRQHandler(void){
	//Check interrupt source
	GPIOIntState = GPIOIntStatus(SW_PORT,true);
	//
	if((GPIOIntState & SW_1) == SW_1){
		// IF USB NOT CONNECTED
		//if(!isCDCConnected()){
		if(!USBDataloggingFlag){
			//Flag de que estoy en la interrupcion por GPIO
			GPIOPinWrite(LED_PORT,LED_2, LED_2);//
			delayMS(500);
			GPIOPinWrite(LED_PORT,LED_2, 0);
			//
			// Nuevo archivo
			//
			WriteFirstLine(&dir_date, &dir_time);
			//
			// Flag
			HibReg[1]=1;
			//end if
		}
		//end if
	}
	if((GPIOIntState & SW_SD) == SW_SD){
		// Check if USB is active
		if(USBDataloggingFlag){
			//
			GPIOPinWrite(LED_PORT,LED_1, LED_1);
			delayMS(200);
			GPIOPinWrite(LED_PORT,LED_1, 0);
			delayMS(100);
			GPIOPinWrite(LED_PORT,LED_1, LED_1);
			delayMS(200);
			//
			USBDataloggingFlag = 0;
			//Disable USB
			USBSerialKaput();
			//
			GPIOPinWrite(LED_PORT,LED_1, 0);
		}
		else if (!USBDataloggingFlag){
			//
			GPIOPinWrite(LED_PORT,LED_1, LED_1);//
			delayMS(500);
			// Flag to configure and initialize USB
			USBDataloggingFlag = 1;
			//
			GPIOPinWrite(LED_PORT,LED_1, 0);
		}
	}
	// Clear interrupt source
	GPIOIntClear(SW_PORT, GPIOIntState);
}
