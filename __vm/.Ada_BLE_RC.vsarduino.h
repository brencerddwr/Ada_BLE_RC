/* 
	Editor: http://www.visualmicro.com
			visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
			the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
			all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
			note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Adafruit Feather 32u4, Platform=avr, Package=adafruit
*/

#define __AVR_ATmega32u4__
#define __AVR_ATmega32U4__
#define ARDUINO 10802
#define ARDUINO_MAIN
#define F_CPU 8000000L
#define __AVR__
#define F_CPU 8000000L
#define ARDUINO 10802
#define ARDUINO_AVR_FEATHER32U4
#define ARDUINO_ARCH_AVR
#define USB_VID 0x239A
#define USB_PID 0x800C
void error(const __FlashStringHelper*err);
void setup(void);
void loop(void);
bool accelMode();
bool buttonMode();
void BLEsetup();
float fscale( float inputValue,  float originalMin, float originalMax, float newBegin, float newEnd, float curve);

#include "pins_arduino.h" 
#include "arduino.h"
#include "Ada_BLE_RC.ino"
