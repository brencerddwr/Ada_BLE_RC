/*********************************************************************
This is an example for our nRF51822 based Bluefruit LE modules

Modified to drive a 3-wheeled BLE Robot Rover! by http://james.devi.to

Pick one up today in the Adafruit shop!

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in
any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEEddystone.h"

#include "BluefruitConfig.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_MS_PWMServoDriver.h"
//#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

//not used, testing acceleration
int accelTime = 25;

//Name your RC here
String BROADCAST_NAME = "Richard's robot rover";

String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"
#define URL                         "https://goo.gl/HV6rBR"


Adafruit_BLEEddystone eddyBeacon(ble);

// Set your forward, reverse, and turning speeds
#define ForwardSpeed                255
#define ReverseSpeed                255
#define TurningSpeed                75
int speed = 0;
int lTrim = 7;
int rTrim = 0;
// A small helper
void error(const __FlashStringHelper*err) {
	Serial.println(err);
	while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

char buf[60];



/**************************************************************************/
/*!
@brief  Sets up the HW an the BLE module (this function is called
automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
	Serial.begin(9600);

	AFMS.begin();  // create with the default frequency 1.6KHz
	// turn on motors
	L_MOTOR->setSpeed(0);
	L_MOTOR->run(RELEASE);

	R_MOTOR->setSpeed(0);
	R_MOTOR->run(RELEASE);

	Serial.begin(115200);
	Serial.println(F("Adafruit Bluefruit Robot Controller Example"));
	Serial.println(F("-----------------------------------------"));

	/* Initialize the module */
	BLEsetup();


}

int velocity = 0;
float x, y;
int L_restrict = 0;
int R_restrict = 0;
unsigned long lastAccelPacket = 0;
bool modeToggle = false;

void loop(void)
{
	// read new packet data
	uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
	//	 if (len == 0) return;

	// Read from Accelerometer input
	if( accelMode() ) {
		lastAccelPacket = millis();
		modeToggle = true;
		return;
	}

	// Stop motors if accelerometer data is turned off (100ms timeout)
	if( millis() - (lastAccelPacket > 100) & modeToggle) {
		L_MOTOR->run(RELEASE);
		R_MOTOR->run(RELEASE);
		modeToggle = false;
		return;
	}

	//if no accelerometer, use control pad
	if( !modeToggle ) buttonMode();

}


bool accelMode(){
	if (packetbuffer[1] == 'A') {
		x = parsefloat( packetbuffer + 2 );
		y = parsefloat( packetbuffer + 6 );

		if( x <= -0.55 ){
			x += 0.55;
			x *= -100.0;
			L_MOTOR->run( BACKWARD );
			R_MOTOR->run( BACKWARD );
			if( x >= 45 ) x = 45;
			if( x <= 0 ) x = 0;
			velocity = map( x, 0, 45, 0 ,255 );
		}
		else if( x >= -0.25 ){
			x+= 0.25;
			x *= 100;
			L_MOTOR->run( FORWARD );
			R_MOTOR->run( FORWARD );
			if( x>= 45 ) x = 45;
			if( x<= 0 ) x = 0;
			velocity = map( x, 0, 45, 0, 255 );
		}
		else{
			L_MOTOR->run( RELEASE );
			R_MOTOR->run( RELEASE );
			velocity = 0;
		}

		//account for L / R accel

		if( y >= 0.1 ){
			y -= 0.1;
			y *= 100;
			if( y >= 50 ) y = 50;
			if( y <= 0 ) y = 0;

			L_restrict = fscale( y, 0.0, 50.0, 0.0, 100.0, -4.0 );
		}
		else if( y <= -0.1 ){
			y += 0.1;
			y *= -100;
			if( y>= 50 ) y = 50;
			if( y<= 0 ) y = 0;

			R_restrict = fscale( y, 0.0, 50.0, 0.0, 100.0, -4.0 );
		}
		else{
			L_restrict = 0;
			R_restrict = 0;
		}

		float Lpercent = ( 100.0 - L_restrict ) / 100.00 ;
		float Rpercent = ( 100.0 - R_restrict ) / 100.00 ;

		// Serial.print( x );
		// Serial.print( "\t" );
		// Serial.print( Lpercent );
		// Serial.print( "\t" );
		// Serial.print( velocity );
		// Serial.print( "\t" );
		// Serial.println( Rpercent );

		L_MOTOR->setSpeed( velocity * Lpercent );
		R_MOTOR->setSpeed( velocity * Rpercent );

		return true;
	}
	return false;
}

bool isMoving = false;

bool buttonMode(){

	//	static unsigned long lastPress = 0;
	uint8_t maxspeed;


	// Buttons
	if (packetbuffer[1] == 'B') {

		uint8_t buttnum = packetbuffer[2] - '0';
		boolean pressed = packetbuffer[3] - '0';

		// Serial.println(buttnum);

		Serial.println(isMoving);
		if (pressed) {
			if(buttnum == 1){
				isMoving = true;
				L_MOTOR->run(RELEASE);
				R_MOTOR->run(FORWARD);
				maxspeed = TurningSpeed;

			}

			if(buttnum == 2){
				isMoving = true;
				L_MOTOR->run(FORWARD);
				R_MOTOR->run(RELEASE);
				maxspeed = TurningSpeed;


			}

			if(buttnum == 3){

			}

			if(buttnum == 4){

				eddyBeacon.startConfigMode(300);
			}

			isMoving = true;
			if(buttnum == 5){
				isMoving = true;
				L_MOTOR->run(FORWARD);
				R_MOTOR->run(FORWARD);
				maxspeed = ForwardSpeed;
			}
			if(buttnum == 6){
				isMoving = true;
				L_MOTOR->run(BACKWARD);
				R_MOTOR->run(BACKWARD);
				maxspeed = ReverseSpeed;
			}
			if(buttnum == 7){
				isMoving = true;
				L_MOTOR->run(BACKWARD);
				R_MOTOR->run(FORWARD);
				maxspeed = TurningSpeed;
			}
			if(buttnum == 8){
				isMoving = true;
				L_MOTOR->run(FORWARD);
				R_MOTOR->run(BACKWARD);
				maxspeed = TurningSpeed;
			}

			// speed up the motors
			for (speed=0; speed < maxspeed; speed+=5)
			{
				L_MOTOR->setSpeed(speed-lTrim);
				R_MOTOR->setSpeed(speed-rTrim);
				delay(accelTime/5);
			}


		}

		else
		{
			isMoving = false;

			for (speed ; speed >= 0; speed-=5)
			{
				L_MOTOR->setSpeed(speed-lTrim);
				R_MOTOR->setSpeed(speed-rTrim);
				delay(accelTime/6);
			}

			L_MOTOR->run(RELEASE);
			R_MOTOR->run(RELEASE);
		}
		return true;
	}
	return false;

}

void BLEsetup(){
	Serial.print(F("Initialising the Bluefruit LE module: "));

	if ( !ble.begin(VERBOSE_MODE) )
	{
		error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
	}
	Serial.println( F("OK!") );

	/* Perform a factory reset to make sure everything is in a known state */
	//	Serial.println(F("Performing a factory reset: "));
	// 	if (! ble.factoryReset() ){
	// 		error(F("Couldn't factory reset"));
	// 	}

	//Convert the name change command to a char array
	BROADCAST_CMD.toCharArray(buf, 60);

	//Change the broadcast device name here!
	if(ble.sendCommandCheckOK(buf)){
		Serial.println("name changed");
	}
	delay(250);

	//reset to take effect
	if(ble.sendCommandCheckOK("ATZ")){
		Serial.println("resetting");
	}
	delay(250);

	//Confirm name change
	ble.sendCommandCheckOK("AT+GAPDEVNAME");

	/* Disable command echo from Bluefruit */
	ble.echo(false);

	Serial.println("Requesting Bluefruit info:");
	/* Print Bluefruit information */
	ble.info();

	Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
	Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
	Serial.println();

	ble.verbose(false);  // debug info is a little annoying after this point!
	eddyBeacon.begin(true);
	if ( !eddyBeacon.setURL(URL) ) {
		error(F("Couldnt set, is URL too long !?"));
	}
	//	Serial.print (F("Beacon URL: "));

	/* Wait for connection */
	while (! ble.isConnected()) {
		delay(500);
	}

	Serial.println(F("*****************"));

	// Set Bluefruit to DATA mode
	Serial.println( F("Switching to DATA mode!") );
	ble.setMode(BLUEFRUIT_MODE_DATA);

	Serial.println(F("*****************"));
}

//Logarithmic mapping function from http://playground.arduino.cc/Main/Fscale
float fscale( float inputValue,  float originalMin, float originalMax, float newBegin, float newEnd, float curve){

	float OriginalRange = 0;
	float NewRange = 0;
	float zeroRefCurVal = 0;
	float normalizedCurVal = 0;
	float rangedValue = 0;
	boolean invFlag = 0;


	// condition curve parameter
	// limit range

	if (curve > 10) curve = 10;
	if (curve < -10) curve = -10;

	curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
	curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

	/*
	Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution
	Serial.println();
	*/

	// Check for out of range inputValues
	if (inputValue < originalMin) {
		inputValue = originalMin;
	}
	if (inputValue > originalMax) {
		inputValue = originalMax;
	}

	// Zero Refference the values
	OriginalRange = originalMax - originalMin;

	if (newEnd > newBegin){
		NewRange = newEnd - newBegin;
	}
	else
	{
		NewRange = newBegin - newEnd;
		invFlag = 1;
	}

	zeroRefCurVal = inputValue - originalMin;
	normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

	/*
	Serial.print(OriginalRange, DEC);
	Serial.print("   ");
	Serial.print(NewRange, DEC);
	Serial.print("   ");
	Serial.println(zeroRefCurVal, DEC);
	Serial.println();
	*/

	// Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
	if (originalMin > originalMax ) {
		return 0;
	}

	if (invFlag == 0){
		rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

	}
	else     // invert the ranges
	{
		rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
	}

	return rangedValue;
}

