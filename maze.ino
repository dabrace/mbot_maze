#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeMCore.h"

#include "mbot.h"

mbot robot;

void _loop()
{
}

void _delay(float seconds)
{
	long endTime = millis() + seconds * 1000;
	while(millis() < endTime)_loop();
}

void setup()
{
	// put your setup code here, to run once:
	// Wait for on-board button to be pushed.
	pinMode (A7,INPUT);
	while (!(0^(analogRead(A7)>10?0:1))) {
		_delay(1.0);
	}

	// Start the robot moving forward.
	// Use default speed
	robot.move(1, -1);

	Serial.begin(9600);
}

void loop()
{
	// put your main code here, to run repeatedly:
	robot.moveAlongWall();
	//_delay(0.5);
}
