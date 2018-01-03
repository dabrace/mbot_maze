#if !defined(__MOTOR_H)
#define __MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <MeMCore.h>

class motor {
public:
	motor(int port_left, int port_right);
	void motor_run(int left_speed, int right_speed);
	MeDCMotor *left;
	MeDCMotor *right;
private:
	int motorPortLeft;
	int motorPortRight;
};


motor::motor(int port_left, int port_right)
{
	left = new MeDCMotor(port_left);
	right = new MeDCMotor(port_right);
	
	motorPortLeft = port_left;
	motorPortRight = port_right;
}

void motor::motor_run(int leftSpeed, int rightSpeed)
{
	left->run((motorPortLeft)==M1?-(leftSpeed):(leftSpeed));
	right->run((motorPortRight)==M1?-(rightSpeed):(rightSpeed));
	Serial.print("motor_run: ");
	Serial.print("motorPortLeft: ");
	Serial.print(motorPortLeft);
	Serial.print(" motorPortRight: ");
	Serial.print(motorPortRight);
	Serial.print(" M1: ");
	Serial.print(M1);
	Serial.print(" M2: ");
	Serial.print(M2);
	Serial.print(" RWS: ");
	Serial.print(rightSpeed);
	Serial.print(" LWS: ");
	Serial.print(leftSpeed);
	Serial.println(".");
}

#endif // __MOTOR_H
