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

/*
 * From the mbot.h mbot::mbot() constructor and MACRO defs.
 * M1 == 9,  motorPortLeft  == 9
 * M2 == 10, motorPortRight == 10
 * So the mbot looks like this:
 * 
 *
 *              +--------------------+                    L Motor       R Motor
 *  A           |                    |        C           <------       ----->
 *  n           |                    |        l           |     ^       ^    |
 *  t           |                    |        o           |     |       |    |
 *  i           |                    |        c           v----->       <----v
 *              |                    |        k
 *  C           | +---+        +---+ |        w
 *  l           | | L |        | R | |        i
 *  o     left  | | M |        | M | | right  s
 *  c     ========| o |        | o |========  e
 *  k     shaft | | t |        | t | | shaft
 *  w           | | o |        | o | |
 *  i           | | r |        | r | |
 *  s           | +---+        +---+ |
 *  e           |                    |
 *              +--------------------+
 *
 * If both motors are turning the drive shafts clockwise, the robot would
 * spin in an anti-clockwise circle. They must spin in opposite directions.
 *
 * So, one motor spins in one direction, while the other spins in the opposite
 * direction.
 */
void motor::motor_run(int leftSpeed, int rightSpeed)
{
	// Spin left shaft opposite of right shaft
	// M1 is the left motor port
	// M2 is the right motor port
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
