#if !defined(__ULTRASONIC_H_)
#define __ULTRASONIC_H_

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <MeMCore.h>

class ultrasonic {
public:
	ultrasonic(int front, int left, int right);
	double frontDistanceCM();
	double leftDistanceCM();
	double rightDistanceCM();
	MeUltrasonicSensor *getFrontP();
	MeUltrasonicSensor *getRightP();
	MeUltrasonicSensor *getLeftP();
private:
	MeUltrasonicSensor *front;
	MeUltrasonicSensor *left;
	MeUltrasonicSensor *right;
};

ultrasonic::ultrasonic(int f, int l, int r)
{
	front = new MeUltrasonicSensor(f);
	left = new MeUltrasonicSensor(l);
	right = new MeUltrasonicSensor(r);
}

MeUltrasonicSensor * ultrasonic::getFrontP()
{
	return front;
}

MeUltrasonicSensor * ultrasonic::getLeftP()
{
	return left;
}

MeUltrasonicSensor * ultrasonic::getRightP()
{
	return right;
}

double ultrasonic::frontDistanceCM()
{
	double d = front->distanceCm();
	if (d < 2.0)
		d = 2.0;
	if (d > 400.0)
		d = 400.0; 
	return d;
}

double ultrasonic::leftDistanceCM()
{
	double d =  left->distanceCm();
	if (d < 2.0)
		d = 2.0;
	if (d > 400.0)
		d = 400.0; 
	return d;
}

double ultrasonic::rightDistanceCM()
{
	double d = right->distanceCm();
	if (d < 2.0)
		d = 2.0;
	if (d > 400.0)
		d = 400.0; 
	return d;
}
#endif // __ULTRASONIC_H_
