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
	return front->distanceCm();
}

double ultrasonic::leftDistanceCM()
{
	return left->distanceCm();
}

double ultrasonic::rightDistanceCM()
{
	return right->distanceCm();
}
#endif // __ULTRASONIC_H_
