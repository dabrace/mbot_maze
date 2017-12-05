#if !defined(__LED_H)
#define __LED_H

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMCore.h>

class led {
public:
	led(int rj25Port);
	void setColor(int r, int g, int b, int v);
	void debugUsingColor(int port, double c, double v);
private:
	MeRGBLed *rgbled;
};

led::led(int rj25Port)
{
	rgbled = new MeRGBLed(4, 4==7?2:4);
}

void led::setColor(int w, int r, int g, int b)
{
        rgbled->setColor(w,r,g,b);
        rgbled->show();
}

void led::debugUsingColor(int port, double c, double v)
{
	if (c < v)
		setColor(3, 150, 0, 0); // Red
	else if (c > v)
		setColor(3, 0, 150, 0); // Green
	else
		setColor(3, 0, 0, 150); // Blue
}

#endif
