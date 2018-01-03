#if !defined(__MBOT_H_)
#define __MBOT_H_

#include "MeBluetooth.h"
#include "motor.h"
#include "ultrasonic.h"
#include "led.h"

/*
DAB
	Here's an alternative way to control the motors when following a wall:

	1. Define the ideal follow distance D and the typical motor speed S.
	2. Read your sensor distance d, maybe using a filter to avoid noise. 
	3. Calculate the error e = D - d if the wall is on the right, or
		e = d - D if wall is left.
	4. Set the motor speeds like this:
    		MotorLeft.setSpeed ( S - k * e )
    		MotorRight.setSpeed ( S + k * e )
    			Where k  is a gain coefficient that you can tweak
			to get the behavior you like.

	Notes for sending serial data over bluetooth:
		if ( Serial.available()) {
			String serialResponse = Serial.readStringUntil(’\r\n’);
			Serial.println("Read: " + serialResponse);
		}

*/

#define WALLDISTANCE 8.58 // 1" = 2.54 CM // Not sure if it is really in CM.
#define MOTORSPEED 80.0
#define SPEED_DELTA_MAX 15.0
#define TURNDELAY 0.3  // Arbitrary guess based on max speed.
#define K 3.0
#define TURN_TOLERANCE 2.0

// Need to look at mbot to see what RJ25 ports the sensors are plugged into
#define FRONT_US_SENSOR_PORT 3
#define LEFT_US_SENSOR_PORT 1
#define RIGHT_US_SENSOR_PORT 2

#define LEFTMOTORPORT 9
#define RIGHTMOTORPORT 10

#define LEDPORT 4

#define FRONTWALL 100
#define RIGHTWALL 200
#define LEFTWALL 300

#define NOWALL 10

class mbot {
public:
	mbot();
	void move(int direction, int speed);
	int moveAlongWall();
private:
	MeBluetooth Bluetooth;
	double angle_rad = PI/180.0;
	double angle_deg = 180.0/PI;
	motor *mbotMotor;
	ultrasonic *mbotUltrasonic;
	led *mbotLed;
	double speed;
	double leftWheelSpeed;
	double rightWheelSpeed;
	// Distance measurements
	double leftDistance;
	double prevLeftDistance;
	double prevLeftDerivitive;
	double rightDistance;
	double prevRightDistance;
	double prevRightDerivitive;
	double frontDistance;
	double prevFrontDistance;
	float normalizeDelta(int delta);
	void do_180();
	void do_turn();
	void _delay(float seconds);
	void _loop();
	double getDistance(int dir);
	void takeDistanceMeasurements();
	void storeMearurements();
	void findWall();
	void followWall();
	int currentWall;
	int wall;
};

mbot::mbot()
{
	mbotMotor = new motor(LEFTMOTORPORT, RIGHTMOTORPORT);
	mbotUltrasonic = new ultrasonic(FRONT_US_SENSOR_PORT,
					LEFT_US_SENSOR_PORT,
					RIGHT_US_SENSOR_PORT);
	mbotLed = new led(LEDPORT);

	speed = MOTORSPEED;
	leftWheelSpeed = MOTORSPEED;
	rightWheelSpeed = MOTORSPEED;

	//findWall();
	takeDistanceMeasurements();
	storeMearurements();
	prevRightDerivitive = 0;
	prevLeftDerivitive = 0;

	Serial.begin(230400);
	Serial.println("CONSTRUCTOR");

        //bluetooth.begin(115200);    //The factory default baud rate is 115200
        Bluetooth.begin(230400);    //The factory default baud rate is 115200

}

void mbot::_loop()
{
}

void mbot::_delay(float seconds)
{
	long endTime = millis() + seconds * 1000;
	while(millis() < endTime)_loop();
}

/*
 * Take a few samples
 */
#define NUM_READINGS 10
double mbot::getDistance(int dir)
{
	double distance[NUM_READINGS] = { 0.0 };
	double total = 0.0;
	double tolerance = 0.0;
	int count = 0;
	int i;

	for (i = 0; i < NUM_READINGS; i++)
		distance[i] = 0.0;

	tolerance = TURN_TOLERANCE * 2;
	// Try to throw away any outliars
	for (i = 0; i < NUM_READINGS; i++) {
		switch(dir) {
		case RIGHTWALL:
			distance[i] = mbotUltrasonic->rightDistanceCM();
			break;
		case LEFTWALL:
			distance[i] = mbotUltrasonic->leftDistanceCM();
			break;
		case FRONTWALL:
			tolerance = 1;
			distance[i] = mbotUltrasonic->frontDistanceCM();
			break;
		} // switch
	} // for

	for (i = 1; i < NUM_READINGS; i++) {
		if (((abs(distance[i] - distance[i-1])) > tolerance) ||
			(distance[i] <= 0))
			continue;
		total += distance[i];
		++count;
	}

	//Serial.print("Distance: ");
	//Serial.println(total/count);

	return total/count;
} // getDistance

void mbot::takeDistanceMeasurements()
{
	//frontDistance = mbotUltrasonic->frontDistanceCM();
	//leftDistance = mbotUltrasonic->leftDistanceCM();
	//rightDistance = mbotUltrasonic->rightDistanceCM();
	frontDistance = getDistance(FRONTWALL);
	rightDistance = getDistance(RIGHTWALL);
	leftDistance = getDistance(LEFTWALL);
}

void mbot::storeMearurements()
{
	prevLeftDistance = leftDistance;
	prevRightDistance = rightDistance;
	prevFrontDistance = frontDistance;
	currentWall = wall;
}

/*
 * find a wall nearest to the mbot.
 *
 * Takes all distance measurements
 * Need to allow for robot drifting.
 *
 * LED Sensors
 *      1F
 *   4R    2L
 *      3
 */
void mbot::findWall()
{
	/*
	 * Will need to turn to keep following a wall.
	 */
	wall = NOWALL;

	frontDistance = getDistance(FRONTWALL);
	rightDistance = getDistance(RIGHTWALL);
	leftDistance = getDistance(LEFTWALL);

	if (frontDistance <= WALLDISTANCE) {
		mbotLed->setColor(1, 200, 0, 0);
		mbotLed->setColor(1, 0, 0, 0);
		wall = FRONTWALL;
		Serial.print("findWall FRONT: ");
		Serial.println(frontDistance);
		return;
	} else if (rightDistance <= (WALLDISTANCE + 2*K)) {
		mbotLed->setColor(4, 200, 0, 0);
		mbotLed->setColor(4, 0, 0, 0);
		wall = RIGHTWALL;
		Serial.print("findWall RIGHT: ");
		Serial.println(rightDistance);
		return;
	} else if (leftDistance <= (WALLDISTANCE + 2*K)) {
		mbotLed->setColor(2, 200, 0, 0);
		mbotLed->setColor(2, 0, 0, 0);
		wall = LEFTWALL;
		Serial.print("findWall LEFT: ");
		Serial.println(leftDistance);
		return;
	} else {
		wall = NOWALL;
		Serial.print("findWall NOWALL: ");
		Serial.println(0);
		return;
	}
} //findWall

// Avoid large deltas
float mbot::normalizeDelta(int delta)
{
	int r = abs(delta);

	if (r > SPEED_DELTA_MAX)
		r = SPEED_DELTA_MAX;
		if (delta < 0)
			r = -r;
	else
		r = delta;

	return r;
}

void mbot::move(int direction, int speed)
{
	int leftSpeed = 0;
	int rightSpeed = 0;
	
	if (speed == -1)
		speed = MOTORSPEED;

	if (direction == 1) {		// Forward
		leftSpeed = speed;
		rightSpeed = speed;
	} else if (direction == 2) {	// Reverse
		leftSpeed = -speed;
		rightSpeed = -speed;
	} else if (direction == 3) {	// Left
		leftSpeed = -speed;
		rightSpeed = speed;
	} else if (direction == 4) {	// Right
		leftSpeed = speed;
		rightSpeed = -speed;
	}

	mbotMotor->motor_run(leftSpeed, rightSpeed);
}

void mbot::do_180()
{
	MeUltrasonicSensor *mbot_ultrasonic;

	switch (wall) {
	/*
	 * Wall is on the left, do 180 right
	 */
	case LEFTWALL:
		mbot_ultrasonic = mbotUltrasonic->getRightP();
		rightWheelSpeed = -speed;
		leftWheelSpeed = speed;
		wall = RIGHTWALL;
		Serial.println("do_180: LEFT");
		break;
	/*
	 * Wall is on the right, do 180 left
	 */
	case RIGHTWALL:
		mbot_ultrasonic = mbotUltrasonic->getLeftP();
		rightWheelSpeed = speed;
		leftWheelSpeed = -speed;
		wall = LEFTWALL;
		Serial.println("do_180: RIGHT");
		break;
	/*
	 * Something is hosed
	 */
	default:
		Serial.println("do_180: NOWALL");
		rightWheelSpeed = 0;
		leftWheelSpeed = 0;
		wall = NOWALL;
		break;
	}

	mbotLed->setColor(1, 0, 0, 200);
	mbotLed->setColor(3, 0, 0, 200);

	mbotMotor->motor_run(leftWheelSpeed, rightWheelSpeed);
	// Keep at it until we are at a set distance from the wall.
	while (mbot_ultrasonic->distanceCm() > (WALLDISTANCE));

	mbotLed->setColor(1, 0, 0, 0);
	mbotLed->setColor(3, 0, 0, 0);

	rightWheelSpeed = speed;
	leftWheelSpeed = speed;
	mbotMotor->motor_run(leftWheelSpeed, rightWheelSpeed);

	return;
} // do_180

/*
 *========================================================================
 * Turn Left or Right
 *
 * Algorithm:
 *            1. Ultrasonic sensor detects missing wall.
 *            2. add in short delay to allow mbot wheels to clear wall
 *            3. Stop inside wheel.
 *            4. Keep outside wheel running.
 *            5. When ultrasonic measures correct distance to wall
 *               start up other wheel.
 *
 *    |                             | Wall
 *    |                             +------------------------------
 *    |                                 +-------+ +-----+
 *    |                                 |tire   | | LS  |
 *    |                                 +-------+ +-----+
 *    |                                 +------------+
 *    |                                 |            |+-+
 *    |                                 |     L      ||F|
 *    |                                 |  B=====>F  ||S|
 *    |                                 |     R      |+-+
 *    |                                 +------------+
 *    |           +-----+               +-------+ +-----+
 *    |   +-+     |  S  |     +-+       |tire   | | RS  |
 *    |   |S| +-------------+ |S|       +-------+ +-----+
 *    |   +-+ |             | +-+ 
 *    |       |             |       +------------------------------
 *    |   +-+ |      F      | +-+   |
 *  W |   |t| |      ^      | |t|   | W
 *  a |   |i| |     L|R     | |i|   | a
 *  l |   |r| |      v      | |r|   | l
 *  l |   |e| |      B      | |e|   | l
 *    |   +-+ |             | +-+   |
 *    |       |             |       |
 *    |       +-------------+       |
 *    |                             |
 *    |                             |
 *
 *========================================================================
 */
void mbot::do_turn()
{
	MeUltrasonicSensor *mbot_ultrasonic;

	mbotLed->setColor(4, 0, 200, 0);
	mbotLed->setColor(2, 0, 200, 0);

	/*
	 * Do not know how far the mbot has traveled
	 * so, delay a while to get enough of the mbot
	 * past the corner to turn.
	 */
	_delay(TURNDELAY);

	mbotLed->setColor(4, 0, 0, 0);
	mbotLed->setColor(2, 0, 0, 0);

	switch (currentWall) {
	/*
	 * Wall was on the left, turn left
	 */
	case LEFTWALL:
		mbot_ultrasonic = mbotUltrasonic->getLeftP();
		rightWheelSpeed = speed;
		leftWheelSpeed = speed/4;
		wall = LEFTWALL;
		mbotLed->setColor(4, 0, 0, 0);
		mbotLed->setColor(2, 200, 0, 0);
		break;
	/*
	 * Wall was on the right, turn right
	 */
	case RIGHTWALL:
		mbot_ultrasonic = mbotUltrasonic->getRightP();
		rightWheelSpeed = speed/4;
		leftWheelSpeed = speed;
		wall = RIGHTWALL;
		mbotLed->setColor(4, 200, 0, 0);
		mbotLed->setColor(2, 0, 0, 0);
		break;
	default:
		rightWheelSpeed = 0;
		leftWheelSpeed = 0;
		wall = NOWALL;
		mbotLed->setColor(4, 0, 200, 0);
		mbotLed->setColor(2, 0, 200, 0);
		break;
	}

	/*
	 * The idea is to keep turning until the sensor reads the proper
	 * distance from the wall.
	 */
	mbotLed->setColor(1, 0, 200, 0);
	mbotLed->setColor(3, 0, 200, 0);
	mbotMotor->motor_run(leftWheelSpeed, rightWheelSpeed);
	// Keep at it until we are at a set distance from the wall.
	while (mbot_ultrasonic->distanceCm() > WALLDISTANCE);
	mbotLed->setColor(1, 0, 0, 0);
	mbotLed->setColor(3, 0, 0, 0);

	/*
	 * Resume forward progress.
	 */
	rightWheelSpeed = speed;
	leftWheelSpeed = speed;

	mbotLed->setColor(4, 0, 0, 0);
	mbotLed->setColor(2, 0, 0, 0);

	return;
} // do_turn

/*
 *========================================================================
 * Follow wall
 *
 * Algorithm:
 *            1. Measure distance to wall using right Ultrasonic sensor
 *            2. Calculate the difference between distance and tolerance
 *            3. Adjust left and right speed based on the delta value.
 *            4. Limit the speed hike to keep the robot from spinning in
 *               circles.
 *
 *    |           +-----+           |
 *    |   +-+     |  S  |     +-+   |  
 *    |   |S| +-------------+ |S|<->| WALLDISTANCE 
 *    |   +-+ |             | +-+   |  
 *    |       |             |       |  
 *    |   +-+ |      m      | +-+   |
 *  W |   |t| |      B      | |t|   | W
 *  a |   |i| |      o      | |i|   | a
 *  l |   |r| |      t      | |r|   | l
 *  l |   |e| |             | |e|   | l
 *    |   +-+ |             | +-+   |
 *    |       |             |       |
 *    |       +-------------+       |
 *    |                             |
 *    |                             |
 *
 *========================================================================
 */

// Calculate the delta based on the buffer zone and current distance
// and multiply times a constant value K to smooth out the jitter.
// Delta will be negative if the distance is larger than ideal dist.
// Therefore a distance > WALLDISTANCE will:
//      1. slow down the right wheel speed and
// 	2. speed up the left wheel speed.
// Avoid too high of a speed delta.
// Helps prevent spinnig in circles when distances are large
void mbot::followWall()
{
	double distance = 0.0;
	double delta = 0.0;
	double derivitive = 0.0;

	/*
	 * Assign right and left wheel speeds.
	 * Recalculate upon each entry into the routine.
	 *
	 * If delta is positive, then we are too close to the wall.
	 * If delta is negative, then we are too far away from the wall.
	 *
	 * Use the first derivitive to determine if we have not corrected enough.
	 *
	 *      +--------------------+ case too far from the wall
	 *     W|                    |
	 *     a|      +---+         |
	 *     l|      |   |<------->| distance
	 *     l|      +---+<->|<--->| WALLDISTANCE
	 *      |            d |     |
	 *      |            e |     | W
	 *      |            l |     | a
	 *      |            t |     | l
	 *      |            a |     | l
	 *
	 *      +--------------------+ case too close to the wall
	 *     W|                    |
	 *     a|            +---+   |
	 *     l|            |   |<->| distance
	 *     l|            +---+   |
	 *      |              |<>   | delta
	 *      |              |<--->| WALLDISTANCE
	 *      |              |     |
	 *      |              |     | W
	 *      |              |     | a
	 *      |              |     | l
	 *      |              |     | l
	 * RWS = Right Wheel Speed
	 * LWS = Left Wheel Speed
	 */
	switch (wall) {
		case FRONTWALL:
			break;
		case LEFTWALL: // Follow along left wall
			mbotLed->setColor(2, 0, 200, 0);
			distance = getDistance(LEFTWALL);
			derivitive = distance - prevLeftDistance;
			delta = WALLDISTANCE - distance;
			if (delta < 0) { // delta is negative, too far from the wall
				if (derivitive < prevLeftDerivitive) // Still heading towards wall
					delta = delta * K; // Decrease RT, Increase LT
				rightWheelSpeed = speed - delta; // delta is negative, RWS less
				leftWheelSpeed = speed + delta;  // delta is negative, LWS more
			} else if (delta > 0) { // delta is positive, too close to wall
				if (derivitive > prevLeftDerivitive) // Still heading away from wall
					delta = delta * K;
				rightWheelSpeed = speed - delta; // delta is positive, Slow down RWS
				leftWheelSpeed = speed + delta;  // Speed up LWS
			} else { // We are exactly where we want to be
				delta = 0;
				rightWheelSpeed = speed;
				leftWheelSpeed = speed;
			}
			prevLeftDerivitive = derivitive;
			mbotLed->setColor(2, 0, 0, 0);
			Serial.print("followWall: LEFT");
			Serial.print(distance);
			Serial.print(" delta: ");
			Serial.print(delta);
			Serial.print(" rws:");
			Serial.print(rightWheelSpeed);
			Serial.print(" lws:");
			Serial.print(leftWheelSpeed);
			Serial.println(".");
			break;
		case RIGHTWALL: // Follow along right wall
			mbotLed->setColor(4, 0, 200, 0);
			distance = getDistance(RIGHTWALL);
			derivitive = distance - prevRightDistance;
			delta = WALLDISTANCE - distance;
			if (delta < 0) { // delta is negative, too far from the wall
				if (derivitive < prevRightDerivitive) // Still heading away from wall
					delta = delta * K;
				rightWheelSpeed = speed - delta;
				leftWheelSpeed = speed + delta;
			} else if (delta > 0) { // delta is positive, too close to the wall.
				if (derivitive > prevRightDerivitive) // Still heading away from wall
					delta = delta * K;
				rightWheelSpeed = speed + delta;
				leftWheelSpeed = speed - delta;
			} else {
				delta = 0;
				rightWheelSpeed = speed;
				leftWheelSpeed = speed;
			}
			prevRightDerivitive = derivitive;
			mbotLed->setColor(4, 0, 0, 0);
			Serial.print("followWall: RIGHT:");
			Serial.print(distance);
			Serial.print(" delta: ");
			Serial.print(delta);
			Serial.print(" rws:");
			Serial.print(rightWheelSpeed);
			Serial.print(" lws:");
			Serial.print(leftWheelSpeed);
			Serial.println(".");
			break;
		case NOWALL:
		default:
			Serial.println("followWall: NOWALL");
			distance = 0;
			rightWheelSpeed = 0;
			leftWheelSpeed = 0;
			break;
	}

	mbotMotor->motor_run(leftWheelSpeed, rightWheelSpeed);
} // followWall

int mbot::moveAlongWall()
{
	//Serial.println("START");

	findWall();

	switch(wall) {
		case FRONTWALL: /* for now it's a barrier, not obsticle.*/
			do_180();
			break;
		case RIGHTWALL:
			followWall(); /* Keep following the wall. */
			break;
		case LEFTWALL:
			followWall(); /* Keep following the wall. */
			break;
		case NOWALL:
			//do_turn(); /* We are at a corner */
			break;
		default:
			Serial.println("moveAlongWall: default");
			followWall(); /* FIXME */
			break;
	} // switch

	// Store previous measurements
	storeMearurements();
} // moveAlongWall
#endif // __MBOT_H_
