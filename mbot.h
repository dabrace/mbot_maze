#if !defined(__MBOT_H_)
#define __MBOT_H_

#include "motor.h"
#include "ultrasonic.h"
#include "led.h"

/*
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
*/

#define WALLDISTANCE 10.58 // 1" = 2.54 CM // Not sure if it is really in CM.
#define MOTORSPEED 80.0
#define SPEED_DELTA_MAX 15.0
#define TURNDELAY 0.3  // Arbitrary guess based on max speed.
#define K 2.0

// Need to look at mbot to see what RJ25 ports the sensors are plugged into
#define FRONT_US_SENSOR_PORT 3
#define LEFT_US_SENSOR_PORT 1
#define RIGHT_US_SENSOR_PORT 2

#define LEFTMOTORPORT 9
#define RIGHTMOTORPORT 10

#define LEDPORT 4

#define FRONT 0
#define RIGHT 1
#define LEFT 2

#define NOWALL -1

class mbot {
public:
	mbot();
	void mbotTurn(int degrees, int dir);
	void move(int direction, int speed);
	int moveAlongWall();
	int followRightWall();
	int followLeftWall();
	int do_180(int dir);
	int do_turn(int speed, int dir);
	float normalizeDelta(int delta);
	int findWall();
	int isWallInFront();
	void _delay(float seconds);
	void _loop();
	void takeDistanceMeasurements();
	void storeMearurements();
	double getDistance(int dir);
private:
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
	int followWall(int wall);
	int currentWall;
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

	currentWall = RIGHT;
	takeDistanceMeasurements();
	storeMearurements();
	prevRightDerivitive = 0;
	prevLeftDerivitive = 0;

	Serial.begin(9600);
}

void mbot::_loop()
{
}

void mbot::_delay(float seconds)
{
	long endTime = millis() + seconds * 1000;
	while(millis() < endTime)_loop();
}

double mbot::getDistance(int dir)
{
	double distance = 0.0;
	double delta = 0.0;
	int i;

	// Try to throw away any outliars
	for (i = 0; i < 10; i++) {
		switch(dir) {
		case RIGHT:
			distance = mbotUltrasonic->rightDistanceCM();
			delta = abs(distance - prevLeftDistance);
			break;
		case LEFT:
			distance = mbotUltrasonic->leftDistanceCM();
			delta = abs(distance - prevRightDistance);
			break;
		case FRONT:
			distance = mbotUltrasonic->frontDistanceCM();
			delta = abs(distance - prevFrontDistance);
			break;
		} // switch

		if (delta <= K)
			break;
	} // for
}

void mbot::takeDistanceMeasurements()
{
	frontDistance = mbotUltrasonic->frontDistanceCM();
	leftDistance = mbotUltrasonic->leftDistanceCM();
	rightDistance = mbotUltrasonic->rightDistanceCM();
}

void mbot::storeMearurements()
{
	prevLeftDistance = leftDistance;
	prevRightDistance = rightDistance;
	prevFrontDistance = frontDistance;
}

int mbot::isWallInFront()
{
	double front;

	front = getDistance(FRONT);
	if (front < WALLDISTANCE) {
		mbotLed->setColor(1, 0, 200, 0);
		//_delay(0.4);
		mbotLed->setColor(1, 0, 0, 0);
		return 1;
	}
	return 0;
}

/*
 * find a wall nearest to the mbot.
 *
 * Takes all distance measurements
 */
int mbot::findWall()
{
	int wall = NOWALL;

	leftDistance = getDistance(LEFT);
	if (leftDistance <= (K+WALLDISTANCE)) {
		mbotLed->setColor(2, 200, 0, 0);
		mbotLed->setColor(2, 0, 0, 0);
		return LEFT;
	}

	frontDistance = getDistance(FRONT);
	if (frontDistance <= WALLDISTANCE) {
		mbotLed->setColor(1, 200, 0, 0);
		mbotLed->setColor(1, 0, 0, 0);
		return FRONT;
	}

	rightDistance = getDistance(RIGHT);
	if (rightDistance <= (K+WALLDISTANCE)) {
		mbotLed->setColor(4, 200, 0, 0);
		mbotLed->setColor(4, 0, 0, 0);
		return RIGHT;
	}

	return wall;
}

void mbot::mbotTurn(int degrees,int dir)
{
	switch (dir) {
	}
}

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

int mbot::do_180(int dir)
{
	MeUltrasonicSensor *mbot_ultrasonic;
	int wall;

	switch (dir) {
	/*
	 * Wall is on the left, do 180 right
	 */
	case LEFT:
		mbot_ultrasonic = mbotUltrasonic->getRightP();
		rightWheelSpeed = -speed;
		leftWheelSpeed = speed;
		wall = RIGHT;
		break;
	/*
	 * Wall is on the right, do 180 left
	 */
	case RIGHT:
		mbot_ultrasonic = mbotUltrasonic->getLeftP();
		rightWheelSpeed = speed;
		leftWheelSpeed = -speed;
		wall = LEFT;
		break;
	default:
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
	//mbotMotor->motor_run(leftWheelSpeed, rightWheelSpeed);

	return wall;
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
int mbot::do_turn(int speed, int dir)
{
	int wall = NOWALL;

	MeUltrasonicSensor *mbot_ultrasonic;

	mbotLed->setColor(4, 0, 0, 200);
	mbotLed->setColor(2, 0, 0, 200);

	/*
	 * Do not know how far the mbot has traveled
	 * so, delay a while to get enough of the mbot
	 * past the corner to turn.
	 */
	_delay(TURNDELAY);

	mbotLed->setColor(4, 0, 0, 0);
	mbotLed->setColor(2, 0, 0, 0);

	switch (dir) {
	/*
	 * Wall was on the left, turn left
	 */
	case LEFT:
		mbot_ultrasonic = mbotUltrasonic->getLeftP();
		rightWheelSpeed = speed;
		leftWheelSpeed = speed/4;
		wall = LEFT;
		mbotLed->setColor(4, 0, 0, 0);
		mbotLed->setColor(2, 200, 0, 0);
		break;
	/*
	 * Wall was on the right, turn right
	 */
	case RIGHT:
		mbot_ultrasonic = mbotUltrasonic->getRightP();
		rightWheelSpeed = speed/4;
		leftWheelSpeed = speed;
		wall = RIGHT;
		mbotLed->setColor(4, 200, 0, 0);
		mbotLed->setColor(2, 0, 0, 0);
		break;
	default:
		mbot_ultrasonic = mbotUltrasonic->getRightP();
		rightWheelSpeed = speed/4;
		leftWheelSpeed = speed;
		wall = RIGHT;
		mbotLed->setColor(4, 0, 200, 0);
		mbotLed->setColor(2, 0, 200, 0);
		break;
	}

	/*
	 * The idea is to keep turning until the sensor reads the proper
	 * distance from the wall.
	 */
	mbotLed->setColor(1, 0, 0, 200);
	mbotLed->setColor(3, 0, 0, 200);
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

	return wall;
} // do_turn

/*
 *========================================================================
 * Follow wall to the right
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

int mbot::followRightWall()
{
	followWall(RIGHT);
}

int mbot::followLeftWall()
{
	followWall(LEFT);
}

// Calculate the delta based on the buffer zone and current distance
// and multiply times a constant value K to smooth out the jitter.
// Delta will be negative if the distance is larger than ideal dist.
// Therefore a distance > WALLDISTANCE will:
//      1. slow down the right wheel speed and
// 	2. speed up the left wheel speed.
// Avoid too high of a speed delta.
// Helps prevent spinnig in circles when distances are large
int mbot::followWall(int wall)
{
	double distance;
	double delta;
	double derivitive;

	/*
	 * Assign right and left wheel speeds.
	 * Recalculate upon each entry into the routine.
	 *
	 * If delta is positive, then we are too close to the wall.
	 * If delta is negative, then we are too far away from the wall.
	 *
	 * Use the first derivitive to determine if we have not corrected enough.
	 */
	switch (wall) {
		case FRONT:
			break;
		case LEFT: // Follow along left wall
			mbotLed->setColor(2, 0, 200, 0);
			distance = getDistance(LEFT);
			derivitive = distance - prevLeftDistance;
			delta = WALLDISTANCE - distance;
			if (delta < 0) { // delta is positive, too close to wall
				if (derivitive < prevLeftDerivitive) // Still heading towards wall
					delta = delta * K;
				rightWheelSpeed = speed + delta;
				leftWheelSpeed = speed - delta;
			} else if (delta > 0) { // delta is negative, too farr from wall
				if (derivitive > prevLeftDerivitive) // Still heading away from wall
					delta = delta * K;
				rightWheelSpeed = speed - delta;
				leftWheelSpeed = speed + delta;
			} else {
				delta = 0;
				rightWheelSpeed = speed;
				leftWheelSpeed = speed;
			}
			prevLeftDerivitive = derivitive;
			mbotLed->setColor(2, 0, 0, 0);
			break;
		case RIGHT: // Follow along right wall
			mbotLed->setColor(4, 0, 200, 0);
			distance = getDistance(RIGHT);
			derivitive = distance - prevRightDistance;
			delta = WALLDISTANCE - distance;
			if (delta < 0) { // delta is positive, too close to wall
				if (derivitive < prevRightDerivitive) // Still heading towards wall
					delta = delta * K;
				rightWheelSpeed = speed - delta;
				leftWheelSpeed = speed + delta;
			} else if (delta > 0) { // delta is negative, too far from wall.
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
			break;
		case NOWALL:
		default:
			distance = 0;
			rightWheelSpeed = 0;
			leftWheelSpeed = 0;
			break;
	}

	mbotMotor->motor_run(leftWheelSpeed, rightWheelSpeed);
}

int mbot::moveAlongWall()
{
	int wall;
	int frontWall;

	wall = findWall();

	//if (wall == NOWALL)
		//wall = do_turn(speed, currentWall);

	currentWall = wall;

	Serial.println("Test");
	switch(wall) {
		case FRONT:
			wall = do_180(wall);
			break;
		case RIGHT:
			followRightWall();
			break;
		case LEFT:
			followLeftWall();
			break;
		default:
			//followWall(NOWALL);
			break;
	}

	// Store previous measurements
	storeMearurements();
}
#endif // __MBOT_H_
