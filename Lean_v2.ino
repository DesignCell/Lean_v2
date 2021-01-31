/*
    Name:       Lean_v2.ino
    Created:	1/6/2019 1:22:48 PM
    Author:     DESIGNCELL-PC\DesignCell
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Keyboard.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int angle = 3;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	Serial.begin(250000);
	//while (!Serial); // wait for Leonardo enumeration, others continue immediately
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	//mpu.setDMPEnabled(true);

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);
	
		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
	
		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;
	
		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

void loop() {
	// if programming failed, don't try to do anything
	//if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	//while (!mpuInterrupt && fifoCount < packetSize) {

	//}

	//mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
		//Serial.println(F("FIFO overflow!"));
	}
	else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		float lean = ypr[2] * 180 / M_PI;

		if (abs(lean) < angle) 
		{
			//Keyboard.releaseAll();
			//Keyboard.end();
			Serial.println(lean);
			Serial.println('-');
		}
		else if (lean >= angle)
		{
			//Keyboard.begin();
			//Keyboard.press('i');
			Serial.println(lean);
			Serial.println('L');
		}
		else if (lean <= -angle)
		{
			//Keyboard.begin();
			//Keyboard.press('o');
			Serial.println(lean);
			Serial.println('R');
		}

#endif
	}
}
