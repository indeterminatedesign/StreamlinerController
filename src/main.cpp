#include <Arduino.h>
#include <Servo.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// Two channels in
// Two channels out

//XYZ Acceleration Values
uint16_t xAxisAccel;
uint16_t yAxisAccel;
uint16_t zAxisAccel;

//Yaw, Pitch, and Roll Angles
float yawAngle;
float pitchAngle;
float rollAngle;

const uint16_t steeringMin = 1100;
const uint16_t steeringMax = 1900;
const uint16_t steeringCenter = 1500;

const uint16_t throttleMin = 1000;
const uint16_t throttleMax = 2000;
const uint16_t throttleCenter = 1000;

const uint16_t auxMin = 1000;
const uint16_t auxMax = 2000;
const uint16_t auxCenter = 1000;

const uint16_t RollSteeringCutRate = 10;
const uint16_t YawSteeringCutRate = 7.5;
const uint16_t ThrottleCutRate = 7.5;
const uint16_t ThrottleAccelRate = 1;

//////////////////////////////////////////////////////////////////
// DIGITAL PINS
//////////////////////////////////////////////////////////////////
#define THROTTLE_IN_PIN 2
#define STEERING_IN_PIN 3
#define THROTTLE_OUT_PIN 8
#define STEERING_OUT_PIN 9
#define INTERRUPT_PIN 1

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;
uint32_t ulAuxStart;

Servo servoThrottle;
Servo servoSteering;

// create local variables to hold a local copies of the channel inputs
// these are declared static so that thier values will be retained
// between calls to loop.
uint16_t unThrottleIn;
uint16_t unSteeringIn;

uint16_t unThrottleOut;
uint16_t unSteeringOut;

// local copy of update flags
uint8_t bUpdateFlags;

uint32_t ulMillis = millis();

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void processMPU();
void processInterrupts();
void processStabilityControl();
float mod(float a);

// ================================================================
// ===               INTERRUPT SERVICE ROUTINES                ===
// ================================================================
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

//ISR for the Throttle Input
void ISRThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if (digitalRead(THROTTLE_IN_PIN))
  {

    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

//ISR for the Steering Input
void ISRSteering()
{
  if (digitalRead(STEERING_IN_PIN))
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void setup()
{
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), ISRThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN), ISRSteering, CHANGE);

  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoSteering.attach(STEERING_OUT_PIN);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(62);
  mpu.setYGyroOffset(11);
  mpu.setZGyroOffset(-22);
  mpu.setXAccelOffset(-1761);
  mpu.setYAccelOffset(-1089);
  mpu.setZAccelOffset(976); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop()
{
  processMPU();
  processInterrupts();
  processStabilityControl();

  //Logging
  Serial.print("Steering Servo In");
  Serial.println(unSteeringIn);
  Serial.print("Throttle Servo In");
  Serial.println(unThrottleIn);
}

/*************************************************************************************
  Process MPU
 ***********************************************************************************/
void processMPU()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    if (mpuInterrupt && fifoCount < packetSize)
    {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize)
  {
    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
    // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT))
  {

    // read a packet from FIFO
    while (fifoCount >= packetSize)
    { // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }

    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //Store the acceleration vecotors in variables
    xAxisAccel = aaReal.x;
    yAxisAccel = aaReal.y;
    zAxisAccel = aaReal.z;

    //Store Yaw, Pitch, Roll into variables
    yawAngle = ypr[0] * 180 / M_PI;
    pitchAngle = ypr[1] * 180 / M_PI;
    rollAngle = ypr[2] * 180 / M_PI;

    //    Serial.print(aaReal.x);
    //    Serial.print("\t");
    //    Serial.print(aaReal.y);
    //    Serial.print("\t");
    //    Serial.println(aaReal.z);
    //
    //    Serial.print("ypr\t");
    //    Serial.print(ypr[0] * 180 / M_PI);
    //    Serial.print("\t");
    //    Serial.print(ypr[1] * 180 / M_PI);
    //    Serial.print("\t");
    //    Serial.println(ypr[2] * 180 / M_PI);
  }
}

/*************************************************************************************
  Process Interrupts
 ***********************************************************************************/
void processInterrupts()
{
  // check shared update flags to see if any channels have a new signal
  if (bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if (bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if (bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    //Logging
    Serial.print("Steering Servo In");
    Serial.println(unSteeringIn);
    Serial.print("Throttle Servo In");
    Serial.println(unThrottleIn);

    interrupts();
  }
}

long stabilityTimerMicros = 0;
const uint16_t stabilityTime = 10000;
float yawAnglePrevious = 0;
uint16_t previousThrottleOut = 0;

/*************************************************************************************
  Process Stability Control
***********************************************************************************/
void processStabilityControl()
{
  float dt = micros() - stabilityTimerMicros;
  float yawRate = 0; //Yaw angle change with respect to time, degrees/sec

  if (dt > stabilityTime)
  {

    yawRate = (yawAngle - yawAnglePrevious) / (dt / 1000000); // Calculate yaw rate degrees/sec

    Serial.print("Yaw Rate");
    Serial.println(yawRate);
    Serial.print("Yaw Angle");
    Serial.println(yawAngle);
    Serial.print("Yaw Angle Previous");
    Serial.println(yawAnglePrevious);
    Serial.print("Delta T");
    Serial.println(dt / 1000000);

    yawAnglePrevious = yawAngle; // Store the current yaw value for the next cycle

    unSteeringOut = unSteeringIn - yawRate * YawSteeringCutRate;
    Serial.print("Yaw rate times yaw cut rate");
    Serial.println(yawRate * YawSteeringCutRate);
    unThrottleOut = unThrottleIn - mod(yawRate) * ThrottleCutRate; //proportional controller.

    //  if (mod(rollError) > 10)
    //  {
    //    //Add more counter steer if the car is about to roll over and cut throttle
    //   unSteeringOut += unSteeringIn + rollError * RollSteeringCutRate; //proportional controller.
    // unThrottleOut = throttleMin;
    // }

    if (unThrottleOut > previousThrottleOut)
    {
      unThrottleOut = previousThrottleOut + ThrottleAccelRate;
      // unThrottleOut = EMA_function(0.06F, unThrottleOut, previousThrottleOut); //Delay throttle through EMA if it is increasing, but throttle cuts are immediate.
    }

    previousThrottleOut = unThrottleOut;
    //Constraint the outputs just in case
    unSteeringOut = constrain(unSteeringOut, steeringMin, steeringMax);
    unThrottleOut = constrain(unThrottleOut, throttleMin, throttleMax);

    //Safety in that ensures the Arduino doesn't turn on the motor prior to the transmitter connecting which has happened :-(
    if (unThrottleIn < 1050)
    {
      unThrottleOut = throttleMin;
    }

    if (unSteeringIn < 1050)
    {
      unSteeringIn = steeringCenter;
    }

    Serial.print("Steering Servo OUT");
    Serial.println(unSteeringOut);
    //Serial.print("Throttle Servo OUT");
    //Serial.println(unThrottleOut);

    //Write output to Servos
    servoThrottle.writeMicroseconds(unThrottleOut);
    servoSteering.writeMicroseconds(unSteeringOut);

    stabilityTimerMicros = micros();
  }
  bUpdateFlags = 0;
}

float mod(float a)
{
  if (a < 0)
  {
    return -a;
  }
  return a;
}

float EMA_function(float alpha, int latest, int stored)
{
  return alpha * latest + (1 - alpha) * stored;
}