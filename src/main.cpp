#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <IBusBM.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//SD Card
#include <FS.h>
#include <SD.h>
#include <SPI.h>

//Class level objects
IBusBM IBus;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

//XYZ Acceleration Values
int16_t xAxisAccel;
int16_t yAxisAccel;
int16_t zAxisAccel;

//Yaw, Pitch, and Roll Angles
float yawRate;
float pitchAngle;
float rollAngle;

float accelX;
float accelY;
float accelZ;

double gpsSpeed;

const uint16_t steeringMin = 1100;
const uint16_t steeringMax = 1900;
const uint16_t steeringCenter = 1500;

const uint16_t throttleMin = 1000;
const uint16_t throttleMax = 2000;
const uint16_t throttleCenter = 1000;

//Proportional constants
const uint16_t YawSteeringCutRate = 7.5;
const uint16_t ThrottleCutRate = 7.5;
const uint16_t ThrottleAccelRate = 5;

// Digital Pins
#define THROTTLE_OUT_PIN 14
#define STEERING_OUT_PIN 27
#define RX2_PIN 16
#define TX2_PIN 17
#define RX1_PIN 4
#define TX1_PIN 3
#define SD_CS 5

Servo servoThrottle;
Servo servoSteering;

// create variables to hold a local copies of the channel inputs
uint16_t ThrottleIn;
uint16_t SteeringIn;

uint16_t ThrottleOut;
uint16_t SteeringOut;

//String to store lines of data message being written to SD card
String dataMessage;
uint32_t dataLineCounter = 0;

SoftwareSerial ss(RX1_PIN, TX1_PIN);

// Function Definitions
void processMPU();
void processRecieverInput();
void processStabilityControl();
void processGPS();
float mod(float a);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void logData();

/*************************************************************************************
  Setup
***********************************************************************************/
void setup()
{
  Serial.begin(115200);
  ss.begin(9600);

  IBus.begin(Serial2, 1, RX2_PIN, TX2_PIN); // Serial bus for reading reciever

  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoSteering.attach(STEERING_OUT_PIN);

  if (!mpu.begin())
  {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }

  // Initialize SD card
  SD.begin(SD_CS);
  if (!SD.begin(SD_CS))
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS))
  {
    Serial.println("ERROR - SD card initialization failed!");
    return; // init failed
  }

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data.txt");
  if (!file)
  {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "Line Number, ThrottleIn, ThrottleOut, SteeringIn, SteeringOut, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, GPS Speed\r\n");
  }
  else
  {
    Serial.println("File already exists");
  }
  file.close();

  /*
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(62);
  mpu.setYGyroOffset(11);
  mpu.setZGyroOffset(-22);
  mpu.setXAccelOffset(-1761);
  mpu.setYAccelOffset(-1089);
  mpu.setZAccelOffset(976); // 1688 factory default for my test chip
  */
}

/*************************************************************************************
  Loop
***********************************************************************************/
void loop()
{
  processMPU();
  processRecieverInput();
  processStabilityControl();
  processGPS();
  logData();
}

/*************************************************************************************
  Process MPU
***********************************************************************************/
void processMPU()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  yawRate = g.gyro.heading * 180 / PI;
  pitchAngle = g.gyro.pitch * 180 / PI;
  rollAngle = g.gyro.roll * 180 / PI;

  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
}

/*************************************************************************************
  Process Interrupts
***********************************************************************************/
void processRecieverInput()
{
  ThrottleIn = IBus.readChannel(2); // get latest value for servo channel 3
  SteeringIn = IBus.readChannel(0); // get latest value for the servo channel 1
                                    /*
  for (int i = 0; i < 5; i++)
  {
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(IBus.readChannel(i));
  }
*/

  Serial.print("ThrottleIn: ");
  Serial.println(ThrottleIn);
  Serial.print("SteeringIn: ");
  Serial.println(SteeringIn);
}

long stabilityTimerMicros = 0;
const uint16_t stabilityInterval = 5000;
uint16_t previousThrottleOut = 0;

/*************************************************************************************
  Process Stability Control
***********************************************************************************/
void processStabilityControl()
{
  float dt = micros() - stabilityTimerMicros;
  float yawRate = 0; //Yaw angle change with respect to time, degrees/sec

  if (dt > stabilityInterval)
  {
    SteeringOut = SteeringIn - yawRate * YawSteeringCutRate;
    Serial.print("Yaw rate times yaw cut rate");
    Serial.println(yawRate * YawSteeringCutRate);
    ThrottleOut = ThrottleIn - fabsf(yawRate) * ThrottleCutRate; //proportional controller.

    if (xAxisAccel > 500) //Traction control based on max accel rate
    {
      ThrottleOut = ThrottleIn - xAxisAccel * ThrottleCutRate;
    }

    if (ThrottleOut > previousThrottleOut)
    {
      ThrottleOut = previousThrottleOut + ThrottleAccelRate;
    }

    previousThrottleOut = ThrottleOut;
    //Constraint the outputs just in case
    SteeringOut = constrain(SteeringOut, steeringMin, steeringMax);
    ThrottleOut = constrain(ThrottleOut, throttleMin, throttleMax);

    //Safety in that ensures the Arduino doesn't turn on the motor prior to the transmitter connecting which has happened :-(
    if (ThrottleIn < 1050)
    {
      ThrottleOut = throttleMin;
    }

    //Center Steering if transmitter signal lost
    if (SteeringIn < 995)
    {
      SteeringIn = steeringCenter;
    }

    Serial.print("Steering Servo OUT");
    Serial.println(SteeringOut);
    Serial.print("Throttle Servo OUT");
    Serial.println(ThrottleOut);

    //Write output to Servos
    servoThrottle.writeMicroseconds(ThrottleOut);
    servoSteering.writeMicroseconds(SteeringOut);

    stabilityTimerMicros = micros();
  }
}

/*************************************************************************************
  Process GPS
***********************************************************************************/
void processGPS()
{
  if (ss.available() > 0)
  {
    gps.encode(ss.read());
    if (gps.speed.isUpdated())
    {
      gpsSpeed = gps.speed.kmph();
      // Speed
      Serial.print(" Speed= ");
      Serial.println(gps.speed.kmph(), 6);
    }
  }
}

long dataLongMicros = 0;
const uint16_t dataLogInterval = 10000;

void logData()
{
  float dt = micros() - dataLongMicros;

  if (dt > dataLogInterval)
  {
    // "Line Number, ThrottleIn, ThrottleOut, SteeringIn, SteeringOut, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, GPS Speed
    dataMessage = String(dataLineCounter) + "," + String(ThrottleIn) + "," + String(ThrottleOut) + "," +
                  String(SteeringIn) + "," + String(SteeringOut) + "," + String(accelX) + "," + String(accelY) + "," +
                  String(accelZ) + "," + String(rollAngle) + "," + String(pitchAngle) + "," + String(yawRate) + "," + String(gpsSpeed) + "\r\n";
    Serial.print("Save data: ");
    Serial.println(dataMessage);
    dataLineCounter++;
    appendFile(SD, "/data.txt", dataMessage.c_str());

    dataLongMicros = micros();
  }
}

// Write to the SD card
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card
void appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}
