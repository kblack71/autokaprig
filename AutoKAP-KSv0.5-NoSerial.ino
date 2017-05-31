/**
 AutoKAP Gimbal w. Servo and GY521
 SONY HX50 with Cableshutter and Landscape 1/2 Sphere Panorama
 
 Karsten Schwarz
 */

// #include "math.h"
#include "Servo.h"
#include "StepperMotor.h"
// * http://www.codeproject.com/Articles/699986/An-Arduino-Library-to-Control-the-BYJ-Stepper

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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// =======================================================
// === Photo Part
// =======================================================

Servo servoA;
StepperMotor motorTilt(7, 8, 9, 10);
StepperMotor motorPan(3, 4, 5, 6);

// const int stepsPerRevolution = 4096; // see StepperMotor.h
const int SERVO_MIDDLE = 90;
const int PINS_BTN_AE = 13;
const int PINS_BTN_AF = 12;
const int CAM_SHOOTING_PULSE = 500;
const int PRE_TRIGGER_TIME = 1500;

float pitch;
float rollAngle;
float rA;
float pS1;
float pS2;
int roll;
int pitchSteps;
int lvlruns;
int servoPos;
int servoAngle;
int tiltCount = 0;

// Panorama Steps for 22mm equiv. - landscape
int anglePan0 = 0;
int anglePan1 = 585;    // turn 7 x 51 degrees for each pic
int angleTilt0 = 0;     // horizontal
int angleTilt1 = 250;  // steps to turn 22 degrees _down_ for 2nd row
int angleTilt2 = -250;   // steps up for 3rd _upper_ row if full spherical pano wanted
int angleNadir = 1024;  // step from horizontal to nadir
int angleZenit = -1024;   // NOT POSSIBLE with my gimbal !!
unsigned int pics = 0;      // number of pictures taken
unsigned int p_pics = 2;   // number of pictures for a full/half panosphere
unsigned int prow_pics = 7; // number of pictures for 1 pano round
unsigned int h_pics;
unsigned int v1_pics;
unsigned int v2_pics;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===        CAMERA TRIGGER - AF / SHUTTER RELEASE             ===
// ================================================================
void triggerCam() {
  delay(1000);
  digitalWrite(PINS_BTN_AF, HIGH);
  // // Serial.print("Focus - ");
  delay(PRE_TRIGGER_TIME);
  digitalWrite(PINS_BTN_AE, HIGH);
  // // Serial.print("Klick");
  delay(CAM_SHOOTING_PULSE);
  digitalWrite(PINS_BTN_AF, LOW);
  digitalWrite(PINS_BTN_AE, LOW);
  delay(1000);
}

// ================================================================
// ===        TAKE PICTURES FOR 1 PANO ROW                      ===
// ================================================================
void shootRow() {
  triggerCam();
  motorPan.move(anglePan1);
}

// ================================================================
// ===      LEVEL CAMERA HORIZONTAL                             ===
// ================================================================
int levelCam(int roll, int pitchSteps) {
  // // Serial.println("Level Camera");
  lvlruns = 0;
  do {
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        pS1 = 0;
        pS2 = 0;
        // display ypr angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // // Serial.print("ypr\t");
        // // Serial.print(ypr[0] * 180 / M_PI);
        // // Serial.print("\t");
        // // Serial.print(ypr[1] * 180 / M_PI);
        // // Serial.print("\t");
        // // Serial.println(ypr[2] * 180 / M_PI);
      
        // Get Z & Y Position of the Camera Plate
        rollAngle = (ypr[2] * 180 / M_PI);
        pitch = (ypr[1] * 180 / M_PI);
      
        // Get Servohead Position
        servoPos =  servoA.read();
        // Serial.print("Servo Position\t");
        // Serial.println((int)servoPos);
      
        // round the Z Positon Float and cast to integer for Servo Movement
        if ( rollAngle >= 0 ) {
          rA = rollAngle + 0.5;
        } else {
          rA = rollAngle + -0.5;
        }
        roll = (int)rA;
        servoAngle = SERVO_MIDDLE - roll;
        
        // Serial.print("roll\t");
        // Serial.println(roll);
        servoA.write(servoAngle);
        delay(590);
        // Serial.print("Servo Angle\t");
        // Serial.println(servoAngle);
      
        // Round Y Float and cast to int for Steppermotor
        pS1 = (pitch / 0.088);
        if ( pS1 >= 0 ) {
          pS2 = pS1 + 0.5;
        } else {
          pS2 = pS1 + -0.5;
        }
        
        pitchSteps = 0 + (int)pS2;
        motorTilt.move(pitchSteps);
      
        // Serial.print("Pitch Steps\t");
        // Serial.println(pitchSteps);
        delay(1000);
        lvlruns++;
         }
  } while ( ( roll != 0 && pitchSteps != 0 ) && lvlruns < 10);
  // Serial.print("roll:\t");
  // Serial.print(roll);
  // Serial.print("\tpitch\t");
  // Serial.print(pitchSteps);
  // Serial.print("\truns\t");
  // Serial.println(lvlruns);
  return roll;
  return pitchSteps;
} // END levelCAM

// ================================================================
// ===                  END FUNCTIONS                           ===
// ================================================================

void setup() {
  // put your setup code here, to run once:
  motorPan.setPeriod(5);
  motorTilt.setPeriod(5);
  servoA.attach(11);
  servoA.write(SERVO_MIDDLE);
  pinMode(PINS_BTN_AE, OUTPUT);
  pinMode(PINS_BTN_AF, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  // Serial.begin(115200);
  // initialize device
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  // Serial.println(F("Testing device connections..."));
  // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // load and configure the DMP
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // get MPU6050 calibration script for this
  mpu.setXAccelOffset(225);
  mpu.setYAccelOffset(1078);
  mpu.setXGyroOffset(-16);
  mpu.setYGyroOffset(107);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1814); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    // Serial.print(devStatus);
    // Serial.println(F(")"));
  }
}

// ===========================================================================
// ===          MAIN LOOP
// ===========================================================================

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("Starting Loop");

  // Wait for Lift?
  // delay(10000);
  
  // give the Camera leveling something to do
  pitchSteps = 4;
  roll = -2;
 // do {
    mpu.resetFIFO();
    levelCam(roll, pitchSteps);
 //} while ( roll != 0 && pitchSteps != 0);

// Serial.println("Running Pano"); 
// for ( pics = 0; pics <= p_pics; pics++) {
    // take pictures for the horizontal row
    for ( h_pics = 0; h_pics <= prow_pics; h_pics++) {
      // Serial.println("Hor. Row");
      shootRow();
    }

    // pitch down for 2nd row
    motorTilt.move(angleTilt1);
    tiltCount = (tiltCount + angleTilt1);
    for ( v1_pics = 0; v1_pics <= prow_pics; v1_pics++) {
      shootRow();
    }
    
    // Vertical Row UP - interferes with my Gimbal construction  
    //
    // motorTilt.move(angleTilt1);
    // tiltCount = (tiltCount + angleTilt1);
    // 
    // for ( v1_pics = 0; v1_pics <= prow_pics; v1_pics++) {
    //   shootRow();
    // }
    motorTilt.move(-tiltCount);
    // Serial.print("Back to Hor.");
    // Serial.println(tiltCount);
    delay(1000);
    tiltCount = 0;
    motorTilt.move(angleNadir);
    // Serial.print("Nadir\t");
    // Serial.println(angleNadir);
    triggerCam();
    motorTilt.move(-angleNadir);
  // }

  //END
}
