#include <uStepperSLite.h>
#include <SPI.h>
#include <Wire.H>
#include <MPU6050_tockn.h>

#define DEBUG_MODE 1

#if DEBUG_MODE
#define DEBUG(val)                              \
  Serial.println("["__func__"]\t" val)
#else
#define DEBUG(val)
#endif

/*
 * =====================================
 *  Pin definitions for uStepper S-Lite
 * =====================================
 */ 
#define SS1 19      // SPI Controller select
#define SCK1 24     // SPI Clock reference
#define MISO1 23    // SPI Master In Slave Out
#define MOSI1 22    // SPI Master Out Slave In

#define IMU_SDA 27  // I2C Serial Data Line
#define IMU_SCL 28  // I2C Serial Clock Line

#define REV_D1 31   // DC Motor Reverse Pin
#define FOR_D0 30   // DC Motor Forward Pin


uStepperSLite stepper; // uStepper motor controller object
MPU6050 mpu6050(Wire); // Gyroscope Object

// Small wrapper function to get angles into angleArray
/* 
 * TODO: Note not all angles a needed, eliminate unneeded angles
 * once done with HW
*/
float getAngle(){
  mpu6050.update();
  return mpu6050.getAngleZ();
}

// General PID Vars
float time, timePrev, elTime;
float angle, anglePrevErr, angleErr;
int period = 50;

/*
 * PID Constants, probably needs some fine tuning
 */
float kp=0; // Proportional gain
float ki=0; // Integral gain
float kd=0; // Derivative gain

float pidP, pidI, pidD, pidTotal;

void PIDCalc(float targetAngle){
  if (millis() > time+period){
    time = millis();
    angle = getAngle();
    angleErr = targetAngle - angle;
    pidP = kp * angleErr;

    float angleDif = angleErr - anglePrevErr;
    pidD = kd * (angleDif/period);
    if( -3 < angleErr && angleErr < 3)
      pidI = pidI + (ki angleErr);
    else
      pidI = 0;

    anglePrevErr = angleErr;

    pidTotal = pidP + pidI + pidD;
  }
}


/* =======================================
 * Motor direction enum
 *  - CW = clockwise
 *  - CCW = counter clockwise
 *  - STOP = STOP THE FUCKING MOTOR 
 */
typedef enum DIR {
  CW,
  CCW,
  STOP
} motorDIR;

void DCMotorControl(motorDIR dir){
  switch(dir){
  case CW:
    // TODO: Make motor go brrrr clockwise
    break;
  case CCW:
    // TODO: Make motor go brrrr counter clockwise
    break;
  case STOP:
  default:
    // TODO: Make motor stop
    break;
  }
}















// DO LATER:
void setup() {
  Serial.begin(9600);
  DEBUG("Starting Wire")
  Wire.begin();
  DEBUG("Starting MPU6050")
  mpu6050.begin();
  DEBUG("Calculating Gyro Offsets for MPU6050")
  mpu6050.calcGyroOffsets();
  DEBUG("Done")
}

void loop() {
  
}
