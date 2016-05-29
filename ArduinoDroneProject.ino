//Author : Prokuma
//License : MIT License
#include <Wire.h>
#define MOTOR_A_PIN 6
#define MOTOR_B_PIN 10
#define MOTOR_C_PIN 9
#define MOTOR_D_PIN 5

//angle Struct
struct angle {
  double roll;
  double pitch;
  double yaw;
};

//sensor value struct
struct sensorResult {
  int16_t AcX;
  int16_t AcY;
  int16_t AcZ;
  int16_t Tmp;
  int16_t GyX;
  int16_t GyY;
  int16_t GyZ;
};

typedef struct angle Angle;
typedef struct angle Error;
typedef struct sensorResult SensorResult;

struct PIDresult {
  Angle control;
  Error error;
};

struct control {
  Angle aimAngle;
  double throttle;
};

typedef struct PIDresult PIDResult;
typedef struct control Control;

const int MPU_addr = 0x68;
const double Pi = 3.1415926535;
const double GYROXYZ_TO_DEGREES_PER_SEC = 131;

//PID Control Constants
double Kp = 0.6; //P control
double Ki = 0.6; //I control
double Kd = 0.3; //D control

/* readAngle()
 * Read Angle to MPU6050
 */
SensorResult* readAngle(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);

  SensorResult result;
  
  result.AcX = Wire.read() << 8 | Wire.read();
  result.AcY = Wire.read() << 8 | Wire.read();
  result.AcZ = Wire.read() << 8 | Wire.read();
  result.Tmp = Wire.read() << 8 | Wire.read();
  result.GyX = Wire.read() << 8 | Wire.read();
  result.GyY = Wire.read() << 8 | Wire.read();
  result.GyZ = Wire.read() << 8 | Wire.read();

  return &result;
}
/* initMPU6050
 * Initialize MPU6050 Sensor
 */
void initMPU6050(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x68);
  Wire.write(0);
  Wire.endTransmission(true);
}

/* calibAngle
 * calibration Sensor
 */
SensorResult* calibAngle(){
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

  SensorResult result;
  
  for(int i = 0; i < 10; i++){
    SensorResult *cresult = readAngle();
    sumAcX += (*cresult).AcX; sumAcY += (*cresult).AcY; sumAcZ += (*cresult).AcZ;
    sumGyX += (*cresult).GyX; sumGyY += (*cresult).GyY; sumGyZ += (*cresult).GyZ;
    delay(100);
  }

  result.AcX = (int16_t)sumAcX / 10;
  result.AcY = (int16_t)sumAcY / 10;
  result.AcZ = (int16_t)sumAcZ / 10;
  result.GyX = (int16_t)sumGyX / 10;
  result.GyY = (int16_t)sumGyY / 10;
  result.GyZ = (int16_t)sumGyZ / 10;

  result.Tmp = (*readAngle()).Tmp;

  return &result;
}

/* calcAccelAngle
 * SensorResult* sensor : Current Sensor Value
 * SensorResult* basedSenserResult : Based Sensor Value
 */
Angle* calcAccelAngle(SensorResult* sensor, SensorResult* basedSensorResult){
  SensorResult accel;

  accel.AcX = (*sensor).AcX - (*basedSensorResult).AcX;
  accel.AcY = (*sensor).AcX - (*basedSensorResult).AcY;
  accel.AcZ = (*sensor).AcZ + (16384 - (*basedSensorResult).AcZ);

  Angle result;

  result.roll = atan(accel.AcY/sqrt(pow(accel.AcX, 2) + pow(accel.AcZ, 2))) * (180/Pi);
  result.pitch = atan(-accel.AcX/sqrt(pow(accel.AcY, 2) + pow(accel.AcZ, 2))) * (180/Pi);
  result.yaw = 0;

  return &result;
}

/* calcGyroAngle
 * SensorResult* sensor : Current Sensor Value
 * SensorResult* basedSenserResult : Based Sensor Value
 * Angle* result : Result Angle
 * double dt : Time increment
 */
void calcGyroAngle(SensorResult *sensor, SensorResult *basedSensorResult,Angle* result, double dt){
  SensorResult gyro;

  gyro.GyX = ((*sensor).GyX - (*basedSensorResult).GyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro.GyY = ((*sensor).GyY - (*basedSensorResult).GyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro.GyZ = ((*sensor).GyZ - (*basedSensorResult).GyZ) / GYROXYZ_TO_DEGREES_PER_SEC;

  (*result).roll += gyro.GyX * dt;
  (*result).pitch += gyro.GyY * dt;
  (*result).yaw += gyro.GyZ * dt;
}

/* calcFilteredAngle
 * Calculate Combination Angle Filter Funtion
 * Angle* accel : accelorator Angle
 * Angle* gyro : gyroscope Angle
 * Angle* result : Result Angle
 * double dt : Time increment
 */
void calcFilteredAngle(Angle* accel, Angle* gyro, Angle* result, double dt){
  const double ALPHA = 0.96;
  Angle tmp;

  tmp.roll = (*result).roll + (*gyro).roll * dt;
  tmp.pitch = (*result).pitch + (*gyro).pitch * dt;
  tmp.yaw = (*result).yaw + (*gyro).yaw * dt;

  (*result).roll = ALPHA * tmp.roll + (1.0 - ALPHA) * (*accel).roll;
  (*result).pitch = ALPHA * tmp.pitch + (1.0 - ALPHA) * (*accel).pitch;
  (*result).yaw = tmp.yaw;
}

/* PID_Calculate
 * PID Control Calculation Function
 * Parameter:aimAnlge, prevAngle, currentAngle
 * aimAngle(Angle *):set aim angle
 * prevAngle(Angle *):set previous current angle
 * currentAngle(Angle *):set current angle
 * double dt : Time increment
 */
PIDResult* PID_Calculate(Angle *aimAngle, Angle *prevAngle ,Angle *currentAngle, double dt) {

  double currentTime = millis();
  
  Error error;
  error.roll = (*aimAngle).roll - (*currentAngle).roll;
  error.pitch = (*aimAngle).pitch - (*currentAngle).pitch;
  error.yaw = (*aimAngle).yaw - (*currentAngle).yaw;

  Angle P_control, I_control, D_control;
  P_control.roll = Kp * error.roll;
  P_control.pitch = Kp * error.pitch;
  P_control.yaw = Kp * error.yaw;
  
  I_control.roll = Ki * error.roll * dt;
  I_control.pitch = Ki * error.pitch * dt;
  I_control.yaw = Ki * error.yaw * dt;

  Angle dAngle;
  dAngle.roll = (*currentAngle).roll - (*prevAngle).roll;
  dAngle.pitch = (*currentAngle).pitch - (*prevAngle).pitch;
  dAngle.yaw = (*currentAngle).yaw - (*prevAngle).yaw;
  
  D_control.roll = - Kd * (dAngle.roll/dt);
  D_control.pitch = - Kd * (dAngle.pitch/dt);
  D_control.yaw = - Kd * (dAngle.yaw/dt);

  PIDResult result;
  result.control.roll = P_control.roll + I_control.roll + D_control.roll;
  result.control.pitch = P_control.pitch + I_control.pitch + D_control.pitch;
  result.control.yaw = P_control.yaw + I_control.yaw + D_control.yaw;

  result.error = error;
  return &result;
}

/* getControllerValue
 * Get aim to controller
 */
Control* getControllerValue(){
  Control result;
  while(Serial1.available() > 0){
    char userInput = Serial1.read();
    switch(userInput){
      case 'T':
        result.throttle = Serial1.read();
        break;
      case 'R':
        result.aimAngle.roll = Serial1.read();
        break;
      case 'P':
        result.aimAngle.pitch = Serial1.read();
        break;
      case 'Y':
        result.aimAngle.yaw = Serial1.read();
        break;
      default:
        Serial1.println("Waiting for control");
    }
  }
  return &result;
}

/* MotorControl
 * Control to Motor
 * Angle* PID_value : Calculated PID value
 * double throttle : throttle value
 */
void MotorControl(Angle* PID_value, double throttle){
  if(throttle == 0){ //IF IDLE STATE
    analogWrite(MOTOR_A_PIN, 0);
    analogWrite(MOTOR_B_PIN, 0);
    analogWrite(MOTOR_C_PIN, 0);
    analogWrite(MOTOR_D_PIN, 0);
  }else{
    double motorASpeed = throttle + (*PID_value).yaw + (*PID_value).roll + (*PID_value).pitch;
    double motorBSpeed = throttle - (*PID_value).yaw - (*PID_value).roll + (*PID_value).pitch;
    double motorCSpeed = throttle + (*PID_value).yaw - (*PID_value).roll - (*PID_value).pitch;
    double motorDSpeed = throttle - (*PID_value).yaw + (*PID_value).roll - (*PID_value).pitch;

    analogWrite(MOTOR_A_PIN, constrain(motorASpeed , 0, 255));
    analogWrite(MOTOR_B_PIN, constrain(motorBSpeed , 0, 255));
    analogWrite(MOTOR_C_PIN, constrain(motorCSpeed , 0, 255));
    analogWrite(MOTOR_D_PIN, constrain(motorDSpeed , 0, 255));
  }
}

SensorResult basedSensorValue;
Angle aimAngle;
Angle prevAngle;
double throttle = 0;
double prevTime = 0;
int loopCount = 0;

void setup() {
  //Initialize
  initMPU6050();
  Serial1.begin(115200);
  basedSensorValue = *calibAngle();
  aimAngle.roll = 0;
  aimAngle.pitch = 0;
  aimAngle.yaw = 0;
  prevAngle.roll = 0;
  prevAngle.pitch = 0;
  prevAngle.yaw = 0;
}

void loop() {
  /* Flow of loop
   * 1.Get aim angle to controller
   * 2.Get Angle
   * 3.PID control calculation
   * 4.Control to motor
   */
   //calculation of increment time
   double currentTime = millis();
   double dt = currentTime - prevTime;

   //1.Get aim angle to controller
   if(Serial1.available() > 0){
    Control* result = getControllerValue();
    aimAngle = (*result).aimAngle;
    throttle = (*result).throttle;
   }
   
   //2.Get Angle
   SensorResult* sensorValue = readAngle();
   Angle* accelAngle = calcAccelAngle(sensorValue, &basedSensorValue);
   Angle* gyroAngle;
   calcGyroAngle(sensorValue, &basedSensorValue, gyroAngle, dt);
   Angle* currentAngle;
   if(loopCount == 0){ //If excute first loop
    *currentAngle = prevAngle;
   }
   calcFilteredAngle(accelAngle, gyroAngle, currentAngle, dt);

   //3.PID control calculation
   PIDResult* pidResult = PID_Calculate(&aimAngle, &prevAngle, currentAngle, dt);

   //4.Control to Motor
   MotorControl(&((*pidResult).control), throttle);
   //set previous value
   prevTime = currentTime;
   prevAngle = *currentAngle;
}
