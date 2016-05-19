//Author : Prokuma

#include <Wire.h>

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

typedef struct PIDresult PIDResult;

const int MPU_addr = 0x68;
const double Pi = 3.1415926535;
const double GYROXYZ_TO_DEGREES_PER_SEC = 131;
double prevTime_PID = 0; //Saving previous time for PID Control

//PID Control Constants
double Kp; //P control
double Ki; //I control
double Kd; //D control

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

/* PID_Calculate
 * PID Control Calculation Function
 * Parameter:aimAnlge, prevAngle, currentAngle
 * aimAngle(Angle *):set aim angle
 * prevAngle(Angle *):set previous current angle
 * currentAngle(Angle *):set current angle
 */
PIDResult* PID_Calculate(Angle *aimAngle, Angle *prevAngle ,Angle *currentAngle) {

  double currentTime = millis();
  double dTime = currentTime - prevTime_PID;
  
  Error error;
  error.roll = (*aimAngle).roll - (*currentAngle).roll;
  error.pitch = (*aimAngle).pitch - (*currentAngle).pitch;
  error.yaw = (*aimAngle).yaw - (*currentAngle).yaw;

  Angle P_control, I_control, D_control;
  P_control.roll = Kp * error.roll;
  P_control.pitch = Kp * error.pitch;
  P_control.yaw = Kp * error.yaw;
  
  I_control.roll = Ki * error.roll * dTime;
  I_control.pitch = Ki * error.pitch * dTime;
  I_control.yaw = Ki * error.yaw * dTime;

  Angle dAngle;
  dAngle.roll = (*currentAngle).roll - (*prevAngle).roll;
  dAngle.pitch = (*currentAngle).pitch - (*prevAngle).pitch;
  dAngle.yaw = (*currentAngle).yaw - (*prevAngle).yaw;
  
  D_control.roll = - Kd * (dAngle.roll/dTime);
  D_control.pitch = - Kd * (dAngle.pitch/dTime);
  D_control.yaw = - Kd * (dAngle.yaw/dTime);

  PIDResult result;
  result.control.roll = P_control.roll + I_control.roll + D_control.roll;
  result.control.pitch = P_control.pitch + I_control.pitch + D_control.pitch;
  result.control.yaw = P_control.yaw + I_control.yaw + D_control.yaw;

  result.error = error;

  prevTime_PID = millis();
  return &result;
}

void Motor_Control(Angle PID_value, double throttle){
  //
}

SensorResult basedSensorValue;

void setup() {
  //init
  initMPU6050();
  Serial.begin(115200);
  basedSensorValue = *calibAngle();
}

void loop() {
  /* Flow of loop
   * 1.Get angle
   * 2.Get acceleration
   * 3.Get aim value to controller
   * 4.PID control calculation
   * 5.throttle calculation
   * 6.Control to motor
   */
}
