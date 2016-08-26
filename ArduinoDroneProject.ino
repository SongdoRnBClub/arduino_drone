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
void readAngle(SensorResult *result){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  
  result->AcX = Wire.read() << 8 | Wire.read();
  result->AcY = Wire.read() << 8 | Wire.read();
  result->AcZ = Wire.read() << 8 | Wire.read();
  result->Tmp = Wire.read() << 8 | Wire.read();
  result->GyX = Wire.read() << 8 | Wire.read();
  result->GyY = Wire.read() << 8 | Wire.read();
  result->GyZ = Wire.read() << 8 | Wire.read();
}
/* initMPU6050
 * Initialize MPU6050 Sensor
 */
void initMPU6050(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

/* calibAngle
 * calibration Sensor
 */
void calibAngle(SensorResult *result){
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

  SensorResult cresult;
  for(int i = 0; i < 10; i++){
    readAngle(&cresult);
    sumAcX += cresult.AcX; sumAcY += cresult.AcY; sumAcZ += cresult.AcZ;
    sumGyX += cresult.GyX; sumGyY += cresult.GyY; sumGyZ += cresult.GyZ;
    delay(100);
  }

  result->AcX = (int16_t)sumAcX / 10;
  result->AcY = (int16_t)sumAcY / 10;
  result->AcZ = (int16_t)sumAcZ / 10;
  result->GyX = (int16_t)sumGyX / 10;
  result->GyY = (int16_t)sumGyY / 10;
  result->GyZ = (int16_t)sumGyZ / 10;
  
  result->Tmp = cresult.Tmp;
}

/* calcAccelAngle
 * SensorResult* sensor : Current Sensor Value
 * SensorResult* basedSenserResult : Based Sensor Value
 */
void calcAccelAngle(SensorResult* sensor, SensorResult* basedSensorResult, Angle* result){
  SensorResult accel;

  accel.AcX = sensor->AcX - basedSensorResult->AcX;
  accel.AcY = sensor->AcX - basedSensorResult->AcY;
  accel.AcZ = sensor->AcZ + (16384 - basedSensorResult->AcZ);

  result->roll = atan(accel.AcY/sqrt(pow(accel.AcX, 2) + pow(accel.AcZ, 2))) * (180/Pi);
  result->pitch = atan(-accel.AcX/sqrt(pow(accel.AcY, 2) + pow(accel.AcZ, 2))) * (180/Pi);
  result->yaw = 0;

}

/* calcGyroAngle
 * SensorResult* sensor : Current Sensor Value
 * SensorResult* basedSenserResult : Based Sensor Value
 * Angle* result : Result Angle
 * double dt : Time increment
 */
void calcGyroAngle(SensorResult *sensor, SensorResult *basedSensorResult,Angle* result,SensorResult* dGyro, double dt){
  //SensorResult gyro;

  dGyro->GyX = (sensor->GyX - basedSensorResult->GyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  dGyro->GyY = (sensor->GyY - basedSensorResult->GyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  dGyro->GyZ = (sensor->GyZ - basedSensorResult->GyZ) / GYROXYZ_TO_DEGREES_PER_SEC;

  result->roll += (dGyro->GyX * dt);
  result->pitch += (dGyro->GyY * dt);
  result->yaw += (dGyro->GyZ * dt);

}

/* calcFilteredAngle
 * Calculate Combination Angle Filter Funtion
 * Angle* accel : accelorator Angle
 * Angle* gyro : gyroscope Angle
 * Angle* result : Result Angle
 * double dt : Time increment
 */
void calcFilteredAngle(Angle* accel, SensorResult* gyro, Angle* result, double dt){
  const double ALPHA = 0.96;
  Angle tmp;

  tmp.roll = result->roll + (gyro->GyX * dt);
  tmp.pitch = result->pitch + (gyro->GyY * dt);
  tmp.yaw = result->yaw + (gyro->GyZ * dt);

  result->roll = (ALPHA * tmp.roll) + ((1.0 - ALPHA) * accel->roll);
  result->pitch = (ALPHA * tmp.pitch) + ((1.0 - ALPHA) * accel->pitch);
  result->yaw = tmp.yaw;
}

/* PID_Calculate
 * PID Control Calculation Function
 * Parameter:aimAnlge, prevAngle, currentAngle, dt, prev_I_controll, result
 * aimAngle(Angle *):set aim angle
 * prevAngle(Angle *):set previous current angle
 * currentAngle(Angle *):set current angle
 * double dt : Time increment
 * prev_I_control(Angle *) : Previous Integral Calculated Number
 * result(PIDResult *) : Result of PID Calculation
 */
void PID_Calculate(Angle *aimAngle, Angle *prevAngle ,Angle *currentAngle, double dt, Angle *prev_I_control, PIDResult* result) {
  
  Error error;
  error.roll = aimAngle->roll - currentAngle->roll;
  error.pitch = aimAngle->pitch - currentAngle->pitch;
  error.yaw = aimAngle->yaw - currentAngle->yaw;

  Angle P_control, D_control;
  P_control.roll = Kp * error.roll;
  P_control.pitch = Kp * error.pitch;
  P_control.yaw = Kp * error.yaw;
  
  prev_I_control->roll += Ki * error.roll * dt;
  prev_I_control->pitch += Ki * error.pitch * dt;
  prev_I_control->yaw += Ki * error.yaw * dt;

  Angle dAngle;
  dAngle.roll = currentAngle->roll - prevAngle->roll;
  dAngle.pitch = currentAngle->pitch - prevAngle->pitch;
  dAngle.yaw = currentAngle->yaw - prevAngle->yaw;
  
  D_control.roll = - Kd * (dAngle.roll/dt);
  D_control.pitch = - Kd * (dAngle.pitch/dt);
  D_control.yaw = - Kd * (dAngle.yaw/dt);
  
  (*result).control.roll = P_control.roll + prev_I_control->roll + D_control.roll;
  (*result).control.pitch = P_control.pitch + prev_I_control->pitch + D_control.pitch;
  (*result).control.yaw = P_control.yaw + prev_I_control->yaw + D_control.yaw;

  (*result).error = error;
}

/* Daulloop_PID_Calculate
 * Dual Loop PID Control Calculation Function
 * Parameter:aimAnlge, prevAngle, currentAngle, dt, currentRate, prev_stabilize_I_controll, prev_rate_I_controll, result
 * aimAngle(Angle *):set aim angle
 * prevAngle(Angle *):set previous current angle
 * currentAngle(Angle *):set current angle
 * double dt : Time increment
 * currentRate(SensorResult *) : CurrentRate to Gyro Sensor
 * prev_stabilize_I_controll(Angle *) : Previous Static Integral Calculated Number for stabilize
 * prev_rate_I_controll(Angle *) : Previous Static Integral Calculated Number for rate
 * result(PIDResult *) : Result of PID Calculation
 */
void Dualloop_PID_Calculate(Angle *aimAngle, Angle *prevAngle ,Angle *currentAngle, double dt,SensorResult *currentRate, Angle *prev_stabilize_I_control,Angle *prev_rate_I_control, PIDResult* result) {
  Error error;
  error.roll = aimAngle->roll - currentAngle->roll;
  error.pitch = aimAngle->pitch - currentAngle->pitch;
  error.yaw = aimAngle->yaw - currentAngle->pitch;

  Angle stabilize_P_control;
  stabilize_P_control.roll = Kp * error.roll;
  stabilize_P_control.pitch = Kp * error.pitch;
  stabilize_P_control.yaw = Kp * error.yaw;

  prev_stabilize_I_control->roll += Ki * error.roll * dt;
  prev_stabilize_I_control->pitch += Ki * error.pitch * dt;
  prev_stabilize_I_control->yaw += Ki * error.yaw * dt;

  Error rateError;
  rateError.roll = stabilize_P_control.roll - (currentRate->GyX / GYROXYZ_TO_DEGREES_PER_SEC);
  rateError.pitch = stabilize_P_control.pitch - (currentRate->GyY / GYROXYZ_TO_DEGREES_PER_SEC);
  rateError.yaw = stabilize_P_control.yaw - (currentRate->GyZ / GYROXYZ_TO_DEGREES_PER_SEC);

  Angle rate_P_control;
  rate_P_control.roll = Kp * rateError.roll;
  rate_P_control.pitch = Kp * rateError.pitch;
  rate_P_control.yaw = Kp * rateError.yaw;

  prev_rate_I_control->roll += Ki * rateError.roll * dt;
  prev_rate_I_control->pitch += Ki * rateError.pitch * dt;
  prev_rate_I_control->yaw += Ki * rateError.yaw * dt;

  result->control.roll = rate_P_control.roll + prev_rate_I_control->roll + prev_stabilize_I_control->roll;
  result->control.pitch = rate_P_control.pitch + prev_rate_I_control->pitch + prev_stabilize_I_control->pitch;
  result->control.yaw = rate_P_control.yaw + prev_rate_I_control->yaw + prev_stabilize_I_control->yaw;

  result->error = error;
}

/* getControllerValue
 * Get aim to controller
 */
void getControllerValue(Control *result){
  bool isReturnedValue = false;
  while(Serial1.available() > 0){
    char userInput = Serial1.read();
    switch(userInput){
      case 'T':
        result->throttle = (int)Serial1.read();
        isReturnedValue = true;
        break;
      case 'R':
        result->aimAngle.roll = (double)Serial1.read() - 50;
        isReturnedValue = true;
        break;
      case 'P':
        result->aimAngle.pitch = (double)Serial1.read() - 50;
        isReturnedValue = true;
        break;
      case 'Y':
        result->aimAngle.yaw = (double)Serial1.read() - 50;
        isReturnedValue = true;
        break;
    }
  }
  if(!isReturnedValue){
    result->throttle = 0;
    result->aimAngle.roll = 0;
    result->aimAngle.pitch = 0;
    result->aimAngle.yaw = 0;
  }
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
    double motorASpeed = throttle + PID_value->yaw + PID_value->roll + PID_value->pitch;
    double motorBSpeed = throttle - PID_value->yaw - PID_value->roll + PID_value->pitch;
    double motorCSpeed = throttle + PID_value->yaw - PID_value->roll - PID_value->pitch;
    double motorDSpeed = throttle - PID_value->yaw + PID_value->roll - PID_value->pitch;

    analogWrite(MOTOR_A_PIN, constrain((int)motorASpeed , 0, 255));
    analogWrite(MOTOR_B_PIN, constrain((int)motorBSpeed , 0, 255));
    analogWrite(MOTOR_C_PIN, constrain((int)motorCSpeed , 0, 255));
    analogWrite(MOTOR_D_PIN, constrain((int)motorDSpeed , 0, 255));
  }
}

void initAngle(Angle* angle){
  angle->roll = 0;
  angle->pitch = 0;
  angle->yaw = 0;
}

void initSensor(SensorResult* sensor){
  sensor->AcX = 0;
  sensor->AcY = 0;
  sensor->AcZ = 0;
  sensor->GyX = 0;
  sensor->GyY = 0;
  sensor->GyZ = 0;
}

SensorResult basedSensorValue;
Angle aimAngle;
Angle prevAngle;
Angle gyroAngle;
Angle accelAngle;
Angle currentAngle;
Angle prevIcontroll;
Angle prevStabilizeIcontrol;
Angle prevRateIcontrol;
SensorResult dGyro;
int throttleInput = 0;
double prevTime = 0;
int loopCount = 0;

void setup() {
  //Initialize
  initMPU6050();
  Serial1.begin(115200);
  initSensor(&basedSensorValue);
  calibAngle(&basedSensorValue);
  initAngle(&aimAngle);
  initAngle(&prevAngle);
  initAngle(&gyroAngle);
  initAngle(&accelAngle);
  initAngle(&currentAngle);
  initAngle(&prevIcontroll);
  initAngle(&prevStabilizeIcontrol);
  initAngle(&prevRateIcontrol);
  initSensor(&dGyro);
}

void loop() {
  /* Flow of loop
   * 1.Get aim angle to controller
   * 2.Get Angle
   * 3.PID control calculation
   * 4.Control to motor
   */
   double currentTime = millis();
   double dt = (currentTime - prevTime) / 1000;

   //1.Get aim angle to controller
   if(Serial1.available() > 0){
      Control result; 
      getControllerValue(&result);
      aimAngle = result.aimAngle;
      throttleInput = result.throttle;
   }
   
   //2.Get Angle
   SensorResult sensorValue;
   readAngle(&sensorValue);

   //calculation of increment time
   calcAccelAngle(&sensorValue, &basedSensorValue, &accelAngle);
   calcGyroAngle(&sensorValue, &basedSensorValue, &gyroAngle, &dGyro, dt);
   if(loopCount == 0){ //If excute first loop
     currentAngle = prevAngle;
   }
   calcFilteredAngle(&accelAngle, &dGyro, &currentAngle, dt);

   //3.PID controll calculation
   PIDResult pidResult;
   //If you want standard PID controll
   PID_Calculate(&aimAngle, &prevAngle, &currentAngle, dt,&prevIcontroll, &pidResult);
   //If you want dual loop PID controll
   //Dualloop_PID_Calculate(&aimAngle, &prevAngle, &currentAngle, dt, &sensorValue, &prevStabilizeIcontrol, &prevRateIcontrol, &pidResult);
  
   //4.Control to Motor
   MotorControl(&(pidResult.control), (double)throttleInput);
   //set previous value
   prevTime = currentTime;
   prevAngle = currentAngle;
   loopCount++;
}
