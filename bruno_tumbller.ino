#include "angle_sensor.h"
#include "motor.h"
#include "pid.h"
#include "pins.h"
#include "pid_phil.h"
#include "kalman.h"
#include "speed_controller.h"

#define FILTER_ALPHA (0.9f)
#define KP (55.f)
#define KI (0.f)
#define KD (0.5f)

#define RAD_TO_DEG_COEFF (57.3f)


unsigned speed = 0;
const float setPoint = 0; // in rad
const float maxAngle = 45.f;

Adafruit_MPU6050 imu;
AngleSensor angleSensor(&imu, FILTER_ALPHA);
Kalman kalmanFilter;
Motor motor(AIN1, PWMA_LEFT, BIN1, PWMB_RIGHT, STBY_PIN);
PIDController pid_phil(KP, KI, KD,
                       0.f, (-255.f), 255.f,
                       -100.f, 100.f);
Pid oldPid(KP, KI, KD);
SpeedController speedController;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  if (!angleSensor.Start()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { }
  }

  motor.stop();
  speedController.start();
  float InitialAngle = angleSensor.getAccAngle() * RAD_TO_DEG_COEFF;
  kalmanFilter.setAngle(InitialAngle);
  delay(1000);
}
#define SPEED_DELAY (10U)
void loop() {
  
  static unsigned long lastTime = micros();
  static unsigned speedDelay = 0;
  static float speedOutput = 0;

  unsigned long now = micros();
  unsigned long diff = now - lastTime;
  lastTime = now;
  float timeStep = (float)(diff) / 1000000.f;
  float filteredAngle = angleSensor.UpdateAngle(timeStep) * RAD_TO_DEG_COEFF;
  float accAngle = angleSensor.getAccAngle() * RAD_TO_DEG_COEFF;
  float gyroRate = (angleSensor.getGyro() * RAD_TO_DEG_COEFF);
  float kalmanAngle = kalmanFilter.getAngle(accAngle, gyroRate, timeStep);

  if( (kalmanAngle > maxAngle) || (kalmanAngle < -maxAngle) )
  {
    motor.stop();
    while(1) { };
  }

  speedDelay++;
  if (speedDelay >= SPEED_DELAY)
  {
    speedDelay = 0;
    speedOutput = speedController.updateControl();
  }

#if 0
  float pidOutput = pid_phil.Update(0.f, kalmanAngle, timeStep);
#else
  float pidOutput = oldPid.computePid(kalmanAngle, -0.75f, timeStep);
#endif
  float totalOutput = pidOutput - speedOutput;
  float command = abs(pidOutput);


#if 1
  if(pidOutput > 0)
  {
    motor.backward(command);
  }
  else
  {
    motor.forward(command);
  }
#else
  static float gyroAngle = 0;
  gyroAngle += (gyroRate * timeStep);
  Serial.print("Time Step:");
  Serial.print(timeStep);
  Serial.print(",");
  Serial.print("Kalman:");
  Serial.print(kalmanAngle);
  Serial.print(",");
  Serial.print("Acc:");
  Serial.print(accAngle);
  Serial.print(",");
  Serial.print("Gyro:");
  Serial.print(gyroAngle);
  Serial.print(",");
  Serial.print("Filtered:");
  Serial.println(filteredAngle);
#endif
}
