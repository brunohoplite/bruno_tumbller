#include "angle_sensor.h"
#include "motor.h"
#include "pid.h"
#include "pins.h"
#include "pid_phil.h"
#include "kalman.h"

#define FILTER_ALPHA (0.80f)
#define OUTPUT_SCALING (2.76f)
#define KP (700.f * OUTPUT_SCALING)
#define KI (250.f * OUTPUT_SCALING)
#define KD (0.1f * OUTPUT_SCALING)

#define RAD_TO_DEG_COEFF (57.3f)


unsigned speed = 0;
const float setPoint = 0; // in rad
const float maxAngle = 0.80f;

Adafruit_MPU6050 imu;
AngleSensor angleSensor(&imu, FILTER_ALPHA);
Kalman kalmanFilter;
Motor motor(AIN1, PWMA_LEFT, BIN1, PWMB_RIGHT, STBY_PIN);
PIDController pid_phil(KP, KI, KD,
                       0.f, (-255.f), 255.f,
                       -100.f, 100.f);
Pid oldPid(KP, KI, KD);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  if (!angleSensor.Start()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { }
  }

  motor.stop();
  float InitialAngle = angleSensor.getAccAngle() * RAD_TO_DEG_COEFF;
  kalmanFilter.setAngle(InitialAngle);
  delay(1000);
}

void loop() {
  
  static unsigned long lastTime = micros();
  unsigned long now = micros();
  float timeStep = (float)(now - lastTime) / 1000000.f;
  float filteredAngle = angleSensor.UpdateAngle(timeStep);
  float accAngle = angleSensor.getAccAngle() * RAD_TO_DEG_COEFF;
  float gyroRate = (angleSensor.getGyro() * RAD_TO_DEG_COEFF) - 1;
  float kalmanAngle = kalmanFilter.getAngle(accAngle, gyroRate, timeStep) / RAD_TO_DEG_COEFF;

  if( (filteredAngle > maxAngle) || (filteredAngle < -maxAngle) )
  {
    motor.stop();
    while(1) { };
  }

  // float pidOutput = pid_phil.Update(0.f, kalmanAngle, timeStep);
  float pidOutput = oldPid.computePid(kalmanAngle, -0.005f, timeStep);
  float command = abs(pidOutput);

  lastTime = now;

  if(pidOutput > 0)
  {
    motor.backward(command);
  }
  else
  {
    motor.forward(command);
  }

#if 0
  Serial.print("Kalman:");
  Serial.print(kalmanAngle);
  Serial.print(",");
  Serial.print("Acc:");
  Serial.print(angleSensor.getAccAngle());
  Serial.print(",");
  Serial.print("Gyro:");
  Serial.print((angleSensor.getGyro() * timeStep));
  Serial.print(",");
  Serial.print("Filtered:");
  Serial.println(filteredAngle);
#endif
}
