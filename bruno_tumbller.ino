#include <Adafruit_MPU6050.h>

#include "motor.h"
#include "pid.h"
#include "pins.h"
#include "pid_phil.h"

#define TIME_STEP (5)

#define OUTPUT_SCALING (2.76f)
#define KP (900.f * OUTPUT_SCALING)
#define KI (50.f * OUTPUT_SCALING)
#define KD (200.f * OUTPUT_SCALING)


float gyroAngle;
float filteredAngle;
unsigned speed = 0;
const float setPoint = 0; // in rad
const float timeStep = ((float)TIME_STEP / 1000.f);
const float maxAngle = 0.70f;
//const float setPointFixRate = 0.5f;

Adafruit_MPU6050 mpu;
Motor motor(AIN1, PWMA_LEFT, BIN1, PWMB_RIGHT, STBY_PIN);
Pid pid(550, 200, 10);
PIDController pid_phil(KP, KI, KD,
                       0.f, (-255.f), 255.f,
                       0.f, 0.f, timeStep);

static float angleFromAcc(float x, float y, float z)
{
  return atan(y / sqrt( (x * x) + (z * z)));
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  //Serial.println("Starting MPU6050 test!");

  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //Serial.println("Init success!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyroAngle = angleFromAcc(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  filteredAngle = gyroAngle;
  motor.stop();
  delay(2000);
}

void loop() {
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accAngle = angleFromAcc(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  float gyroDelta = g.gyro.x * timeStep;
  gyroAngle += gyroDelta;
  filteredAngle = 0.9 * (filteredAngle + gyroDelta) + 0.1 * accAngle;

  if( (filteredAngle > maxAngle) || (filteredAngle < -maxAngle) )
  {
    motor.stop();
    while(1) { };
  }
  

  //float pidOutput = pid.computePid(filteredAngle, setPoint, timeStep);
  float pidOutput = pid_phil.Update(0.f, filteredAngle);
  float command = abs(pidOutput);
  // if(command > 255)
  // {
  //   command = 255;
  // }

  if(pidOutput > 0)
  {
    motor.backward(command);
  }
  else
  {
    motor.forward(command);
  }

  Serial.print("Command:");
  Serial.print(pidOutput);
  Serial.print(",");
  #if 0
  Serial.print("Acc:");
  Serial.print(accAngle);
  Serial.print(",");
  Serial.print("Gyro:");
  Serial.print(gyroAngle);
  Serial.print(",");
  #endif
  Serial.print("Filtered:");
  Serial.println(filteredAngle);

  delay(TIME_STEP);
}
