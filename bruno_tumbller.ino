#include <Adafruit_MPU6050.h>

#include "motor.h"
#include "pid.h"
#include "pins.h"

#define TIME_STEP 10

Adafruit_MPU6050 mpu;
Motor motor(AIN1, PWMA_LEFT, BIN1, PWMB_RIGHT, STBY_PIN);
Pid pid(5, 0.1, 1, 0);

float gyroAngle;
float filteredAngle;
unsigned speed = 0;

static float angleFromAcc(float x, float y, float z)
{
  return atan(y / sqrt( (x * x) + (z * z))) * 180 / 3.14;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
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
  delay(2000);
}

const float timeStep = (float)TIME_STEP / 1000;
const float maxAngle = 40.f;

void loop() {
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accAngle = angleFromAcc(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  float gyroDelta = g.gyro.x * timeStep  * 180 / 3.14;
  gyroAngle += gyroDelta;
  filteredAngle = 0.9 * (filteredAngle + gyroDelta) + 0.1 * accAngle;

  if( (filteredAngle > maxAngle) || (filteredAngle < -maxAngle) )
  {
    motor.stop();
    while(1) { };
  }

  float pidOutput = pid.computePid(filteredAngle, timeStep);
  float command = (abs(pidOutput) * 255) / 100;
  if(pidOutput > 0)
  {
    motor.backward(command);
  }
  else
  {
    motor.forward(command);
  }

  Serial.print("PID: ");
  Serial.print(command);
  Serial.print(",");
  #if 0
  Serial.print("Acc: ");
  Serial.print(accAngle);
  Serial.print(",");
  Serial.print("Gyro: ");
  Serial.print(gyroAngle);
  Serial.print(",");
  #endif
  Serial.print("Filtered: ");
  Serial.println(filteredAngle);

  delay(TIME_STEP);
}
