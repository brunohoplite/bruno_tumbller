#include <Adafruit_MPU6050.h>

#define TIME_STEP 10

Adafruit_MPU6050 mpu;
float gyroAngle;
float filteredAngle;

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
  delay(TIME_STEP);
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accAngle = angleFromAcc(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  float gyroDelta = g.gyro.x * ((float)TIME_STEP / 1000)  * 180 / 3.14;
  gyroAngle += gyroDelta;
  filteredAngle = 0.9 * (filteredAngle + gyroDelta) + 0.1 * accAngle;
#if 1
  Serial.print("Acc: ");
  Serial.print(accAngle);
  Serial.print(",");
#endif
  Serial.print("Gyro: ");
  Serial.print(gyroAngle);
  Serial.print(",");
  Serial.print("Filtered: ");
  Serial.println(filteredAngle);

  delay(TIME_STEP);

#if 0
  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
#endif
}
