#include "angle_sensor.h"
#include <math.h>

static float angleFromAcc(float x, float y, float z);

AngleSensor::AngleSensor(Adafruit_MPU6050* imu, const float alpha)
{
    _alpha = alpha;

    if (imu != nullptr)
    {
        _imu = imu;
        _imu->setAccelerometerRange(MPU6050_RANGE_8_G);
        _imu->setGyroRange(MPU6050_RANGE_500_DEG);
        _imu->setFilterBandwidth(MPU6050_BAND_184_HZ);
    }
}

bool AngleSensor::Start(void)
{
    bool ret = false;

    if (_imu != nullptr)
    {
        ret = _imu->begin();
    }

    return ret;
}

float AngleSensor::UpdateAngle(const float timeStep)
{
    sensors_event_t a, g, temp;

    _imu->getEvent(&a, &g, &temp);
    float accAngle = angleFromAcc(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    float gyroDelta = g.gyro.x * timeStep;
    gyroAngle += gyroDelta;
    filteredAngle = _alpha * (filteredAngle + gyroDelta) + ((1 - _alpha) * accAngle);

    return filteredAngle;
}

static float angleFromAcc(float x, float y, float z)
{
  return atan(y / sqrt( (x * x) + (z * z)));
}