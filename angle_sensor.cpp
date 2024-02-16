#include "angle_sensor.h"
#include <math.h>

static float angleFromAcc(float x, float y, float z);

AngleSensor::AngleSensor(Adafruit_MPU6050* imu, const float alpha)
{
    _alpha = alpha;
    _filteredAngle = 0;
    _accAngle = 0;
    _gyro = 0;

    if (imu != nullptr)
    {
        _imu = imu;
        _imu->setAccelerometerRange(MPU6050_RANGE_2_G);
        _imu->setGyroRange(MPU6050_RANGE_250_DEG);
        _imu->setFilterBandwidth(MPU6050_BAND_44_HZ);
        _imu->setClock(MPU6050_PLL_GYROX);
    }
}

bool AngleSensor::Start(void)
{
    bool ret = false;

    if (_imu != nullptr)
    {
        ret = _imu->begin();

        if (ret)
        {
            sensors_event_t a, g, temp;

            _imu->getEvent(&a, &g, &temp);
            float accAngle = angleFromAcc(a.acceleration.x, a.acceleration.y, a.acceleration.z);
            _accAngle = accAngle;
            _gyro = g.gyro.x; // Initial state
        }
    }

    return ret;
}

float AngleSensor::UpdateAngle(const float timeStep)
{
    sensors_event_t a, g, temp;

    _imu->getEvent(&a, &g, &temp);
    float accAngle = angleFromAcc(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    float gyroDelta = g.gyro.x * timeStep;
    _gyro = g.gyro.x;
    _accAngle = accAngle;
    _filteredAngle = _alpha * (_filteredAngle + gyroDelta) + ((1 - _alpha) * accAngle);

    return _filteredAngle;
}

float AngleSensor::getGyro(void)
{
    return _gyro;
}

float AngleSensor::getAccAngle(void)
{
    return _accAngle;
}

static float angleFromAcc(float x, float y, float z)
{
  return atan(y / sqrt( (x * x) + (z * z)));
}