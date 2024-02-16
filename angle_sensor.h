

#include <Adafruit_MPU6050.h>


class AngleSensor
{
public:
    AngleSensor(Adafruit_MPU6050* imu, const float alpha);
    bool Start(void);
    float UpdateAngle(const float timeStep);
    float getGyroAngle(void);
    float getAccAngle(void);
private:
    Adafruit_MPU6050* _imu;
    float _alpha;
    float _accAngle;
    float _gyroAngle;
    float _filteredAngle;
};