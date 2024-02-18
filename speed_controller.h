
#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

class SpeedController
{
public:
    SpeedController();
    SpeedController(float kp, float ki);
    void start(void);
    float updateControl(void);
private:
    float _kp;
    float _ki;
    float speed_filter;
    float speed_filter_old;
    float car_speed_integeral;
    float setting_car_speed;
};

#endif