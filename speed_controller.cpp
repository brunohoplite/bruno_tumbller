

#include "speed_controller.h"
#include <MsTimer2.h>
#include <PinChangeInt.h>
#include "pins.h"

#define KP (20.f)
#define KI (1.f)

static void encoderCountRightA(void);
static void encoderCountLeftA(void);

static volatile unsigned long encoder_count_right_a = 0;
static volatile unsigned long encoder_count_left_a = 0;

SpeedController::SpeedController(void)
{
    _kp = KP;
    _ki = KI;
    speed_filter = 0;
    speed_filter_old = 0;
    car_speed_integeral = 0;
    setting_car_speed = 0;
}

SpeedController::SpeedController(float kp, float ki)
{
    _kp = kp;
    _ki = ki;
    speed_filter = 0;
    speed_filter_old = 0;
    car_speed_integeral = 0;
    setting_car_speed = 0;
}

void SpeedController::start(void)
{
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
    attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, encoderCountRightA, CHANGE);
}

float SpeedController::updateControl(void)
{
    cli();
    double car_speed = (encoder_count_right_a + encoder_count_left_a) / 2;
    encoder_count_right_a = 0;
    encoder_count_left_a = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    double speed_control_output = -_kp * speed_filter - _ki * car_speed_integeral;
    sei();

    return speed_control_output;
}

static void encoderCountRightA(void)
{
    encoder_count_right_a++;
}

static void encoderCountLeftA(void)
{
    encoder_count_left_a++;
}