#include "avr_servo.hpp"

#include <Arduino.h>

AVRServo::AVRServo() : Adafruit_PWMServoDriver()
{}

void AVRServo::open_servo(uint8_t servo)
{
    writeMicroseconds(servo, servo_max[servo]);
}

void AVRServo::close_servo(uint8_t servo)
{
    writeMicroseconds(servo, servo_min[servo]);
}

void AVRServo::set_servo_percent(uint8_t servo, uint8_t percent)
{
    if (percent > 100)
        percent = 100;

    uint16_t micro = map(percent, 0, 100, servo_min[servo], servo_max[servo]);

    writeMicroseconds(servo, micro);
}

void AVRServo::set_servo_absolute(uint8_t servo, uint16_t absolute)
{
    if (absolute < servo_min[servo])
        absolute = servo_min[servo];
    if (absolute < servo_min[servo])
        absolute = servo_max[servo];
    writeMicroseconds(servo, absolute);
}

void AVRServo::set_servo_min(uint8_t servo, uint16_t min)
{
    servo_min[servo] = min;
}

void AVRServo::set_servo_max(uint8_t servo, uint16_t max)
{
    servo_max[servo] = max;
}

uint8_t AVRServo::check_controller(void)
{
    int res = (int)readPrescale();

    if (res != 0)
        return 1;
    else
        return 0;
}