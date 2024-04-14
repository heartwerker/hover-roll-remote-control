#pragma once
#ifndef ESC_60V_DRIVER_H
#define ESC_60V_DRIVER_H

#include <Arduino.h>
#include <elapsedMillis.h>

class ESC_60V_Driver
{
public:
    ESC_60V_Driver(uint8_t pwm_pin, uint8_t dir_pin, uint8_t speed_pin)
    {
        control_pwm_pin = pwm_pin;
        control_dir_pin = dir_pin;
        feedback_speed_pin = speed_pin;
    }

    void begin()
    {
        pinMode(feedback_speed_pin, INPUT_PULLUP);
        pinMode(control_pwm_pin, OUTPUT);
        pinMode(control_dir_pin, OUTPUT);
        analogWriteFreq(2000);
        analogWriteRange(1024);
    }

    void computeSpeed()
    {
        if (since_count > 1000)
        {
            // compute speed as counts / since_count and print result
            speed += ((float(counter_speed_pulse) / float(since_count)) - speed) * 0.2f;
            since_count = 0;
            counter_speed_pulse = 0;

            // Serial.printf("speed: %f\n", speed);
        }
    }

    void loop()
    {
        if (since_loop > 5)
        {
            since_loop = 0;
            computeSpeed();
            applySpeed();

#if 0
            if (since_reverse > 5000)
            {
                since_reverse = 0;
                setSpeed(-target);
            }
#endif
        }
    }

    void setSpeed(float percentage)
    {
        percentage = constrain(percentage, -1.0f, 1.0f);
        target = percentage;
    }
    void applySpeed()
    {
        current += (target - current) * 0.0201f;

        bool dir_output = current >= 0;
        if (invert_dir)
            dir_output = !dir_output;
        digitalWrite(control_dir_pin, dir_output);

        int pwm = fabs(current) * 1023;
        analogWrite(control_pwm_pin, pwm);

        if (since_update > 50)
        {
            since_update = 0;
            Serial.printf(" (%d) - dir: %d  pwm: %4d  | ", index, dir_output, pwm);
        }
    }

public:
    int index = 0;
    elapsedMillis since_update = 0;

    uint8_t feedback_speed_pin;
    int counter_speed_pulse = 0;

    bool invert_dir = false;

private:
    elapsedMillis since_loop = 0;
    elapsedMillis since_count = 0;
    elapsedMillis since_reverse = 0;
    float speed = 0;
    float target = 0;
    float current = 0;

    uint8_t control_pwm_pin;
    uint8_t control_dir_pin;
};

#endif // ESC_60V_DRIVER_H
//pwm, dir, speed
ESC_60V_Driver ESC_Right(D1, D3, D2);
ESC_60V_Driver ESC_Left(D5, D7, D6);

void ICACHE_RAM_ATTR ISR_Left()
{
    static bool prev_state = false;
    bool state = digitalRead(ESC_Left.feedback_speed_pin);
    if (state != prev_state)
    {
        ESC_Left.counter_speed_pulse++;
        prev_state = state;
    }
}

void ICACHE_RAM_ATTR ISR_Right()
{
    static bool prev_state = false;
    bool state = digitalRead(ESC_Right.feedback_speed_pin);
    if (state != prev_state)
    {
        ESC_Right.counter_speed_pulse++;
        prev_state = state;
    }
}

void ESC_60V_Driver_setup()
{
    ESC_Left.begin();
    ESC_Left.index = 0;
    ESC_Right.begin();
    ESC_Right.index = 1;
    ESC_Right.invert_dir = true;
    // attachInterrupt(digitalPinToInterrupt(ESC_Left.feedback_speed_pin), ISR_Left, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(ESC_Right.feedback_speed_pin), ISR_Right, CHANGE);
}

void ESC_60V_DRIVER_setSpeed(float left, float right)
{
    ESC_Left.setSpeed(left);
    ESC_Right.setSpeed(right);
}

void ESC_60V_Driver_loop()
{
    ESC_Left.loop();
    ESC_Right.loop();
}