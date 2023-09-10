#pragma once
#ifndef CONTROL_MAPPING_H
#define CONTROL_MAPPING_H

// this computes wii remote to espnow message (hovercontrol cmds)

#include "espnow_protocol.h"
#include "wii_i2c.h"
#include "elapsedMillis.h"
#include "helper.h"

typedef struct struct_control
{
    float steer = 0;
    float speed = 0;
    int range = 200;
} struct_control;
struct_control control;

float acc_x_max = 1;
float acc_y_max = 1;
short int acc_y_offset = 90;
int acc_y;

elapsedMillis since_acc_control = 0;

typedef struct
{
    uint16_t activated;
    int16_t max_x;
    int16_t min_x;
} FastMode;
FastMode fastMode;

message_from_remote map_wii_control(wii_i2c_nunchuk_state wii)
{

#if DEBUG_WII_RAW
    wii_print_state(wii);
#endif

    message_from_remote msg;

    float wii_x = clipf(float(wii.x + 1) / 127.0, -1, 1);
    float wii_y = clipf(float(wii.y + 0) / 127.0, -1, 1);

    if (wii.z && !wii.c) // lower button only =========================
    {
        control.steer = wii_x;
        control.speed = wii_y;
        control.range = 500;

#if DEBUG_MAPPING
        Serial.printf("wii_x: %2.2f wii_y: %2.2f steer: %2.2f speed: %2.2f, range: %4d ----- ", wii_x, wii_y, control.steer, control.speed, control.range);
#endif
    }
    else if (wii.c && !wii.z) // upper button only =========================
    {
#if 0 // via joystick
        control.steer = wii_x;
        control.speed = wii_y;
        control.range = 500;

#if DEBUG_MAPPING
        Serial.printf("wii_x: %2.2f wii_y: %2.2f steer: %2.2f speed: %2.2f, range: %4d ----- ", wii_x, wii_y, control.steer, control.speed, control.range);
#endif

#else // via acceleration ?!
        if (fabs(wii.acc_x) > acc_x_max)
            acc_x_max = fabs(wii.acc_x);

#define MIN_RANGE 60
        if (acc_x_max > MIN_RANGE)
            acc_x_max -= 0.05;

        control.steer = wii.acc_x;
        control.steer /= acc_x_max;
        control.steer = convert_zero_zone(control.steer, 0.1);
        control.steer *= 3.0;
        control.steer = clipf(control.steer, -1, 1);

#if 0
        acc_y = wii.acc_y - acc_y_offset;
        if (fabs(acc_y) > acc_y_max)
            acc_y_max = fabs(acc_y);

        if (acc_y_max > MIN_RANGE)
            acc_y_max -= 0.05;

        control.speed = acc_y;
        control.speed /= acc_y_max;
        control.speed = convert_zero_zone(control.speed, 0.1);
#else
        control.speed = wii.y;
        control.speed /= 127;
#endif

#if 0
        float factor_startup = clipf(float(since_acc_control) / 3000.0, 0, 1);
        control.steer *= factor_startup;
        control.speed *= factor_startup;
#endif

        control.speed = wii_y;
        control.range = 1000;

        Serial.printf("wii.acc_x: %4d wii.acc_y: %4d acc_y_offset: %4d acc_y: %4d steer: %2.2f speed: %2.2ff", wii.acc_x, wii.acc_y, acc_y_offset, acc_y, control.steer, control.speed);
#endif
    }
    else if (wii.z && wii.c) // both buttons =========================
    {
        if (fastMode.activated)
        {
            control.steer = wii_x;
            control.speed = wii_y;
            control.range = 1000;

#if DEBUG_MAPPING
            Serial.printf("wii_x: %2.2f wii_y: %2.2f steer: %2.2f speed: %2.2f, range: %4d ----- ", wii_x, wii_y, control.steer, control.speed, control.range);
#endif
        }

        fastMode.max_x = wii.acc_x > fastMode.max_x ? wii.acc_x : fastMode.max_x;
        fastMode.min_x = wii.acc_x < fastMode.min_x ? wii.acc_x : fastMode.min_x;

        if ((fastMode.max_x > 100) && (fastMode.min_x < -100))
            fastMode.activated = 1;
    }

    // ========================= covert cotrol to msg =========================

    if (!wii.z && !wii.c) // no buttons =========================
    {
        // reset helpers
        fastMode.activated = false;
        fastMode.min_x = 0;
        fastMode.max_x = 0;

        since_acc_control = 0;
        acc_y_offset = wii.acc_y;
        // Serial.printf("wii.acc_x: %4d wii.acc_y: %4d acc_y_offset: %4d acc_y: %4d steer: %2.2f speed: %2.2ff", wii.acc_x, wii.acc_y, acc_y_offset, acc_y, steer, speed);
    }
    else
    {
#define STEER_FACTOR 1

#if USE_CMD_L_R
#if USE_DUAL_BOARDS
        float speed_Left = control.speed + control.steer * STEER_FACTOR * (-1);
        speed_Left = clipf(speed_Left, -1, 1) * control.range;
        msg.cmd_Left_L = speed_Left;
        msg.cmd_Left_R = speed_Left;
        
        float speed_Right = control.speed - control.steer * STEER_FACTOR * (-1);
        speed_Right = clipf(speed_Right, -1, 1) * control.range;

        msg.cmd_Right_L = speed_Right;
        msg.cmd_Right_R = speed_Right;

#else
        int16_t uSpeed_L = (int16_t)((speed + steer * STEER_FACTOR) * range);
        int16_t uSpeed_R = (int16_t)((speed - steer * STEER_FACTOR) * range);

        // Limit speed
        uSpeed_L = clipf(uSpeed_L, -range, range);
        uSpeed_R = clipf(uSpeed_R, -range, range);

        msg.cmd_L = uSpeed_L;
        msg.cmd_R = uSpeed_R;
#endif
#else
#if USE_DUAL_BOARDS
        // not intuitive dont use
#else
        // Convert to int
        int16_t uSteer = (int16_t)(steer * range);
        int16_t uSpeed = (int16_t)(speed * range);

        // Limit speed
        if (uSpeed > range)
            uSpeed = range;
        else if (uSpeed < -range)
            uSpeed = -range;

        // Send
        msg.cmd_Left_L = uSteer;
        msg.cmd_Left_R = uSpeed;
#endif
#endif
    }

    return msg;
}

#endif
