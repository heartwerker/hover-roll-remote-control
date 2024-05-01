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
        control.range = 300;
    }
    else if (wii.c && !wii.z) // upper button only =========================
    {
        control.steer = wii_x;
        control.speed = wii_y;
        control.range = 1000;
    }
    else
    {
        control.speed = 0;
        control.steer = 0;
        control.range = 0;
    }

#if DEBUG_MAPPING
    Serial.printf("wii_x: %2.2f wii_y: %2.2f steer: %2.2f speed: %2.2f, range: %4d ----- ", wii_x, wii_y, control.steer, control.speed, control.range);
#endif

    //  covert control to msg =========================

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


#define STEER_FACTOR 1.0

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
        msg.cmd_L = (int16_t)((control.speed + control.steer * STEER_FACTOR) * control.range);
        msg.cmd_R = (int16_t)((control.speed - control.steer * STEER_FACTOR) * control.range);

        msg.cmd_L = clipf(msg.cmd_L, -control.range, control.range);
        msg.cmd_R = clipf(msg.cmd_R, -control.range, control.range);
#endif

    return msg;
}

#endif
