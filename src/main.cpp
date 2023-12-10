#include <Arduino.h>
#include "config.h"
#include "ui.h"
#include <elapsedMillis.h>
#include "control_mapping.h"

#if USE_ESPNOW
#include "espnow_protocol.h"
void ESPNOW_receiveBytes(uint8_t *data, uint8_t len);
elapsedMillis since_received = 0;
#endif

#if USE_WIFI
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#endif

#if IS_REMOTE
#if USE_WII_NUNCHUCK
// https://github.com/moefh/esp32-wii-nunchuk
#include "wii_i2c.h"
// pins connected to the Nunchuk:
#define PIN_SDA D2
#define PIN_SCL D1
#endif
#endif

#if USE_HOVER_SERIAL
#include "hover_serial.h"
HoverSerial hover_Left(D1, D2);
HoverSerial hover_Right(D5, D6);
#endif

#if USE_ESC
#include "ESC_60V_Driver.h"
#endif

// ########################## SETUP ##########################
void setup()
{
    Serial.begin(115200);
    printf("\n\n=== Starting Program === Booting\n");

#if USE_ESPNOW
    ESPNOW_Init(ESPNOW_receiveBytes);
#endif

#if USE_WIFI
    WiFi.mode(WIFI_STA);

    Serial.println(WiFi.macAddress());
    delay(5000);

    WiFiManager wm; // wm.setDebugOutput(false);
                    // wm.resetSettings(); // reset settings

    if (!wm.autoConnect("AP: wemos-ha-base"))
        Serial.println("Failed to connect WIFI :-/");
    Serial.println("Connected to WIFI :-)");

    ui_indicate_wifi_connected();
#endif

#if IS_REMOTE
#if USE_WII_NUNCHUCK
    if (!wii_i2c_init(PIN_SDA, PIN_SCL))
    {
        Serial.printf("Error initializing nunchuk :(");
    }
    else
    {
        Serial.printf("initialized nunchuk :(");
    }
    delay(500);
    wii_i2c_request_state();
#endif
#endif

#if IS_RECEIVER
#if USE_HOVER_SERIAL
    hover_Left.init();
    hover_Right.init();
#endif
#endif
#if USE_ESC
    ESC_60V_Driver_setup();
#endif
}

void setSpeedMotors(float left_L, float left_R, float right_L, float right_R)
{
#if USE_HOVER_SERIAL
    hover_Left.send(left_L, left_R);
    hover_Right.send(right_L, right_R);
#endif
#if USE_ESC
    ESC_60V_DRIVER_setSpeed(left_L / 1000.f, left_R / 1000.f);
#endif
}
// ########################## RX callback ##########################
void ESPNOW_receiveBytes(uint8_t *data, uint8_t len)
{
    since_received = 0;

#if IS_REMOTE
    memcpy(&msg_from_receiver, data, len);
    msg_from_receiver.speed_Left_L = INVERT_SPEED_LEFT_L ? -msg_from_receiver.speed_Left_L : msg_from_receiver.speed_Left_L;
    msg_from_receiver.speed_Left_R = INVERT_SPEED_LEFT_R ? -msg_from_receiver.speed_Left_R : msg_from_receiver.speed_Left_R;
    msg_from_receiver.speed_Right_L = INVERT_SPEED_RIGHT_L ? -msg_from_receiver.speed_Right_L : msg_from_receiver.speed_Right_L;
    msg_from_receiver.speed_Right_R = INVERT_SPEED_RIGHT_R ? -msg_from_receiver.speed_Right_R : msg_from_receiver.speed_Right_R;

#else // IS_RECEIVER
    memcpy(&msg_from_remote, data, len);

#if USE_DUAL_BOARDS
    setSpeedMotors(
        msg_from_remote.cmd_Left_L,
        msg_from_remote.cmd_Left_R,
        msg_from_remote.cmd_Right_L,
        msg_from_remote.cmd_Right_R);
#else
    setSpeedMotors(
        msg_from_remote.cmd_L,
        msg_from_remote.cmd_R,
        0, 0);
#endif
#endif
}

// ########################## LOOP ##########################
elapsedMillis since_update = 0;
void loop()
{
#if IS_REMOTE

    if (since_update > 20)
    {
        since_update = 0;

#if USE_WII_NUNCHUCK
        const unsigned char *data = wii_i2c_read_state();
        wii_i2c_request_state();
        if (!data)
        {
            Serial.printf("no wii data available :( \n");
        }
        else
        {
#if 0
            Serial.print("data: ");
            for (int i = 0; i < 6; i++)
                Serial.printf("0x%02X ", data[i]);
            Serial.println();
#endif

            wii_i2c_nunchuk_state state;
            wii_i2c_decode_nunchuk(data, &state);

#if USE_ESPNOW
            message_from_remote msg = map_wii_control(state);

#if DEBUG_TX
#if DEBUG_FOR_PLOTTER
            Serial.printf("%4d,%4d,%4d,%4d,", msg.cmd_Left_L, msg.cmd_Left_R, msg.cmd_Right_L, msg.cmd_Right_R);
#else
            Serial.printf("CMDs Left_LR: %4d,%4d, Right_LR: %4d,%4d  |", msg.cmd_Left_L, msg.cmd_Left_R, msg.cmd_Right_L, msg.cmd_Right_R);
#endif
#endif
            ESPNOW_sendMessage(&msg);

            // if (msg.cmd_Left_L != 0 || msg.cmd_Left_R != 0 || msg.cmd_Right_L != 0 || msg.cmd_Right_R != 0)
            //   ESPNOW_sendBytes((uint8_t *)&msg, sizeof(msg));
#endif
        }
#endif

#if DEBUG_RX
#if DEBUG_FOR_PLOTTER
        Serial.printf("%4d,%4d,%4d,%4d,", msg_from_receiver.speed_Left_L, msg_from_receiver.speed_Left_R, msg_from_receiver.speed_Right_L, msg_from_receiver.speed_Right_R);
#else
        //  print all 4 msg.speed values in one row for plotting with comma separated:
        Serial.printf("Speeds Left_LR %4d,%4d, Right_LR %4d,%4d |", msg_from_receiver.speed_Left_L, msg_from_receiver.speed_Left_R, msg_from_receiver.speed_Right_L, msg_from_receiver.speed_Right_R);
#endif
#endif

#if DEBUG_TX || DEBUG_RX || DEBUG_MAPPING
        Serial.println();
#endif
    }

#else // loop() IS_RECEIVER
    // ===========================================================================

#if USE_HOVER_SERIAL
    hover_Left.loop_receive();
    hover_Right.loop_receive();
#endif
#if USE_ESC
    ESC_60V_Driver_loop();
#endif

#if USE_ESC && USE_SERIAL_MONITOR_CONTROL
    if (Serial.available())
    {
        float percent = clipf(Serial.parseInt(), 0, 9);
        percent /= 9.f;
        Serial.printf("Read percent: %f\n", percent);
        // percent *= 0.2;

        setSpeedMotors(
            percent,
            percent,
            percent,
            percent);
    }
#endif

    if (since_update > 50)
    {
        since_update = 0;

#if USE_HOVER_SERIAL
#if USE_ESPNOW
        message_to_remote msg;
        HoverSerialFeedback state;

        state = hover_Left.getFeedback();
        msg.speed_Left_L = state.speedL_meas;
        msg.speed_Left_R = state.speedR_meas;

        state = hover_Right.getFeedback();
        msg.speed_Right_L = state.speedL_meas;
        msg.speed_Right_R = state.speedR_meas;

#if DEBUG_TX
        Serial.printf("%d,%d,%d,%d", msg.speed_Left_L, msg.speed_Left_R, msg.speed_Right_L, msg.speed_Right_R);
#endif
        ESPNOW_sendBytes((uint8_t *)&msg, sizeof(msg));
#endif
#endif

        // safety off if no new values are received
        if (since_received > 1000)
        {
            since_received = 900;

            setSpeedMotors(0, 0, 0, 0);

            digitalWrite(LED_BUILTIN, (millis() % 500) < 250);
        }
        else
        {
            digitalWrite(LED_BUILTIN, (millis() % 2000) < 1000);
        }

#if DEBUG_RX
#if USE_DUAL_BOARDS
    //  print all 4 msg.speed values in one row for plotting with comma separated:
    Serial.printf("%d,%d,%d,%d", msg_from_remote.cmd_Left_L, msg_from_remote.cmd_Left_R, msg_from_remote.cmd_Right_L, msg_from_remote.cmd_Right_R);
#else
    Serial.printf("%4d   %4d | ", msg_from_remote.cmd_L, msg_from_remote.cmd_R);
#endif
#endif

#if DEBUG_TX || DEBUG_RX || DEBUG_SERIAL_RECEIVE
    Serial.println();
#endif
    }
#endif
}