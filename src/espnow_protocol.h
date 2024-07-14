#pragma once
#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <ESP8266WiFi.h>
#include <espnow.h>
#include "config.h"

// ========================= PROTOCOL =========================
typedef struct message_from_remote
{
#if USE_CMD_L_R
#if USE_DUAL_BOARDS
    int16_t cmd_Left_L=0;
    int16_t cmd_Left_R=0;;
    int16_t cmd_Right_L=0;
    int16_t cmd_Right_R=0;
#else
    int16_t cmd_L;
    int16_t cmd_R;
#endif
#endif
} message_from_remote;

    message_from_remote msg_from_remote;

typedef struct message_to_remote
{
#if USE_CMD_L_R
#if USE_DUAL_BOARDS
    int16_t speed_Left_L;
    int16_t speed_Left_R;
    int16_t speed_Right_L;
    int16_t speed_Right_R;
#else
    int16_t speed_L;
    int16_t speed_R;
#endif
#endif
} message_to_remote;

    message_to_remote msg_from_receiver;

// uint8_t *address_target = nullptr;
uint8_t *addr_L = nullptr;
uint8_t *addr_R = nullptr;


//================================================================
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
#if 0
    if (sendStatus == 0)
        Serial.print("Delivery success  - ");
    else
        Serial.print("Delivery fail     - ");
#endif
}

typedef void (*ESPNOW_RX_data_callback)(uint8_t *data, uint8_t len);
ESPNOW_RX_data_callback receiveBytes = nullptr;

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
    if (receiveBytes != nullptr)
        receiveBytes(incomingData, len);
}

void ESPNOW_Init(ESPNOW_RX_data_callback callback)
{
    receiveBytes = callback;

#if IS_REMOTE
    addr_L = MAC_L;
    addr_R = MAC_R;
#else
    addr_L = MAC_ADDRESS_REMOTE;
#endif

#if 1
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
#endif

    if (esp_now_init() != 0)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Set ESP-NOW Role
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    esp_now_add_peer(addr_L, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    esp_now_add_peer(addr_R, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
}

void ESPNOW_sendBytes(uint8_t *data, uint8_t len)
{
    if (addr_L != nullptr)
    {
        esp_now_send(addr_L, data, len);
        esp_now_send(addr_R, data, len);
    }
}

void ESPNOW_sendMessage(message_to_remote *msg)
{
    message_to_remote message;
    memcpy(&message, msg, sizeof(message_to_remote));

    uint8_t *data = (uint8_t *)&message;
    ESPNOW_sendBytes(data, sizeof(message_to_remote));
}

void ESPNOW_sendMessage(message_from_remote *msg)
{
    if (addr_L != nullptr)
    {
        message_from_remote msg_Left;

        msg_Left.cmd_L = msg->cmd_L * (INVERT_CMD_LEFT_L ? -1 : 1);
        msg_Left.cmd_R = msg->cmd_L * (INVERT_CMD_LEFT_R ? -1 : 1);

        uint8_t *data = (uint8_t *)&msg_Left;
        esp_now_send(addr_R, data, sizeof(message_from_remote));
    }
    if (addr_R != nullptr)
    {
        message_from_remote msg_Right;

        msg_Right.cmd_L = msg->cmd_R * (INVERT_CMD_RIGHT_L ? -1 : 1);
        msg_Right.cmd_R = msg->cmd_R * (INVERT_CMD_RIGHT_R ? -1 : 1);

        uint8_t *data = (uint8_t *)&msg_Right;
        esp_now_send(addr_L, data, sizeof(message_from_remote));
    }
}
// void ESPNOW_sendMessage(message_from_remote *msg)
// {
//     if (addr_L != nullptr)
//     {
//         message_from_remote message;
//         memcpy(&message, msg, sizeof(message_from_remote));

// #if USE_DUAL_BOARDS
//         message.cmd_Left_L *= INVERT_CMD_LEFT_L ? -1 : 1;
//         message.cmd_Left_R *= INVERT_CMD_LEFT_R ? -1 : 1;
//         message.cmd_Right_L *= INVERT_CMD_RIGHT_L ? -1 : 1;
//         message.cmd_Right_R *= INVERT_CMD_RIGHT_R ? -1 : 1;
// #else
//         message.cmd_L *= INVERT_CMD_LEFT_L ? -1 : 1;
//         message.cmd_R *= INVERT_CMD_RIGHT_L ? -1 : 1;
// #endif

//         uint8_t *data = (uint8_t *)&message;
//         ESPNOW_sendBytes(data, sizeof(message_from_remote));
//     }
// }

#endif // ESPNOW_PROTOCOL_H