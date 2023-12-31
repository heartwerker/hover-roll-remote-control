#pragma once
#ifndef HOVER_SERIAL_H
#define HOVER_SERIAL_H

// *******************************************************************
//  Base for this file was the "Arduino Nano 5V example code"
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
//
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • USING Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication

#include <SoftwareSerial.h>

    
#if USE_CMD_L_R
    typedef struct
    {
        uint16_t start;
        int16_t cmd_L;
        int16_t cmd_R;
        uint16_t checksum;
    } HoverSerialCommand;
#else
    typedef struct
    {
        uint16_t start;
        int16_t steer;
        int16_t speed;
        uint16_t checksum;
    } HoverSerialCommand;
#endif

    typedef struct
    {
        uint16_t start;
        int16_t cmd1;
        int16_t cmd2;
        int16_t speedR_meas;
        int16_t speedL_meas;
        int16_t batVoltage;
        int16_t boardTemp;
        uint16_t cmdLed;
        uint16_t checksum;
    } HoverSerialFeedback;

class HoverSerial
{
private:
    SoftwareSerial swSerial;

    // Global variables
    uint8_t idx = 0;
    uint16_t bufStartFrame;
    byte *p;
    byte incomingByte;
    byte incomingBytePrev;

    HoverSerialCommand Command;
    HoverSerialFeedback Feedback;
    HoverSerialFeedback NewFeedback;

public:
    // Construct with RX and TX pins
    HoverSerial(uint8_t rxPin, uint8_t txPin) : swSerial(rxPin, txPin){}

    void init()
    {
        swSerial.begin(HOVER_SERIAL_BAUD);
    }

#if USE_CMD_L_R
    void send(int16_t cmd_L, int16_t cmd_R)
    {
        Command.start = (uint16_t)START_FRAME;
        Command.cmd_L = cmd_L;
        Command.cmd_R = cmd_R;
        Command.checksum = (uint16_t)(Command.start ^ Command.cmd_L ^ Command.cmd_R);

        swSerial.write((uint8_t *)&Command, sizeof(Command));

#else

    void send(int16_t uSteer, int16_t uSpeed)
    {
        // Create command
        Command.start = (uint16_t)START_FRAME;
        Command.steer = (int16_t)uSteer;
        Command.speed = (int16_t)uSpeed;
        Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
        swSerial.write((uint8_t *)&Command, sizeof(Command));

#endif
    }

    HoverSerialFeedback getFeedback()
    {
        return Feedback;
    }

    void loop_receive()
    {
        // Check for new data availability in the Serial buffer
        if (swSerial.available())
        {
            incomingByte = swSerial.read();                                     // Read the incoming byte
            bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
        
        
        // Copy received data
        if (bufStartFrame == START_FRAME)
        { // Initialize if new data is detected
            p = (byte *)&NewFeedback;
            *p++ = incomingBytePrev;
            *p++ = incomingByte;
            idx = 2;
        }
        else if (idx >= 2 && idx < sizeof(HoverSerialFeedback))
    { // Save the new received data
            *p++ = incomingByte;
            idx++;
        }

        // Check if we reached the end of the package
        if (idx == sizeof(HoverSerialFeedback))
        {
            uint16_t checksum;
            checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

            // Check validity of the new data
            if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
            {
                // Copy the new data
                memcpy(&Feedback, &NewFeedback, sizeof(HoverSerialFeedback));

#if DEBUG_SERIAL_RECEIVE && 0
#if 0
            // Print data to built-in Serial
            Serial.print("1: ");
            Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");
            Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");
            Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");
            Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");
            Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");
            Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");
            Serial.println(Feedback.cmdLed);
#else
                // Print both in speed values in one line with fixed digit width
                Serial.printf(" Serial RX %+10d %+10d  ---  %+10d %+10d | ", Feedback.cmd1, Feedback.cmd2, Feedback.speedR_meas, Feedback.speedL_meas);
#endif
#endif
            }
            else
            {
                // Here you might want to handle the error, maybe throw an exception or similar.
                Serial.println("Non-valid data skipped");
            }
            idx = 0; // Reset the index
        }

        // Update previous states
        incomingBytePrev = incomingByte;
        }

    }
};

#endif // HOVER_SERIAL_H
