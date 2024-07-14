
# Hover Roll Remote Control

This is the firmware for (remote) for motorizing eg. a sofa with up to **4 hoverboard motors**.

It is inspired by and uses [github.com/EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)

- For control a Wii Nunchuck i used.

- For power output use either
    - hoverboard main PCB with custom firmware
    - or [BLDC 3 Phase 400W Motor Control 6V-60V](https://de.aliexpress.com/item/1005002287420689.html)
    

## Getting Started.

This project uses:

- PlatformIO (VSode)
- Wemos D1 mini (ESP8266)

## Configuration
Most adjustments can be made in the __"config.h"__ file.

- use this branch: https://github.com/heartwerker/hoverboard-firmware-hack-FOC/tree/master-hover-roll


### Hardware

#### Remote

Use: config.h:
```
#define IS_REMOTE 1
```

Connect Nunchuck:
SDA <-> D2
SCL <-> D1


