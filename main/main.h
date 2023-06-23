#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <stdio.h>

#include "mpu6050.h"
#include "wifi.h"
#include "motor.h"
#include "myType.h"
#include "protocol.h"

#define THROTTLE_Period_ms  200 

#define FREQ            332               //6kHz = 332, 500Hz = 4000
#define OFF             0
#define ON              1
#define FALSE           0
#define TRUE            1

extern bool Debug;
extern float THROTTLE_IDLE, THROTTLE_MAX, THROTTLE_MIN;
