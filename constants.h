#define WIFI      1
#define THP       1
#define AGT       1
#define MAG       1
#define LIDAR     1
#define PIR       1
#define PROXIMITY 1
#define NFC       0
#define JSON      1
#define PRINTS    0
#if WIFI
#include <WiFi.h>
#endif

#if THP || AGT || MAG || LIDAR || NFC 
#include <Wire.h>
#endif

#if THP
#include <Adafruit_BME280.h>
#endif

#if AGT
#include <Adafruit_MPU6050.h>
#endif

#if MAG
#include <Adafruit_LIS2MDL.h>
#endif

#if JSON
#include <HTTPClient.h>
#include <ArduinoJson.h>
#endif

#if NFC
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
#endif

#include <Adafruit_Sensor.h>
#include <float.h>

const char* ssid = "";
const char* password = "";
const char* uri = "";

const int pirPin = 3;
const int proximityPin = 45;
const int lidarSdaPin = 8;
const int lidarSclPin = 9;

#define SEALEVELPRESSURE_HPA (1013.25)

#define LIDAR_ADDRESS         0x10
#define LIDAR_DATALENGTH      9
unsigned char lidar_buff[] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };

bool bWIFI = false;
bool bTHP = false;
bool bAGT = false;
bool bMAG = false;
bool bLIDAR = false;
bool bPIR = false;
bool bPROXIMITY = false;
bool bNFC = false;
