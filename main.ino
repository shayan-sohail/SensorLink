#include "constants.h"

#if THP
Adafruit_BME280 bme; // I2C
#endif

#if AGT
Adafruit_MPU6050 mpu;
#endif

#if MAG
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
#endif

#if NFC
PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);
#endif

#if JSON
DynamicJsonDocument json(512);
HTTPClient http;
#endif

struct Data
{
  float temperature = 0;
  float pressure = 0;
  float altitude = 0;
  float humidity = 0;
  sensors_vec_t acceleration = { 0 };
  sensors_vec_t gyro = { 0 };
  sensors_vec_t magnetic = { 0 };
  String lastCardType = "--";
  String lastCardUid = "--";
  bool motion = false;
  bool proximity = false;
  uint16_t lidar = 0;
} data;


void setup() {
  Serial.begin(115200);

  Serial.println("Start...");

#if WIFI
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  bWIFI = true;
#endif

#if THP
  if (!bme.begin(0x77))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    bTHP = false;
  }
  else
  {
    Serial.println("THP Initialized");
    bTHP = true;
  }
#endif

#if AGT
  if (!mpu.begin())
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    bAGT = false;
  }
  else
  {
    Serial.println("AGT Initialized");
    bAGT = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }
#endif

#if MAG
  if (!mag.begin())
  {
    Serial.println("Could not find a valid LIS2MDL sensor, check wiring!");
    bMAG = false;
  }
  else
  {
    Serial.println("MAG Initialized");
    bMAG = true;
  }
#endif

#if LIDAR
  Wire.begin(lidarSdaPin, lidarSclPin);
  Serial.println("LIDAR Initialized");
  bLIDAR = true;
#endif

#if PIR
  xTaskCreatePinnedToCore(pirTask, "PIR Task", 2400, nullptr, 1, nullptr, 1);
  Serial.println("PIR Initialized");
  bPIR = true;
#endif

#if PROXIMITY
  attachInterrupt(digitalPinToInterrupt(proximityPin), proximityTask, RISING);
  Serial.println("PROXIMITY Initialized");
  bPROXIMITY = true;
#endif

#if NFC
  nfc.begin();
  Serial.println("NFC Initialized");
  bNFC = true;
#endif
}

void loop() {

#if THP
  if (bTHP)
    getTHPValues(data.temperature, data.pressure, data.altitude, data.humidity);
#endif

#if AGT
  if (bAGT)
    getAGTValues(data.acceleration, data.gyro);
#endif

#if MAG
  if (bMAG)
    getMAGValues(data.magnetic);
#endif

#if LIDAR
  if (bLIDAR)
    getLIDARValue(data.lidar);
#endif

#if NFC
  if (bNFC)
    getNfcValue(data.lastCardType, data.lastCardUid);
#endif

#if PRINTS
  printSensorValues();
#endif

#if JSON
  if (bWIFI)
    sendToServer();
#endif

  delay(1000);
}


#if THP
void getTHPValues(float& temperature, float& pressure, float& altitude, float& humidity)
{
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
}
#endif

#if AGT
void getAGTValues(sensors_vec_t& acceleration, sensors_vec_t& gyro)
{
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  acceleration.x = a.acceleration.x;
  acceleration.y = a.acceleration.y;
  acceleration.z = a.acceleration.z;

  gyro.x = g.gyro.x;
  gyro.y = g.gyro.y;
  gyro.z = g.gyro.z;
}
#endif

#if MAG
void getMAGValues(sensors_vec_t& magnetic)
{
  sensors_event_t m;
  mag.getEvent(&m);

  magnetic.x = m.magnetic.x;
  magnetic.y = m.magnetic.y;
  magnetic.z = m.magnetic.z;
}
#endif

#if LIDAR
void getLIDARValue(uint16_t& lidar)
{
  Wire.beginTransmission(LIDAR_ADDRESS);
  Wire.write(lidar_buff, 5);
  Wire.endTransmission();

  Wire.requestFrom(LIDAR_ADDRESS, LIDAR_DATALENGTH);
  uint8_t data[LIDAR_DATALENGTH] = { 0 };
  uint16_t distance = 0, strength = 0;
  int16_t temperature = 0;
  int index = 0;
  while (Wire.available() > 0 && index < LIDAR_DATALENGTH) {
    data[index++] = Wire.read();
  }
  if (index == LIDAR_DATALENGTH) {
    distance = data[2] + data[3] * 256;
    lidar = distance;
  }
  else {
    lidar = 0;
  }
}
#endif

#if PIR
void pirTask(void* parameter)
{
  pinMode(pirPin, INPUT);

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    data.motion |= digitalRead(pirPin) == HIGH; // Once a motion is detected it will keep its motion detected state unless it resets
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
#endif

#if PROXIMITY
void proximityTask()
{
  data.proximity = true;
}
#endif

#if NFC
void getNfcValue(String& lastCardType, String& lastCardUid)
{
  if (nfc.tagPresent(1))
  {
    NfcTag tag = nfc.read();
    lastCardType = tag.getTagType();
    lastCardUid = tag.getUidString();
  }
}
#endif

#if JSON
void sendToServer()
{
  static int count = 0;
  count++;
  if (WiFi.status() == WL_CONNECTED && count % 1 == 0)
  {
    json["location"]["lat"] = 12;
    json["location"]["lng"] = 152;
    json["temperature"] = data.temperature;
    json["pressure"] = data.pressure;
    json["humidity"] = data.humidity;
    json["acceleration"]["x"] = data.acceleration.x;
    json["acceleration"]["y"] = data.acceleration.y;
    json["acceleration"]["z"] = data.acceleration.z;
    json["gyroscope"]["x"] = data.gyro.x;
    json["gyroscope"]["y"] = data.gyro.y;
    json["gyroscope"]["z"] = data.gyro.z;
    json["magnetometer"]["x"] = data.magnetic.x;
    json["magnetometer"]["y"] = data.magnetic.y;
    json["magnetometer"]["z"] = data.magnetic.z;
    json["last_card_type"] = data.lastCardType;
    json["last_card_uid"] = data.lastCardUid;
    json["motion"] = data.motion;
    json["proximity"] = data.proximity;
    json["lidar"] = data.lidar;

    // Convert JSON document to string
    String jsonPayload;
    serializeJson(json, jsonPayload);

    http.begin(uri);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonPayload);
    Serial.println("JSON: ");
    Serial.println(jsonPayload);
    // Check for response
    if (httpResponseCode > 0)
    {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String response = http.getString();
      Serial.println("Response: ");
      Serial.println(response);
    }
    else
    {
      Serial.print("Error sending HTTP POST request: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
}
#endif

#if PRINTS
void printSensorValues()
{
  if (bTHP)
  {
    Serial.print("THP: ");
    Serial.print(data.temperature);
    Serial.print(" *C, ");

    Serial.print(data.pressure);
    Serial.print(" hPa, ");

    Serial.print(data.altitude);
    Serial.print(" m, ");

    Serial.print(data.humidity);
    Serial.println(" %");
  }
  if (bAGT)
  {
    Serial.print("AGT: ");
    Serial.print(data.acceleration.x);
    Serial.print(", ");
    Serial.print(data.acceleration.y);
    Serial.print(", ");
    Serial.print(data.acceleration.z);
    Serial.print(" m/s^2, ");

    Serial.print(data.gyro.x);
    Serial.print(", ");
    Serial.print(data.gyro.y);
    Serial.print(", ");
    Serial.print(data.gyro.z);
    Serial.println(" rad/s");
  }
  if (bMAG)
  {
    Serial.print("MAG: ");
    Serial.print(data.magnetic.x);
    Serial.print(", ");
    Serial.print(data.magnetic.y);
    Serial.print(", ");
    Serial.print(data.magnetic.z);
    Serial.println(" µT");
  }
  if (bLIDAR)
  {
    Serial.print("Distance: ");
    Serial.print(data.lidar);
    Serial.println(" cm");
  }
  if (bPIR)
  {
    Serial.print("PIR: ");
    Serial.println(data.motion == true ? "Detected" : "Not Detected");
    data.motion = false; // For testing purposes
  }
  if (bPROXIMITY)
  {
    Serial.print("PROXIMITY: ");
    Serial.println(data.proximity == true ? "Detected" : "Not Detected");
    data.proximity = false; // For testing purposes
  }
  if (bNFC)
  {
    Serial.print("NFC Type: ");
    Serial.print(data.lastCardType);
    Serial.print(", NFC UID: ");
    Serial.println(data.lastCardUid);

    //For Testing Purposes
    data.lastCardType = "--";
    data.lastCardUid = "--";
  }

  if (bTHP || bAGT || bMAG || bLIDAR || bPIR || bPROXIMITY || bNFC)
    Serial.println();
}
#endif