#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <ArduinoOTA.h>
#include <Ticker.h>

#include <BlynkSimpleEsp8266.h>
#include <ArduinoJson.h>

#include "appconfig.h"
#include "appinfo.h"

#pragma region[Variables]

const float tankHeightInCM = 152.4;
const float tankRadiusInCM = 45.7;
const float lowVolumeThersholdInPercentage = 15.0;
const float highVolumeThersholdInPercentage = 95.0;

float totalTankVolumeInCubicCM = 0.0;
float totalTankVolumeInLiters = 0.0;
float filledHeightInCM = 0.0;
float filledVolumeInCubicCM = 0.0;
float filledVolumeInLiters = 0.0;
float filledVolumeInPercentage = 0.0;

long duration = 0; // variable for the duration of sound wave travel
int distanceFromWaterLevel = 0; 

//Ticker stripTicker;
BlynkTimer timer;

HTTPClient httpClient; //Object of class HTTPClient
String apiToken;
String latestFirmwareFileUrl;


//Speed of sound m/s = 331.4 + (0.606 * Temp) + (0.0124 * Humidity)


#pragma endregion

#pragma region[Initialization]

void initializePins()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(CHIP_DIGITAL_PIN_SENSOR_PING, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(CHIP_DIGITAL_PIN_SENSOR_ECHO, INPUT); // Sets the echoPin as an INPUT
}

void initializeSerial()
{
  // Start serial and initialize stdout
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.print("Serial successfully initiated...");
  delay(2000);
}

void initiallizeWifiAndBlynk()
{
  // Config Blynk server
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_USERNAME, WIFI_PASSWORD);
  Serial.print("Blynk started...");
}

void initializeLocalOTA()
{
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA mode ready...");
  Serial.print("Use IP ");
  Serial.print(WiFi.localIP());
  Serial.println(" to upload new firmware");
}

void initializeEEPROM()
{
  // EEPROM.begin(512);

  // EEPROM.get(addrMoistureThresholdUpper, moistureThresholdUpper); //Get the upper moisture threshold
  // EEPROM.get(addrMoistureThresholdLower, moistureThresholdLower); //Get the upper moisture threshold
  // EEPROM.get(addrEnableAlarm, enableAlarm);
  // EEPROM.get(addrEnablestrip, enablestrip);
  // EEPROM.get(addrEnableAutoFirmwareUpdate, enableAutoFirmwareUpdate);
}

#pragma endregion

long readSensor()
{

}

float getFilledPercentageByPercentage(float volumeInLiters){
  return (volumeInLiters * 100) / totalTankVolumeInLiters;
}

float getFilledPercentageByCubicCM(float volumeInCubicCM){
  return (volumeInCubicCM * 100) / totalTankVolumeInCubicCM;
}

float convertCubicCMToLiters(float cubicCM){
  return cubicCM / 1000;
}

float getfilledVolumeInCubicCM(float heightInCM){
  return 3.14 * tankRadiusInCM * tankRadiusInCM * heightInCM;
}

void sendData(){
  //filledHeightInCM = tankHeightInCM - readSensor();
  filledHeightInCM = tankHeightInCM - distanceFromWaterLevel;
  filledVolumeInCubicCM = getfilledVolumeInCubicCM(filledHeightInCM);
  filledVolumeInLiters = convertCubicCMToLiters(filledVolumeInCubicCM);
  filledVolumeInPercentage = getFilledPercentageByCubicCM(filledVolumeInCubicCM);

  // Send notifications
  if(filledVolumeInPercentage >= highVolumeThersholdInPercentage){
      // TODO: send
      sendPushNotification("Tank is 95 full", "Your water tank is mre than 95 full");
  }
  else if (filledVolumeInPercentage <= lowVolumeThersholdInPercentage)
  {
    sendPushNotification("Tank is empty", "Your water tank is mre than 95 full");
  }
}

#pragma region[Blynk]

void updateBlynkCloud()
{
  // Send data to blynk cloud
  // Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_OUT_MOIS_SENSOR_CURRENT, currentMoistureLevelRecorded);
  // Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_OUT_MOIS_SENSOR_MIN, minMoistureLevelRecorded);
  // Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_OUT_MOIS_SENSOR_MAX, maxMoistureLevelRecorded);
}

BLYNK_CONNECTED()
{
  Blynk.syncAll();
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_OUT_MAC, WiFi.macAddress());
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_OUT_DEVICE, DEVICE_TYPE);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_OUT_FW_VERSION, CURRENT_FIRMWARE_VERSION);
  Serial.println("Synced with Blynk server...");
}

#pragma endregion

#pragma region[PUSH NOTIFICATIONS]

void sendPushNotification(char* title, char* message){
  String apiTokenRequest = String(PUSH_BULLET_BASE_URL);
  apiTokenRequest.concat("/users/authenticate");
  //HTTPClient client;
  httpClient.begin(apiTokenRequest);
  httpClient.addHeader("Content-Type", "application/json");
  httpClient.addHeader("Authorization", "Bearer o.dBXFt06SgHmuZtArEKgYZCgoeE3ggzO8"); //TODO: Make it to take from constant
  httpClient.addHeader("accept", "*/*");

  Serial.print("Requesting API token from: ");
  Serial.println(apiTokenRequest.c_str());

  // TODO: Make it to take from constant
  int statusCode = httpClient.POST("{ \"type\": \"note\", \"channel_tag\": \"8dd75bee-00e9-45fe-9f62-18c2b127dd0d\", \"title\": \"TITLE\", \"body\": \"BODY\"}");
  if (statusCode == 200)
  {
    Serial.println("Message pushed to Push Bullet");
  }
  else
  {
    Serial.print("Firmware version check failed, got HTTP response code ");
    Serial.println(statusCode);
  }

  httpClient.end();
}

#pragma endregion

#pragma region[Private Methods]

void printSystemInfo()
{
  Serial.print("Device Type: ");
  Serial.println(DEVICE_TYPE);
  Serial.print("Firmware Version: ");
  Serial.println(CURRENT_FIRMWARE_VERSION);
}

#pragma endregion

#pragma region[Setup &Loop]

void setup()
{
  initializeSerial();
  printSystemInfo();
  initializePins();
  initializeEEPROM();

  initiallizeWifiAndBlynk();
  initializeLocalOTA();

  totalTankVolumeInCubicCM = getfilledVolumeInCubicCM(tankHeightInCM);
  totalTankVolumeInLiters = convertCubicCMToLiters(totalTankVolumeInCubicCM);

  timer.setInterval(FREQUENCY_IN_SECONDS_SEND_DATA, sendData);
}

void loop()
{
   // Clears the trigPin condition
  digitalWrite(CHIP_DIGITAL_PIN_SENSOR_PING, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(CHIP_DIGITAL_PIN_SENSOR_PING, HIGH);
  delayMicroseconds(10);
  digitalWrite(CHIP_DIGITAL_PIN_SENSOR_PING, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  distanceFromWaterLevel = pulseIn(CHIP_DIGITAL_PIN_SENSOR_ECHO, HIGH);
  filledHeightInCM = duration * 0.034 / 2;
  // Handle OTA
  ArduinoOTA.handle();

  // Start Bylnk
  Blynk.run();
}

#pragma endregion