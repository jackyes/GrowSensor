#include <algorithm>
#include <iostream>
#include <Arduino.h>
#include <WiFi.h>
#include <Button2.h>
#include <Wire.h>
#include <ErriezBH1750.h>
#include "DHT.h"
#include <Adafruit_BME280.h>
#include <WiFiMulti.h>
#include "esp_wifi.h"
#include <HTTPClient.h>

#define I2C_SDA 25
#define I2C_SCL 26
//#define DHT12_PIN 16
#define BAT_ADC 33
#define SALT_PIN 34
#define SOIL_PIN 32
#define BOOT_PIN  0
#define POWER_CTRL  4
#define USER_BUTTON 35

#define TIME_TO_SLEEP 15                 //Time ESP32 will go to sleep (in minuts) between each reading
String apiKey = "xxxxx";     //Enter your Write API key from ThingSpeak
String IFTTTkey = "xxxx"; // Enter your IFTTT Key
String IFTTTLowMoistNm = "xxx";      //Enter the IFTTT event name for low moisture
String IFTTTLowBattNm = "xxx";      //Enter the IFTTT event name for low battery
int LowMoistVal = 35;                   //Low moisture value to trigger IFTTT Notification
int LowBatVal = 3200;                   //Low battery Voltage (mV)
#define WIFI_SSID "xxxx"       //WiFi SSID
#define WIFI_PASSWD "xxx"     //WiFi Password

// ADDR line LOW/open:  I2C address 0x23 (0x46 including R/W bit) [default]
// ADDR line HIGH:      I2C address 0x5C (0xB8 including R/W bit)
BH1750 sensor(LOW);

Adafruit_BME280 bmp;     //0x77
#define DHTPIN 16
#define DHTTYPE DHT12
DHT dht(DHTPIN, DHTTYPE);
#define uS_TO_MIN_FACTOR 60000000       // Conversion factor for micro seconds to MIN 
Button2 button(BOOT_PIN);
Button2 useButton(USER_BUTTON);
WiFiMulti multi;
HTTPClient http;

bool bme_found = false;
bool bh1750_found = false;

void smartConfigStart(Button2 &b)
{
    Serial.println("smartConfigStart...");
    WiFi.disconnect();
    WiFi.beginSmartConfig();
    while (!WiFi.smartConfigDone()) {
        Serial.print(".");
        delay(200);
    }
    WiFi.stopSmartConfig();
    Serial.println();
    Serial.print("smartConfigStop Connected:");
    Serial.print(WiFi.SSID());
    Serial.print("PSW: ");
    Serial.println(WiFi.psk());
}

void sleepHandler(Button2 &b)
{
    Serial.println("Enter Deepsleep ...");
    esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
    delay(1000);
    esp_deep_sleep_start();
}

void DeepSleep(){
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);   // poweroff RTC_SLOW_MEM hibernate mode
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);   // poweroff RTC_FAST_MEM hibernate mode
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);     // poweroff RTC_PERIPH hibernate mode
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_MIN_FACTOR);
  esp_deep_sleep_start(); //Enter in DeepSleep
}

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    wifi_config_t current_conf;
    esp_wifi_get_config(WIFI_IF_STA, &current_conf);
    int ssidlen = strlen((char *)(current_conf.sta.ssid));
    int passlen = strlen((char *)(current_conf.sta.password));

    if (ssidlen == 0 || passlen == 0) {
        multi.addAP(WIFI_SSID, WIFI_PASSWD);
        Serial.println("Connect to defalut ssid, you can long press BOOT button enter smart config mode");
        while (multi.run() != WL_CONNECTED) {
            Serial.print('.');
        }
    } else {
        WiFi.begin();
    }

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi connect fail!,please restart retry,or long press BOOT button enter smart config mode\n");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }

    button.setLongClickHandler(smartConfigStart);
    useButton.setLongClickHandler(sleepHandler);

    Wire.begin(I2C_SDA, I2C_SCL);

    dht.begin();

    //! Sensor power control pin , use deteced must set high
    pinMode(POWER_CTRL, OUTPUT);
    digitalWrite(POWER_CTRL, 1);
    delay(1000);

    if (!bmp.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        bme_found = false;
    } else {
        bme_found = true;
    }
    sensor.begin(ModeOneTime, ResolutionMid);
}


uint32_t readSalt()
{
  uint8_t samples = 120;
  uint32_t humi = 0;
  uint16_t array[120];

  for (int i = 0; i < samples; i++) {
    array[i] = analogRead(SALT_PIN);
    delay(2);
  }
  std::sort(array, array + samples);
  for (int i = 0; i < samples; i++) {
      if (i == 0 || i == samples - 1)continue;
        humi += array[i];
      }
  humi /= samples - 2;
  return humi;
}

uint16_t readSoil()
{
  uint16_t soil = analogRead(SOIL_PIN);
  return map(soil, 0, 4095, 100, 0);
}

float readBattery()
{
  int vref = 1100;
  uint16_t volt = analogRead(BAT_ADC);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

void loop()
{
if (WiFi.status() == WL_CONNECTED) {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  uint16_t lux;
  uint32_t salt = readSalt();
  uint16_t soil = readSoil();
  if (soil <= LowMoistVal){
     http.begin(String("https://maker.ifttt.com/trigger/") + IFTTTLowMoistNm + "/with/key/" + IFTTTkey); 
     int httpCode = http.GET();
     http.end(); //Free the resources
  }
  sensor.startConversion(); //light sensor
  //To avoid bad battery voltage readings
  float bat = 0;

  for (int i=0; i<3; i++){
    delay(5);
    bat+=readBattery();
  }
  bat=bat/3;
  if (bat <= LowBatVal){
     http.begin(String("https://maker.ifttt.com/trigger/") + IFTTTLowBattNm + "/with/key/" + IFTTTkey); 
     int httpCode = http.GET();
     http.end(); //Free the resources
  }
  
  if (sensor.waitForCompletion()) {
    lux = sensor.read();    
  } else {
    lux = -1;
  }
  if ((lux != 8180) &&(lux >= 0)){
    http.begin(String("https://api.thingspeak.com/update?api_key=") + apiKey + "&field1=" + t + "&field2=" + h + "&field3=" + lux + "&field4=" + salt + "&field5=" + soil + "&field6=" + bat); //lux data ok
  } else {
    http.begin(String("https://api.thingspeak.com/update?api_key=") + apiKey + "&field1=" + t + "&field2=" + h + "&field4=" + salt + "&field5=" + soil + "&field6=" + bat); //No lux data
  }

  int httpCode = http.GET(); 
  /*if (httpCode > 0) { //Check for the returning code   ---DEBUG PURPOSE
    String payload = http.getString();
    Serial.println(httpCode);
    Serial.println(payload);
  }else {
    Serial.println("Error on HTTP request");
  }*/
  http.end(); //Free the resources
    DeepSleep();
}
DeepSleep(); //if disconnected go in deepsleep
}
