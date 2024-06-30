#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 2
#define DHTTYPE DHT22

float Level;
float uvIntensity;

SoftwareSerial sim800l(9, 10);
LiquidCrystal_I2C lcd(0x27, 20, 4);

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

void setup() {
  Serial.begin(9600);
  sim800l.begin(9600);
  dht.begin();
  lcd.init();
  lcd.backlight();

  sim800l.println("AT+CMGF=1");  // set SMS text mode
  delay(100);
  sim800l.println("AT+CNMI=1,2,0,0,0");  // set SIM800L to notify when new SMS is received
  delay(100);

  lcd.clear();
  lcd.setCursor(7, 1);
  lcd.print("SYSTEM");
  lcd.setCursor(3, 2);
  lcd.print("INITIALIZATION");
  delay(2000);

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}
void loop() {
  delay(delayMS);
  sensors_event_t event;
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("WEATHER STATION");
  lcd.setCursor(0, 1);
  lcd.print("T: ");
  dht.temperature().getEvent(&event);
  lcd.print(event.temperature, 1);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(12, 1);
  lcd.print("H: ");
  dht.humidity().getEvent(&event);
  lcd.print(event.relative_humidity, 1);
  lcd.print("%");
  lcd.setCursor(0, 2);
  lcd.print("WIND SPEED: ");
  WindSpeed();
  lcd.print(Level, 1);
  lcd.print(" m/s");
  lcd.setCursor(0, 3);
  lcd.print("UV LIGHT: ");
  UvIntensity();
  lcd.print(uvIntensity, 2);
  lcd.print(" W/m^2");
  delay(1000);

  if (sim800l.available()) {                         // check if there is a message available
    String message = sim800l.readString();           // read the message
    Serial.println("Received message: " + message);  // print the message to the serial monitor
    if (message.indexOf("STATUS") != -1) {           // if the message contains "ON"
      sim800l.println("AT+CMGF=1");  // Configuring TEXT mode
      updateSerial();
      sim800l.println("AT+CMGS=\"+254757885645\"");  //change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      sim800l.println("   WEATHER STATION DATA");  //text content
      sim800l.print("Temperature: ");
      dht.temperature().getEvent(&event);
      sim800l.print(event.temperature, 1);
      sim800l.println(" C");
      sim800l.print("Humidity: ");
      dht.humidity().getEvent(&event);
      sim800l.print(event.relative_humidity, 1);
      sim800l.println("%");
      sim800l.print("Wind Speed: ");
      WindSpeed();
      sim800l.print(Level, 1);
      sim800l.println("m/s");
      sim800l.print("UV Light Intensity: ");
      UvIntensity();
      sim800l.print(uvIntensity);
      sim800l.print("W/m^2");
      updateSerial();
      sim800l.write(26);
      delay(2000);
    }
  }
}
void updateSerial() {
  delay(500);
  while (Serial.available()) {
    sim800l.write(Serial.read());  //Forward what Serial received to Software Serial Port
  }
  while (sim800l.available()) {
    Serial.write(sim800l.read());  //Forward what Software Serial received to Serial Port
  }
}
void WindSpeed() {
  int sensorValue = analogRead(A2);
  float outvoltage = sensorValue * (5.0 / 1023.0);
  Level = 6 * outvoltage;  //The level of wind speed is proportional to the output voltage.
}
void UvIntensity() {
  int UvValue = analogRead(A1);
  uvIntensity = mapFloat(UvValue, 0, 1023, 0, 15);
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}