/* 
 * Project: Cloud
 * Author: Isaac
 * Date: 3-18-24
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

#include "Particle.h"
#include "math.h"
#include "Adafruit_SSD1306.h"
#include "Air_Quality_Sensor.h"
#include "Adafruit_BME280.h"

// Set the system mode to semi-automatic
SYSTEM_MODE(SEMI_AUTOMATIC);

// Pin definitions
const int DUST_SENSOR_PIN = D3;
const int AIR_QUALITY_SENSOR_PIN = A0;
const int MOISTURE_SENSOR_PIN = A1;
const int PUMP_PIN = D16;
const int BUTTON_PIN = D5;

// Variables
unsigned long sampletime_ms = 10000; // 10 seconds
unsigned long duration;
unsigned long starttime;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
int OLED_RESET = -1;
int moistureValue;
String DateTime, TimeOnly;

// Create an instance of the Air Quality Sensor
AirQualitySensor airSensor(AIR_QUALITY_SENSOR_PIN);

// Create an instance of the Adafruit SSD1306 display
Adafruit_SSD1306 display(OLED_RESET);

// Create an instance of the Adafruit BME280 sensor
Adafruit_BME280 bme;

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);

  // Initialize Grove - Dust Sensor
  pinMode(DUST_SENSOR_PIN, INPUT);

  // Initialize Grove - Air Quality Sensor
  airSensor.init();

  // Initialize Adafruit SSD1306 display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  // Initialize Adafruit BME280 sensor
  bme.begin(0x76);

  // Initialize moisture sensor pin
  pinMode(MOISTURE_SENSOR_PIN, INPUT);

  // Initialize pump pin
  pinMode(PUMP_PIN, OUTPUT);

  // Initialize button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Wait for sensors to stabilize
  delay(1000);
  starttime = millis();

  // Configure time zone
  Time.zone(-6); // MST = -7, MDT = -6
  Particle.syncTime(); // Sync time with Particle Cloud
}

void loop() {
  duration = pulseIn(DUST_SENSOR_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  if ((millis() - starttime) > sampletime_ms) {
    // Calculate dust sensor readings
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0);
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
    Serial.printf("Low Pulse %i\n", lowpulseoccupancy);
    Serial.printf("Dust Ratio %0.2f\n", ratio);
    Serial.printf("Dust Concentration %0.2f\n", concentration);

    // Read Grove - Air Quality Sensor value
    int airQualityValue = airSensor.slope();

    // Read BME280 sensor values
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;

    // Read moisture sensor value
    moistureValue = analogRead(MOISTURE_SENSOR_PIN);

    // Print sensor values
    Serial.printf("Grove - Air Quality Sensor Value: %d\n", airQualityValue);
    Serial.printf("Temperature: %.2f °C\n", temperature);
    Serial.printf("Humidity: %.2f %%\n", humidity);
    Serial.printf("Pressure: %.2f hPa\n", pressure);
    Serial.printf("Moisture Value: %i\n", moistureValue);

    // Publish sensor data to a dashboard
    Particle.publish("dust_concentration", String(concentration));
    Particle.publish("air_quality", String(airQualityValue));
    Particle.publish("temperature", String(temperature));
    Particle.publish("humidity", String(humidity));
    Particle.publish("pressure", String(pressure));
    Particle.publish("moisture", String(moistureValue));

    // Automatically water the plant if soil is too dry
    if (moistureValue < 500) {
      digitalWrite(PUMP_PIN, HIGH);
      delay(2000); // Turn on the pump for 2 seconds
      digitalWrite(PUMP_PIN, LOW);
    }

    // Display readings on the Adafruit SSD1306 display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.printf("Dust: %.2f\n", concentration);
    display.printf("AQ: %d\n", airQualityValue);
    display.printf("Temp: %.2f C\n", temperature);
    display.printf("Hum: %.2f %%\n", humidity);
    display.printf("Pres: %.2f hPa\n", pressure);
    display.printf("Moisture: %i\n", moistureValue);
    DateTime = Time.timeStr();
    TimeOnly = DateTime.substring(11, 19);
    display.printf("Time: %s\n", TimeOnly.c_str());
    display.display();

    // Reset variables
    lowpulseoccupancy = 0;
    starttime = millis();
  }

  // Check if the manual watering button is pressed
  if (digitalRead(BUTTON_PIN) == LOW) {
    digitalWrite(PUMP_PIN, HIGH);
    delay(2000); // Turn on the pump for 2 seconds
    digitalWrite(PUMP_PIN, LOW);
  }
}