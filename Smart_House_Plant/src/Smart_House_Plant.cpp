/* Project: Robo Can
 * Author: Isaac Martinez
 * Date: 03-20-24
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

#include "Particle.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"
#include "Adafruit_BME280.h"
#include "Grove_Air_quality_Sensor.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe pumponandoffbutton = Adafruit_MQTT_Subscribe(&mqtt,AIO_USERNAME "/feeds/pumponandoffbutton");
Adafruit_MQTT_Publish moisturesensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisturesensor");
Adafruit_MQTT_Publish dustConcentration = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustConcentration");
Adafruit_MQTT_Publish airQuality = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airQuality");
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish _humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish _pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");

/************Declare Variables*************/
unsigned int last, lastTime;
float subValue,pubValue;
int randNumber;
int onandoffbutton;
/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();


// Set the system mode to semi-automatic
SYSTEM_MODE(SEMI_AUTOMATIC);

// Pin definitions
const int DUST_SENSOR_PIN = D3;
const int AIR_QUALITY_SENSOR_PIN = A0;
const int MOISTURE_SENSOR_PIN = A1;
const int PUMP_PIN = D16;
const int BUTTON_PIN = D5;

// Variables
unsigned long sampletime_ms = 120000; // 10 seconds
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

    // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  //Setup MQTT subscription
  mqtt.subscribe(&pumponandoffbutton);


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

   MQTT_connect();
  MQTT_ping();


  // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &pumponandoffbutton) {
      subValue = atoi((char *)pumponandoffbutton.lastread);
    }
    if(subValue == HIGH){
     digitalWrite (D16,HIGH);
     }
      else{
        digitalWrite(D16,LOW);
        }   
      
    }
  if((millis()-lastTime > 120000)) {
    if(mqtt.Update()) {
      // randNumber = random(300);
      // Serial.printf("The number is = %i \n",randNumber);
      moisturesensor.publish(moistureValue);
      // Serial.printf("Publishing %0.2f \n",randNumber); 
      } 
    lastTime = millis();
  }

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
    dustConcentration.publish(concentration);
    airQuality.publish(airQualityValue);
    temp.publish(temperature);
    _humidity.publish(humidity);
    _pressure.publish(pressure);
    moisturesensor.publish(moistureValue);

    // Automatically water the plant if soil is too dry
    if (moistureValue > 4000) {
      digitalWrite(PUMP_PIN, HIGH);
      delay(500); // Turn on the pump for 2 seconds
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

  // // Check if the manual watering button is pressed
  //  if (digitalRead(BUTTON_PIN) == LOW) {
  //    digitalWrite(PUMP_PIN, HIGH);
  //   delay(500); // Turn on the pump for 2 seconds
  // digitalWrite(PUMP_PIN, LOW);
  }

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  //Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
//74 thru 107 should be added to code to connect and maintain the connection to the adafruit service