#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusWaterSensor.h>

// Wi-Fi
const char *Wifissid = "HabibiGarden";
const char *WiFipassword = "Prodigy123";

// MQTT Broker settings
const char *mqttBroker = "192.168.1.55";
const int mqttPort = 1883;
const char *mqttUser = "";     // Leave empty if no username is required
const char *mqttPassword = ""; // Leave empty if no password is required

// MQTT topics
const char *TDSTopic = "ABsensor/TDS";
const char *phTopic = "ABsensor/ph";
const char *temperatureTopic = "ABsensor/temperature";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// RS485 Pin Sensor
#define RX2_PIN 16
#define TX2_PIN 17
#define REDE_PIN 2
#define BAUDRATE_2 9600

LiquidCrystal_I2C lcd(16, 2);

HardwareSerial Sensor(2);
EC_TDS ec_tds(Sensor, RX2_PIN, TX2_PIN, BAUDRATE_2);
PH_Temperature ph_temp(Sensor, RX2_PIN, TX2_PIN, BAUDRATE_2);

void preTransmission();
void postTransmission();
void connectToWiFi();
void connectToMQTT();
void publishSensorData();
void displayScrollingText(String text);

void setup()
{ // Serial setup
  Serial.begin(115200);

  // LCD setup
  lcd.autoAddress();
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");

  // RS485 setup
  pinMode(REDE_PIN, OUTPUT);
  digitalWrite(REDE_PIN, LOW);

  ec_tds.preTransmission(preTransmission);
  ph_temp.preTransmission(preTransmission);
  ec_tds.postTransmission(postTransmission);
  ph_temp.postTransmission(postTransmission);
  ec_tds.begin(0x02, 0x03, 0x00, 0x02);
  ph_temp.begin(0x01, 0x03, 0x00, 0x02);

  // WiFi setup
  connectToWiFi();

  // MQTT setup
  mqttClient.setServer(mqttBroker, mqttPort);
}
void loop()
{
  // Ensure MQTT client is connected
  if (!mqttClient.connected())
  {
    connectToMQTT();
  }
  mqttClient.loop();

  // read sensor data
  float tds = ec_tds.readValue();
  float temperature = ph_temp.readTemperature();
  float pH = ph_temp.readPH();

  // Display running text di LCD
  String text = "TDS: " + String(tds, 1) + " PPM  PH: " + String(pH, 1) + "  T: " + String(temperature, 1) + "*C";
  displayScrollingText(text);
  delay(500);

  // publish data ke MQTT tiap 5 detik
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis > 5000)

    lastMillis = millis();
  publishSensorData();
}

void connectToWiFi()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");
  WiFi.begin("HabibiGarden", "Prodigy123");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    lcd.setCursor(0, 1);
    lcd.print(".");
  }

  Serial.println("WiFi Connected");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected");
}

void connectToMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT...");
    lcd.setCursor(0, 1);
    lcd.print("MQTT Connect...");
    if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword))
    {
      Serial.println("Connected to MQTT");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MQTT Connected");
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void publishSensorData()
{
  float tds = ec_tds.readValue();
  float temperature = ph_temp.readTemperature();
  float pH = ph_temp.readPH();

  // Publish data ke MQTT
  mqttClient.publish(TDSTopic, String(tds, 1).c_str());
  mqttClient.publish(phTopic, String(pH, 1).c_str());
  mqttClient.publish(temperatureTopic, String(temperature, 1).c_str());

  Serial.println("Sensor data published");
}

void preTransmission()
{
  digitalWrite(REDE_PIN, HIGH);
}

void postTransmission()
{
  digitalWrite(REDE_PIN, LOW);
}

void displayScrollingText(String text)
{
  for (int i = 0; i < text.length() - 15; i++)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(text.substring(i, i + 16));
    delay(300); // delay per karakter
  }
}