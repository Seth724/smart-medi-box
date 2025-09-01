#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <NTPClient.h>
#include <ESP32Servo.h>  
#include <math.h>   

#define DHT_PIN 15
#define BUZZER 12
#define LDR_PIN 34  
#define SERVO_PIN 13  

WiFiClient espClient;
PubSubClient mqttClient(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

char tempAr[6];
char lightIntensityAr[6];  

DHTesp dhtSensor;
Servo shadeServo;  

bool isScheduledON = false;
unsigned long scheduledOnTime;

// Light intensity sampling and averaging
float lightIntensitySum = 0;
int lightSampleCount = 0;
float lastAverageLightIntensity = 0;

// Configurable parameters with default values
int samplingInterval = 5;  // ts in seconds (default: 5s)
int sendingInterval = 120;  // tu in seconds (default: 2 minutes = 120s)
float minAngle = 30;       // θ_offset (default: 30 degrees)
float controllingFactor = 0.75;  // γ (default: 0.75)
float idealTemp = 30;      // T_med (default: 30°C)

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);

  setupWifi();

  setupMqtt();

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  timeClient.begin();
  timeClient.setTimeOffset(5.5 * 3600);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  // Initialize servo for shaded window
  shadeServo.attach(SERVO_PIN);
  shadeServo.write(minAngle);  // Set initial position to minimum angle

  pinMode(LDR_PIN, INPUT);  
}

void loop() {
  if (!mqttClient.connected()) {
    connectToBroker();
  }
  mqttClient.loop();

  updateTemperature();
  Serial.println(tempAr);
  mqttClient.publish("EE-ADMIN-TEMP", tempAr);

  checkSchedule();

  // Handle light intensity sampling and averaging
  unsigned long currentTime = millis();
  
  // Sample light intensity every `samplingInterval` seconds
  if (currentTime - lastSampleTime >= samplingInterval * 1000) {
    float lightIntensity = readLightIntensity();
    lightIntensitySum += lightIntensity;
    lightSampleCount++;
    lastSampleTime = currentTime;
  }

  // Send average light intensity every `sendingInterval` seconds
  if (currentTime - lastSendTime >= sendingInterval * 1000) {
    if (lightSampleCount > 0) {
      lastAverageLightIntensity = lightIntensitySum / lightSampleCount;
      String(lastAverageLightIntensity, 2).toCharArray(lightIntensityAr, 6);
      mqttClient.publish("EE-ADMIN-LIGHT-INTENSITY", lightIntensityAr);
      Serial.print("Average Light Intensity: ");
      Serial.println(lastAverageLightIntensity);

      // Update servo angle based on light intensity and temperature
      updateServoAngle();
    }
    // Reset for next averaging period
    lightIntensitySum = 0;
    lightSampleCount = 0;
    lastSendTime = currentTime;
  }

  delay(1000);
}

void buzzerOn(bool on) {
  if (on) {
    tone(BUZZER, 256);
  } else {
    noTone(BUZZER);
  }
}

void setupMqtt() {
  mqttClient.setServer("broker.hivemq.com", 1883);
  mqttClient.setCallback(receiveCallback);
}

void receiveCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char payloadCharAr[length];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    payloadCharAr[i] = (char)payload[i];
  }

  Serial.println();

  if (strcmp(topic, "EE-ADMIN-MAIN-ON-OFF") == 0) {
    buzzerOn(payloadCharAr[0] == '1');
  } else if (strcmp(topic, "EE-ADMIN-SCH-ON") == 0) {
    if (payloadCharAr[0] == 'N') {
      isScheduledON = false;
    } else {
      isScheduledON = true;
      scheduledOnTime = atol(payloadCharAr);
    }
  } else if (strcmp(topic, "EE-ADMIN-SAMPLE-INTERVAL") == 0) {
    samplingInterval = atoi(payloadCharAr);
    Serial.print("Sampling Interval Updated: ");
    Serial.println(samplingInterval);
  } else if (strcmp(topic, "EE-ADMIN-SEND-INTERVAL") == 0) {
    sendingInterval = atoi(payloadCharAr);
    Serial.print("Sending Interval Updated: ");
    Serial.println(sendingInterval);
  } else if (strcmp(topic, "EE-ADMIN-MIN-ANGLE") == 0) {
    minAngle = atoi(payloadCharAr);
    Serial.print("Minimum Angle Updated: ");
    Serial.println(minAngle);
  } else if (strcmp(topic, "EE-ADMIN-CONTROLLING-FACTOR") == 0) {
    controllingFactor = atof(payloadCharAr);
    Serial.print("Controlling Factor Updated: ");
    Serial.println(controllingFactor);
  } else if (strcmp(topic, "EE-ADMIN-IDEAL-TEMP") == 0) {
    idealTemp = atof(payloadCharAr);
    Serial.print("Ideal Temperature Updated: ");
    Serial.println(idealTemp);
  }
}

void connectToBroker() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32-12345")) {
      Serial.println("connected");
      mqttClient.subscribe("EE-ADMIN-MAIN-ON-OFF");
      mqttClient.subscribe("EE-ADMIN-SCH-ON");
      mqttClient.subscribe("EE-ADMIN-SAMPLE-INTERVAL");  // Subscribe to sampling interval
      mqttClient.subscribe("EE-ADMIN-SEND-INTERVAL");    // Subscribe to sending interval
      mqttClient.subscribe("EE-ADMIN-MIN-ANGLE");        // Subscribe to minimum angle
      mqttClient.subscribe("EE-ADMIN-CONTROLLING-FACTOR");  // Subscribe to controlling factor
      mqttClient.subscribe("EE-ADMIN-IDEAL-TEMP");       // Subscribe to ideal temperature
    } else {
      Serial.print("failed ");
      Serial.print(mqttClient.state());
      delay(5000);
    }
  }
}

void updateTemperature() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  String(data.temperature, 2).toCharArray(tempAr, 6);
}

void setupWifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println("Wokwi-GUEST");
  WiFi.begin("Wokwi-GUEST", "");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

unsigned long getTime() {
  timeClient.update();
  return timeClient.getEpochTime();
}

void checkSchedule() {
  if (isScheduledON) {
    unsigned long currentTime = getTime();
    if (currentTime > scheduledOnTime) {
      buzzerOn(true);
      isScheduledON = false;
      mqttClient.publish("EE-ADMIN-MAIN-ON-OFF-ESP", "1");
      mqttClient.publish("EE-ADMIN-SCH-ESP-ON", "0");
      Serial.println("Scheduled ON");
    }
  }
}

// Read and normalize light intensity (0 to 1) for 0–100,000 lux range
float readLightIntensity() {
  int ldrValue = analogRead(LDR_PIN);  // Read raw LDR value (0–4095 on ESP32)
  float lux = (float)ldrValue * (100000.0 / 4095.0);  // Map to 0–100,000 lux
  float intensity = lux / 100000.0;  // Normalize to 0–1
  if (intensity < 0.0) intensity = 0.0;
  if (intensity > 1.0) intensity = 1.0;
  return intensity;
}

// Calculate and update servo angle based on the equation
void updateServoAngle() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  float currentTemp = data.temperature;  // T (measured temperature)

  // Equation: θ = θ_offset + (180 − θ_offset) × I × γ × ln(ts / tu) × (T / T_med)
  float I = lastAverageLightIntensity;  // Light intensity (0–1)
  float T = currentTemp;  // Current temperature
  float T_med = idealTemp;  // Ideal temperature
  float ts = (float)samplingInterval;  // Sampling interval in seconds
  float tu = (float)sendingInterval;   // Sending interval in seconds
  float gamma = controllingFactor;     // Controlling factor (γ)
  float theta_offset = minAngle;       // Minimum angle (θ_offset)

  // Calculate motor angle (θ)
  float angle = theta_offset;
  if (tu > 0 && ts > 0) {  // Avoid division by zero or invalid logarithm
    float logTerm = log(ts / tu);
    angle = theta_offset + (180 - theta_offset) * I * gamma * logTerm * (T / T_med);
  } else {
    Serial.println("Warning: Invalid ts or tu, using minimum angle");
  }

  // Ensure angle is within 0–180 degrees
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  // Update servo position
  shadeServo.write((int)angle);
  Serial.print("Servo Angle: ");
  Serial.println(angle);
}
