#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include "secrets.h"

// ===== MQTT TOPICS CONFIGURATION =====
const char* mqtt_topic_control = MQTT_CONTROL_TOPIC;     // "chickendoor/control" - Command input
const char* mqtt_topic_status = MQTT_STATUS_TOPIC;       // "chickendoor/status" - Status output
const char* mqtt_topic_heartbeat = MQTT_HEARTBEAT_TOPIC; // "chickendoor/heartbeat" - Periodic alive signal
const char* mqtt_topic_emergency = MQTT_EMERGENCY_TOPIC; // "chickendoor/emergency" - Emergency stop control
const char* mqtt_topic_steps = "chickendoor/steps";      // "chickendoor/steps" - Set number of steps
const char* mqtt_topic_speed = "chickendoor/speed";      // "chickendoor/speed" - Set motor speed
const char* mqtt_client_id = MQTT_CLIENT_ID;             // "ESP32ChickenDoor" - Client identifier

// Add more MQTT topics here as needed:
// const char* mqtt_topic_debug = "chickendoor/debug";
// const char* mqtt_topic_error = "chickendoor/error";
// =====================================

// ===== CONFIGURATION VARIABLES (SET YOUR DEFAULTS HERE) =====
// Motor settings - adjust these defaults to your preferred values
const int stepsPerRevolution = 200;  // Steps per revolution for your motor
const int defaultSteps = 12000;      // Default steps (adjust to your door)
const int defaultSpeed = 100;        // Default speed in steps/second (slower than before)
const int minSpeed = 10;             // Minimum allowed speed
const int maxSpeed = 200;            // Maximum allowed speed

// EEPROM addresses for persistent storage
const int EEPROM_STEPS_ADDR = 0;     // Address to store step count
const int EEPROM_SPEED_ADDR = 8;     // Address to store speed
const int EEPROM_MAGIC_ADDR = 12;    // Address to store magic number (validates EEPROM)
const int EEPROM_MAGIC_VALUE = 0xABCD1234; // Magic number to check if EEPROM is initialized

// Motor control variables (loaded from EEPROM)
int desiredSteps = defaultSteps;     // Current step count
int motorSpeed = defaultSpeed;       // Current motor speed
// =============================================================

// Pin assignments
const int dirPin = 32;
const int stepPin = 25;
const int enPin = 27;  // Emergency stop/enable pin for motor driver

// Create AccelStepper object (DRIVER mode for step/direction)
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// LED pin
#define ONBOARD_LED_PIN 2  // Onboard LED on the ESP32
#define STATUS_LED_PIN 26  // External status LED (pin 25 is used by stepper)

// Heartbeat settings
const unsigned long heartbeatInterval = 30000;  // Send heartbeat every 30 seconds
unsigned long lastHeartbeat = 0;

bool motorSequenceTriggered = false;  // Flag to trigger the motor sequence

// WiFi and MQTT connection settings
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;

WiFiClient espClient;
PubSubClient client(espClient);

// Motor control
bool motorRunning = false;  // Track if motor is currently running

// Function declarations
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void publishStatus(const char* status);
void publishHeartbeat();
void runMotorSequence();
void moveMotorForward();
void moveMotorBackward();
void reconnect();
void checkWiFiConnection();
void checkEmergencyStop();
void loadStepsFromEEPROM();
void saveStepsToEEPROM(int steps);
void publishCurrentSteps();
void loadSpeedFromEEPROM();
void saveSpeedToEEPROM(int speed);
void publishCurrentSpeed();
void setStatusLED(bool on);

void loadStepsFromEEPROM() {
  // Check if EEPROM has been initialized
  int magicValue;
  EEPROM.get(EEPROM_MAGIC_ADDR, magicValue);
  
  if (magicValue == EEPROM_MAGIC_VALUE) {
    // EEPROM is initialized, load the saved steps
    EEPROM.get(EEPROM_STEPS_ADDR, desiredSteps);
    Serial.print("Loaded steps from EEPROM: ");
    Serial.println(desiredSteps);
  } else {
    // First time boot, save default steps
    desiredSteps = defaultSteps;
    saveStepsToEEPROM(desiredSteps);
    Serial.print("First boot - using default steps: ");
    Serial.println(desiredSteps);
  }
}

void saveStepsToEEPROM(int steps) {
  EEPROM.put(EEPROM_STEPS_ADDR, steps);
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.commit(); // Important for ESP32!
  Serial.print("Saved steps to EEPROM: ");
  Serial.println(steps);
}

void publishCurrentSteps() {
  if (client.connected()) {
    String stepsMessage = String(desiredSteps);
    client.publish(mqtt_topic_steps, stepsMessage.c_str(), true);  // Retain message
    Serial.print("Published current steps: ");
    Serial.println(desiredSteps);
  }
}

void loadSpeedFromEEPROM() {
  // Check if EEPROM has been initialized
  int magicValue;
  EEPROM.get(EEPROM_MAGIC_ADDR, magicValue);
  
  if (magicValue == EEPROM_MAGIC_VALUE) {
    // EEPROM is initialized, load the saved speed
    EEPROM.get(EEPROM_SPEED_ADDR, motorSpeed);
    Serial.print("Loaded speed from EEPROM: ");
    Serial.println(motorSpeed);
  } else {
    // First time boot, save default speed
    motorSpeed = defaultSpeed;
    saveSpeedToEEPROM(motorSpeed);
    Serial.print("First boot - using default speed: ");
    Serial.println(motorSpeed);
  }
}

void saveSpeedToEEPROM(int speed) {
  EEPROM.put(EEPROM_SPEED_ADDR, speed);
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.commit(); // Important for ESP32!
  Serial.print("Saved speed to EEPROM: ");
  Serial.println(speed);
}

void publishCurrentSpeed() {
  if (client.connected()) {
    String speedMessage = String(motorSpeed);
    client.publish(mqtt_topic_speed, speedMessage.c_str(), true);  // Retain message
    Serial.print("Published current speed: ");
    Serial.println(motorSpeed);
  }
}

void setStatusLED(bool on) {
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
  Serial.print("Status LED: ");
  Serial.println(on ? "ON" : "OFF");
}

void setup_wifi() {
  delay(10);
  Serial.println();
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
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  if (String(topic) == mqtt_topic_control && !motorRunning) {
    if (message == "close") {
      Serial.println("Close command received");
      motorSequenceTriggered = true;
    } else {
      Serial.println("Unknown command or motor busy");
    }
  } else if (String(topic) == mqtt_topic_emergency) {
    if (message == "stop" || message == "ON") {
      Serial.println("REMOTE EMERGENCY STOP - Disabling motor");
      digitalWrite(enPin, HIGH);  // Disable motor driver
      setStatusLED(false);        // Turn off status LED
      if (motorRunning) {
        motorRunning = false;
        motorSequenceTriggered = false;
        stepper.stop();           // Stop AccelStepper immediately
        stepper.setCurrentPosition(0); // Reset position
      }
      publishStatus("emergency_stop");  // Always publish emergency status
    } else if (message == "reset" || message == "OFF") {
      Serial.println("Remote emergency stop reset - Enabling motor");
      digitalWrite(enPin, LOW);   // Enable motor driver
      if (!motorRunning) {
        publishStatus("ready");
      }
    }
  } else if (String(topic) == mqtt_topic_steps) {
    // Handle step count changes
    int newSteps = message.toInt();
    if (newSteps > 0 && newSteps <= 50000) { // Reasonable limits
      // Only process if the value is actually different
      if (newSteps != desiredSteps) {
        desiredSteps = newSteps;
        saveStepsToEEPROM(desiredSteps);
        // Don't republish to avoid feedback loop - HA already knows the value
        Serial.print("Step count updated to: ");
        Serial.println(desiredSteps);
        publishStatus("steps_updated");
      } else {
        Serial.println("Step count unchanged - ignoring message");
      }
    } else {
      Serial.println("Invalid step count - must be 1-50000");
    }
  } else if (String(topic) == mqtt_topic_speed) {
    // Handle speed changes
    int newSpeed = message.toInt();
    if (newSpeed >= minSpeed && newSpeed <= maxSpeed) { // Speed limits
      // Only process if the value is actually different
      if (newSpeed != motorSpeed) {
        motorSpeed = newSpeed;
        saveSpeedToEEPROM(motorSpeed);
        // Don't republish to avoid feedback loop - HA already knows the value
        Serial.print("Motor speed updated to: ");
        Serial.println(motorSpeed);
        publishStatus("speed_updated");
      } else {
        Serial.println("Speed unchanged - ignoring message");
      }
    } else {
      Serial.print("Invalid speed - must be ");
      Serial.print(minSpeed);
      Serial.print("-");
      Serial.println(maxSpeed);
    }
  }
}

void publishStatus(const char* status) {
  if (client.connected()) {
    client.publish(mqtt_topic_status, status, true);  // Retain message
    Serial.print("Published status: ");
    Serial.println(status);
  }
}

void publishHeartbeat() {
  if (client.connected()) {
    // Create heartbeat payload with uptime
    String heartbeatPayload = String(millis() / 1000); // Uptime in seconds
    client.publish(mqtt_topic_heartbeat, heartbeatPayload.c_str(), false);  // Don't retain heartbeat
    Serial.print("Published heartbeat: ");
    Serial.print(heartbeatPayload);
    Serial.println(" seconds uptime");
  }
}

void runMotorSequence() {
  motorRunning = true;
  setStatusLED(true);  // Turn on status LED
  publishStatus("closing");
  delay(100);  // Give time for MQTT message to be sent
  
  Serial.println("Starting motor sequence");
  
  // Move motor forward (close door)
  moveMotorForward();
  
  // Wait 1 second
  Serial.println("Waiting 1 second...");
  delay(1000);
  
  // Move motor backward (unwind string)
  moveMotorBackward();
  
  // Sequence complete
  Serial.println("Motor sequence complete");
  setStatusLED(false);  // Turn off status LED
  publishStatus("ready");
  
  motorRunning = false;
}

void moveMotorForward() {
  Serial.println("Moving motor forward (closing door)");
  digitalWrite(enPin, LOW);  // Enable motor driver
  
  // Set up AccelStepper with constant speed (no acceleration)
  stepper.setMaxSpeed(motorSpeed);     // Use saved speed
  stepper.setAcceleration(motorSpeed * 10); // Very high acceleration = almost instant to max speed
  stepper.move(desiredSteps);          // Move forward by desiredSteps
  
  // Run the motor until it reaches the target
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    yield();  // Allow ESP32 to handle background tasks
    
    // Check for emergency stop during movement
    if (digitalRead(enPin) == HIGH) {
      Serial.println("Emergency stop detected during forward movement");
      break;
    }
  }
  
  digitalWrite(enPin, HIGH);  // Disable motor driver
  Serial.println("Forward movement complete");
}

void moveMotorBackward() {
  Serial.println("Moving motor backward (unwinding string)");
  digitalWrite(enPin, LOW);  // Enable motor driver
  
  // Set up AccelStepper with constant speed (no acceleration)
  stepper.setMaxSpeed(motorSpeed);     // Use saved speed
  stepper.setAcceleration(motorSpeed * 10); // Very high acceleration = almost instant to max speed
  stepper.move(-desiredSteps);         // Move backward by desiredSteps (negative)
  
  // Run the motor until it reaches the target
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    yield();  // Allow ESP32 to handle background tasks
    
    // Check for emergency stop during movement
    if (digitalRead(enPin) == HIGH) {
      Serial.println("Emergency stop detected during backward movement");
      break;
    }
  }
  
  digitalWrite(enPin, HIGH);  // Disable motor driver
  Serial.println("Backward movement complete");
}

void checkWiFiConnection() {
  // Check if WiFi is still connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost! Attempting to reconnect...");
    
    // Disconnect and try to reconnect
    WiFi.disconnect();
    delay(1000);
    
    WiFi.begin(ssid, password);
    
    // Try to reconnect for up to 30 seconds
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("");
      Serial.println("WiFi reconnected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("");
      Serial.println("WiFi reconnection failed. Will try again next cycle.");
    }
  }
}

void checkEmergencyStop() {
  // No physical button - emergency stop only via MQTT
  // This function is kept for future use if needed
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect with LWT (Last Will and Testament)
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_password, 
                      mqtt_topic_status, 0, true, "offline")) {
      Serial.println("connected");
      // Subscribe to the control topic
      client.subscribe(mqtt_topic_control);
      // Subscribe to the emergency stop topic
      client.subscribe(mqtt_topic_emergency);
      // Subscribe to the steps topic
      client.subscribe(mqtt_topic_steps);
      // Subscribe to the speed topic
      client.subscribe(mqtt_topic_speed);
      // Publish online status and current values
      publishStatus("ready");
      publishCurrentSteps();
      publishCurrentSpeed();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup Started");

  // Initialize EEPROM
  EEPROM.begin(512);  // Allocate 512 bytes for EEPROM
  
  // Load step count from EEPROM
  loadStepsFromEEPROM();
  
  // Load motor speed from EEPROM
  loadSpeedFromEEPROM();

  // Initialize LEDs and motor pins
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);  // Status LED pin
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);           // Disable the stepper driver initially
  digitalWrite(ONBOARD_LED_PIN, LOW);  // Turn off onboard LED initially
  setStatusLED(false);                 // Turn off status LED initially

  // AccelStepper doesn't need pinMode for step/dir pins - it handles them automatically
  // Motor speed and acceleration are set in the movement functions

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Wait for MQTT connection
  while (!client.connected()) {
    reconnect();
  }

  // Initialize heartbeat timer
  lastHeartbeat = millis();

  Serial.println("Setup Completed, Waiting for MQTT commands...");
}

void loop() {
  // Check WiFi connection
  checkWiFiConnection();
  
  // Handle MQTT connection (only if WiFi is connected)
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  } else {
    Serial.println("Waiting for WiFi connection...");
    delay(1000);
    return; // Skip the rest of the loop if no WiFi
  }

  // Check if motor sequence was triggered by MQTT
  if (motorSequenceTriggered) {
    motorSequenceTriggered = false;
    runMotorSequence();
  }

  // Send heartbeat periodically (only when not running motor)
  if (!motorRunning && millis() - lastHeartbeat >= heartbeatInterval) {
    publishHeartbeat();
    lastHeartbeat = millis();
  }
  
  // Small delay to prevent excessive CPU usage
  delay(10);
}
