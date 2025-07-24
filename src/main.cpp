#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include "secrets.h"


// ===== MQTT TOPICS CONFIGURATION =====
// All MQTT topics defined in secrets.h for consistency
const char* mqtt_topic_control = MQTT_CONTROL_TOPIC;      // "chickendoor/control" - Command input
const char* mqtt_topic_status = MQTT_STATUS_TOPIC;        // "chickendoor/status" - Status output  
const char* mqtt_topic_heartbeat = MQTT_HEARTBEAT_TOPIC;  // "chickendoor/heartbeat" - Periodic alive signal
const char* mqtt_topic_steps = MQTT_STEPS_TOPIC;          // "chickendoor/steps" - Set number of steps (forward)
const char* mqtt_topic_speed = MQTT_SPEED_TOPIC;          // "chickendoor/speed" - Set motor speed
const char* mqtt_topic_backsteps = MQTT_BACKSTEPS_TOPIC;  // "chickendoor/backsteps" - Set backward steps for compensation
const char* mqtt_client_id = MQTT_CLIENT_ID;              // "ESP32ChickenDoor" - Client identifier

// ===== CONFIGURATION VARIABLES (SET YOUR DEFAULTS HERE) =====
// Motor settings - adjust these defaults to your preferred values
const int stepsPerRevolution = 200;  // Steps per revolution for your motor
const int defaultSteps = 6000;      // Default forward steps (adjust to your door)
const int defaultBackSteps = 6000;  // Default backward steps (start same as forward, adjust for compensation)
const int defaultSpeed = 2000;        // Default speed in steps/second (slower than before)
const int minSpeed = 10;             // Minimum allowed speed
const int maxSpeed = 5000;            // Maximum allowed speed

// Validation limits
const int maxStepsLimit = 25000;     // Maximum allowed step count for safety
const int minStepsLimit = 100;       // Minimum allowed step count (1 full revolution)

// EEPROM addresses for persistent storage
const int EEPROM_STEPS_ADDR = 0;     // Address to store forward step count
const int EEPROM_SPEED_ADDR = 8;     // Address to store speed
const int EEPROM_MAGIC_ADDR = 12;    // Address to store magic number (validates EEPROM)
const int EEPROM_BACKSTEPS_ADDR = 16; // Address to store backward step count
const uint32_t EEPROM_MAGIC_VALUE = 0xABCD1234; // Magic number to check if EEPROM is initialized

// Motor control variables (loaded from EEPROM)
int desiredSteps = defaultSteps;     // Current forward step count
int backwardSteps = defaultBackSteps; // Current backward step count
int motorSpeed = defaultSpeed;       // Current motor speed
// =============================================================

// Pin assignments
const int dirPin = 32;
const int stepPin = 25;
const int enablePin = 27;  // Motor driver enable pin (active LOW - LOW=enabled, HIGH=disabled/sleep)

// Create AccelStepper object (DRIVER mode for step/direction)
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// LED pins
#define ONBOARD_LED_PIN 2  // Onboard LED on the ESP32
#define STATUS_LED_PIN 26  // External status LED (stepPin=25, dirPin=32, enablePin=27 are used by stepper)

// Heartbeat settings
const unsigned long heartbeatInterval = 30000;  // Send heartbeat every 30 seconds
unsigned long lastHeartbeat = 0;

// WiFi reconnection settings
const unsigned long wifiReconnectDeadTime = 30000;  // Force disconnect after 30 seconds
unsigned long wifiLostTime = 0;

// Motor state machine
enum MotorState {
  IDLE,
  MOVING_FWD, 
  DWELL,
  MOVING_REV
};

MotorState motorState = IDLE;
unsigned long motorStateStartTime = 0;
bool motorSequenceTriggered = false;  // Flag to trigger the motor sequence

// WiFi and MQTT connection settings
const char* ssid = WIFI_SSID;              // From secrets.h
const char* password = WIFI_PASSWORD;      // From secrets.h
const char* mqtt_server = MQTT_SERVER;     // From secrets.h
const int mqtt_port = MQTT_PORT;           // From secrets.h
const char* mqtt_user = MQTT_USER;         // From secrets.h
const char* mqtt_password = MQTT_PASSWORD; // From secrets.h

WiFiClient espClient;
PubSubClient client(espClient);

// Motor control
bool motorRunning = false;  // Track if motor is currently running

// Fixed buffer for MQTT messages to avoid heap fragmentation
char mqttBuffer[32];

// EEPROM write optimization to avoid unnecessary flash wear
template<typename T>
void eepromWriteIfChanged(int addr, const T& val) {
  T old;
  EEPROM.get(addr, old);
  if (memcmp(&old, &val, sizeof(T)) != 0) {
    EEPROM.put(addr, val);
    EEPROM.commit(); // Single page write
  }
}

// Function declarations
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void publishStatus(const char* status);
void publishHeartbeat();
void runMotorSequence();
void updateMotorStateMachine();
void reconnect();
void checkWiFiConnection();
void loadStepsFromEEPROM();
void saveStepsToEEPROM(int steps);
void publishCurrentSteps();
void loadBackStepsFromEEPROM();
void saveBackStepsToEEPROM(int steps);
void publishCurrentBackSteps();
void loadSpeedFromEEPROM();
void saveSpeedToEEPROM(int speed);
void publishCurrentSpeed();
void setStatusLED(bool on);

void loadStepsFromEEPROM() {
  // Check if EEPROM has been initialized
  uint32_t magicValue;
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
  eepromWriteIfChanged(EEPROM_STEPS_ADDR, steps);
  eepromWriteIfChanged(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  Serial.print("Saved steps to EEPROM: ");
  Serial.println(steps);
}

void publishCurrentSteps() {
  if (client.connected()) {
    snprintf(mqttBuffer, sizeof(mqttBuffer), "%d", desiredSteps);
    client.publish(mqtt_topic_steps, mqttBuffer, true);  // Retain message
    Serial.print("Published current forward steps: ");
    Serial.println(desiredSteps);
  }
}

void loadBackStepsFromEEPROM() {
  // Check if EEPROM has been initialized
  uint32_t magicValue;
  EEPROM.get(EEPROM_MAGIC_ADDR, magicValue);
  
  if (magicValue == EEPROM_MAGIC_VALUE) {
    // EEPROM is initialized, load the saved backward steps
    EEPROM.get(EEPROM_BACKSTEPS_ADDR, backwardSteps);
    Serial.print("Loaded backward steps from EEPROM: ");
    Serial.println(backwardSteps);
  } else {
    // First time boot, save default backward steps
    backwardSteps = defaultBackSteps;
    saveBackStepsToEEPROM(backwardSteps);
    Serial.print("First boot - using default backward steps: ");
    Serial.println(backwardSteps);
  }
}

void saveBackStepsToEEPROM(int steps) {
  eepromWriteIfChanged(EEPROM_BACKSTEPS_ADDR, steps);
  eepromWriteIfChanged(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  Serial.print("Saved backward steps to EEPROM: ");
  Serial.println(steps);
}

void publishCurrentBackSteps() {
  if (client.connected()) {
    snprintf(mqttBuffer, sizeof(mqttBuffer), "%d", backwardSteps);
    client.publish(mqtt_topic_backsteps, mqttBuffer, true);  // Retain message
    Serial.print("Published current backward steps: ");
    Serial.println(backwardSteps);
  }
}

void loadSpeedFromEEPROM() {
  // Check if EEPROM has been initialized
  uint32_t magicValue;
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
  eepromWriteIfChanged(EEPROM_SPEED_ADDR, speed);
  eepromWriteIfChanged(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  Serial.print("Saved speed to EEPROM: ");
  Serial.println(speed);
  
  // Update stepper max speed only - acceleration stays minimal
  stepper.setMaxSpeed(speed);
}

void publishCurrentSpeed() {
  if (client.connected()) {
    snprintf(mqttBuffer, sizeof(mqttBuffer), "%d", motorSpeed);
    client.publish(mqtt_topic_speed, mqttBuffer, true);  // Retain message
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
  
  // Use fixed buffer to avoid heap fragmentation
  char message[64];
  unsigned int copyLen = (length < sizeof(message) - 1) ? length : sizeof(message) - 1;
  memcpy(message, payload, copyLen);
  message[copyLen] = '\0';
  Serial.println(message);

  if (String(topic) == mqtt_topic_control && !motorRunning) {
    if (strcmp(message, "close") == 0) {
      Serial.println("Close command received");
      motorSequenceTriggered = true;
    } else {
      Serial.println("Unknown command or motor busy");
    }
  } else if (String(topic) == mqtt_topic_steps) {
    // Reject parameter changes while motor is running
    if (motorRunning) {
      Serial.println("Motor running - rejecting step count change");
      return;
    }
    
    // Handle step count changes
    int newSteps = atoi(message);
    if (newSteps >= minStepsLimit && newSteps <= maxStepsLimit) { // Use defined limits
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
      Serial.print("Invalid step count - must be ");
      Serial.print(minStepsLimit);
      Serial.print("-");
      Serial.println(maxStepsLimit);
    }
  } else if (String(topic) == mqtt_topic_backsteps) {
    // Reject parameter changes while motor is running
    if (motorRunning) {
      Serial.println("Motor running - rejecting backward step count change");
      return;
    }
    
    // Handle backward step count changes
    int newBackSteps = atoi(message);
    if (newBackSteps >= minStepsLimit && newBackSteps <= maxStepsLimit) { // Use defined limits
      // Only process if the value is actually different
      if (newBackSteps != backwardSteps) {
        backwardSteps = newBackSteps;
        saveBackStepsToEEPROM(backwardSteps);
        // Don't republish to avoid feedback loop - HA already knows the value
        Serial.print("Backward step count updated to: ");
        Serial.println(backwardSteps);
        publishStatus("backsteps_updated");
      } else {
        Serial.println("Backward step count unchanged - ignoring message");
      }
    } else {
      Serial.print("Invalid backward step count - must be ");
      Serial.print(minStepsLimit);
      Serial.print("-");
      Serial.println(maxStepsLimit);
    }
  } else if (String(topic) == mqtt_topic_speed) {
    // Reject parameter changes while motor is running
    if (motorRunning) {
      Serial.println("Motor running - rejecting speed change");
      return;
    }
    
    // Handle speed changes
    int newSpeed = atoi(message);
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
    // Create heartbeat payload with uptime using fixed buffer
    snprintf(mqttBuffer, sizeof(mqttBuffer), "%lu", millis() / 1000);
    client.publish(mqtt_topic_heartbeat, mqttBuffer, false);  // Don't retain heartbeat
    Serial.print("Published heartbeat: ");
    Serial.print(mqttBuffer);
    Serial.println(" seconds uptime");
  }
}

void runMotorSequence() {
  motorRunning = true;
  setStatusLED(true);  // Turn on status LED
  publishStatus("closing");
  
  // Enable motor driver (active LOW)
  digitalWrite(enablePin, LOW);
  delay(50);  // Give driver time to wake up
  
  Serial.println("Starting motor sequence");
  motorState = MOVING_FWD;
  motorStateStartTime = millis();
  
  // Update speed and start forward movement
  stepper.setMaxSpeed(motorSpeed);
  stepper.move(desiredSteps);
}

void updateMotorStateMachine() {
  unsigned long currentTime = millis();
  
  switch (motorState) {
    case IDLE:
      // Check if motor sequence was triggered
      if (motorSequenceTriggered) {
        motorSequenceTriggered = false;
        runMotorSequence();
      }
      break;
      
    case MOVING_FWD:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        Serial.println("Forward movement complete");
        motorState = DWELL;
        motorStateStartTime = currentTime;
      }
      break;
      
    case DWELL:
      if (currentTime - motorStateStartTime >= 1000) { // Wait 1 second
        Serial.println("Moving motor backward (unwinding string)");
        Serial.print("Forward steps completed: ");
        Serial.println(desiredSteps);
        Serial.print("Current position: ");
        Serial.println(stepper.currentPosition());
        Serial.print("Backward steps to move: ");
        Serial.println(backwardSteps);
        
        // Debug: Check direction pin state before direction change
        Serial.print("Direction pin before change: ");
        Serial.println(digitalRead(dirPin));
        
        motorState = MOVING_REV;
        stepper.setMaxSpeed(motorSpeed);
        
        // Try using move() with negative value instead of moveTo()
        int moveSteps = -backwardSteps;  // Make sure it's negative
        Serial.print("Moving steps value: ");
        Serial.println(moveSteps);
        
        stepper.move(moveSteps);
        
        Serial.print("Stepper target position set to: ");
        Serial.println(stepper.targetPosition());
        Serial.print("Distance to go: ");
        Serial.println(stepper.distanceToGo());
        
        // Debug: Check direction pin state after setting new target
        Serial.print("Direction pin after change: ");
        Serial.println(digitalRead(dirPin));
        
        // Check if distance to go is actually negative (which means backward)
        if (stepper.distanceToGo() > 0) {
          Serial.println("WARNING: distanceToGo is POSITIVE - this means FORWARD movement!");
        } else if (stepper.distanceToGo() < 0) {
          Serial.println("GOOD: distanceToGo is NEGATIVE - this means BACKWARD movement!");
        } else {
          Serial.println("ERROR: distanceToGo is ZERO - no movement!");
        }
        
        // Force a step to ensure direction change takes effect
        delay(1);
        stepper.run();
        Serial.print("Direction pin after first step: ");
        Serial.println(digitalRead(dirPin));
      }
      break;
      
    case MOVING_REV:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        Serial.println("Backward movement complete");
        Serial.print("Final stepper position: ");
        Serial.println(stepper.currentPosition());
        Serial.print("Expected final position: ");
        Serial.println(desiredSteps - backwardSteps);
        Serial.print("Position difference from start: ");
        Serial.println(stepper.currentPosition() - 0);
        Serial.println("Motor sequence complete");
        
        // Clear position for next cycle
        stepper.setCurrentPosition(0);
        
        // Disable motor driver to save power (active LOW - HIGH=disabled)
        digitalWrite(enablePin, HIGH);
        
        setStatusLED(false);  // Turn off status LED
        publishStatus("ready");
        motorRunning = false;
        motorState = IDLE;
      }
      break;
  }
}

void checkWiFiConnection() {
  // Check if WiFi is still connected
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiLostTime == 0) {
      // First time we notice WiFi is down - let it try automatic reconnection
      wifiLostTime = millis();
      Serial.println("WiFi connection lost! Waiting for automatic reconnection...");
    } else if (millis() - wifiLostTime >= wifiReconnectDeadTime) {
      // WiFi has been down for too long, force a reconnection
      Serial.println("WiFi down too long, forcing reconnection...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, password);
      wifiLostTime = millis(); // Reset timer for this attempt
    }
  } else {
    // WiFi is connected, reset lost time
    if (wifiLostTime != 0) {
      Serial.println("WiFi reconnected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      wifiLostTime = 0;
    }
  }
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
      // Subscribe to the steps topic
      client.subscribe(mqtt_topic_steps);
      // Subscribe to the backward steps topic
      client.subscribe(mqtt_topic_backsteps);
      // Subscribe to the speed topic
      client.subscribe(mqtt_topic_speed);
      // Publish online status and current values
      publishStatus("ready");
      publishCurrentSteps();
      publishCurrentBackSteps();
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
  
  // Load backward step count from EEPROM
  loadBackStepsFromEEPROM();
  
  // Load motor speed from EEPROM
  loadSpeedFromEEPROM();

  // Initialize LEDs and motor pins
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);  // Status LED pin
  pinMode(enablePin, OUTPUT);       // Motor driver enable pin
  digitalWrite(ONBOARD_LED_PIN, LOW);  // Turn off onboard LED initially
  setStatusLED(false);                 // Turn off status LED initially
  digitalWrite(enablePin, HIGH);       // Disable motor driver initially (save power)

  // AccelStepper doesn't need pinMode for step/dir pins - it handles them automatically
  // Configure AccelStepper once during setup
  stepper.setMaxSpeed(motorSpeed);      // Set initial max speed
  stepper.setAcceleration(50);          // Very low acceleration for smoothest movement
  stepper.setCurrentPosition(0);        // Start at position 0

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

  // Run motor state machine (non-blocking)
  updateMotorStateMachine();

  // Send heartbeat periodically (runs continuously now)
  if (millis() - lastHeartbeat >= heartbeatInterval) {
    publishHeartbeat();
    lastHeartbeat = millis();
  }
  
  // Small delay to prevent excessive CPU usage
  delay(10);
}
