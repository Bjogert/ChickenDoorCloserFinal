# Chicken Door Controller

ESP32-based automatic chicken door controller with MQTT integration for Home Assistant.

## Features

- **Automatic door closing**: Winds string to close door, then unwinds for manual opening
- **MQTT integration**: Status updates and remote control via Home Assistant
- **Manual override**: Physical button for testing/emergency use
- **Power management**: WiFi disconnects during motor operation for reliable stepper control
- **Status reporting**: Real-time updates ("closing", "ready", "offline")
- **Last Will Testament**: Automatic offline detection

## Hardware Requirements

- ESP32 (Wemos D1 Mini ESP32 or compatible)
- NEMA 23 stepper motor with driver
- Manual push button
- String/cable system for door mechanism

## Pin Configuration

```cpp
const int dirPin = 32;      // Stepper direction
const int stepPin = 25;     // Stepper step
const int enPin = 27;       // Stepper enable
const int buttonPin = 33;   // Manual button (to ground)
```

## Setup Instructions

### 1. Create Secrets File

Copy `src/secrets.h.template` to `src/secrets.h` and update with your credentials:

```cpp
#define WIFI_SSID "YourWiFiNetwork"
#define WIFI_PASSWORD "YourWiFiPassword"
#define MQTT_SERVER "192.168.1.100"
#define MQTT_USER "your_mqtt_user"
#define MQTT_PASSWORD "your_mqtt_password"
```

### 2. Build and Upload

```bash
pio run --target upload
pio device monitor
```

## MQTT Topics

- **Control**: `chickendoor/control` - Send `"close"` to trigger door closing
- **Status**: `chickendoor/status` - Receives `"closing"`, `"ready"`, or `"offline"`

## Home Assistant Integration

```yaml
mqtt:
  button:
    - name: "Close Chicken Door"
      command_topic: "chickendoor/control"
      payload_press: "close"
      
  sensor:
    - name: "Chicken Door Status"
      state_topic: "chickendoor/status"
```

## Security

- `secrets.h` is excluded from git via `.gitignore`
- Never commit credentials to version control
- Use the template file for sharing/deployment

## Motor Operation

1. Receives "close" command
2. Disconnects WiFi for power management
3. Runs stepper motor forward (winds string)
4. Waits 1 second
5. Runs stepper motor backward (unwinds string)
6. Reconnects WiFi and reports "ready"

## Troubleshooting

- **Motor doesn't move**: Check power supply and driver wiring
- **WiFi issues**: Check credentials in `secrets.h`
- **MQTT not working**: Verify broker settings and network connectivity
- **Watchdog resets**: Normal during motor operation due to power management
