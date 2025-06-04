# ESP-NOW UART Hub

A bridge component that receives sensor data via ESP-NOW wireless protocol and forwards it to an MQTT hub via UART communication. This is part of the FAO56 IoT system for agricultural monitoring.

## Overview

The ESP-NOW UART Hub acts as an intermediary between wireless sensor nodes and the main MQTT hub. It receives sensor data from multiple ESP-NOW enabled sensor nodes and relays this data through a UART connection to the MQTT hub for further processing and cloud transmission.

## Architecture

```
[Sensor Nodes] ---> [ESP-NOW UART Hub] ---> [UART] ---> [MQTT Hub] ---> [Cloud/Broker]
    (Wireless)           (This Device)      (Serial)     (WiFi/MQTT)
```

## Features

- **ESP-NOW Communication**: Receives data from multiple sensor nodes wirelessly
- **Automatic Pairing**: Handles pairing requests from new sensor nodes
- **UART Bridge**: Forwards sensor data to MQTT hub via serial communication
- **WiFi Credential Management**: Receives and applies WiFi credentials from MQTT hub
- **FreeRTOS Multitasking**: Concurrent handling of ESP-NOW and UART communication
- **Queue-based Data Handling**: Reliable data buffering and transmission

## Hardware Requirements

- ESP32 development board (NodeMCU-32S or similar)
- UART connection to MQTT hub device
- Power supply (3.3V or 5V depending on board)

## Pin Configuration

```cpp
#define RX_HUB 26    // UART RX pin for MQTT hub communication
#define TX_HUB 27    // UART TX pin for MQTT hub communication
#define BAUD_RATE 115200
```

## Data Structures

### Sensor Data
```cpp
struct sensorData {
    uint8_t msgType;    // Message type (DATA or PAIRING)
    char nodeID[8];     // Sensor node identifier
    float temp;         // Temperature reading
    float humidity;     // Humidity reading
    long moisture;      // Soil moisture reading
};
```

### Pairing Structure
```cpp
struct structPairing {
    uint8_t msgType;    // PAIRING message type
    char nodeID[8];     // Node identifier
    uint8_t macAddr[6]; // MAC address
    uint8_t channel;    // WiFi channel
};
```

## Operation Flow

1. **Initialization**: Sets up ESP-NOW, UART communication, and FreeRTOS tasks
2. **WiFi Setup**: Receives WiFi credentials from MQTT hub and connects to network
3. **Pairing Process**: Handles pairing requests from sensor nodes automatically
4. **Data Reception**: Receives sensor data from paired nodes via ESP-NOW
5. **Data Forwarding**: Queues and transmits data to MQTT hub via UART

## Tasks

### Main Tasks
- **startConnection**: Handles WiFi connection and ESP-NOW initialization
- **sendSerial**: Processes queued sensor data and sends to MQTT hub
- **handleUartCommands**: Processes commands from MQTT hub

## Communication Protocol

### With Sensor Nodes (ESP-NOW)
- Receives pairing requests and sensor data
- Responds to pairing with hub MAC address and channel
- Maintains peer connections for data reception

### With MQTT Hub (UART)
- Receives WiFi credentials (marked with '`')
- Responds to test connections (T -> A)
- Forwards sensor data as binary structs
- Confirms WiFi credential receipt with 'W'

## Building and Deployment

### Prerequisites
- PlatformIO IDE or Arduino IDE with ESP32 support
- ESP32 board package installed

### Build Configuration
The project uses PlatformIO with configuration in `platformio.ini`:
- Target: ESP32 NodeMCU-32S
- Framework: Arduino
- Monitor speed: 115200 baud

### Upload
1. Connect ESP32 to computer via USB
2. Select correct COM port
3. Build and upload firmware
4. Monitor serial output for debugging

## Configuration
The config file is located in `include/config.h`

### Hub ID
Default hub identifier is set in config.h:
```cpp
#define HUB_ID "H-0"
```

### UART Pins
Modify pin assignments in config.h if needed:
```cpp
#define RX_HUB 26
#define TX_HUB 27
```

## Debugging

The system provides extensive serial debugging output:
- ESP-NOW initialization status
- Pairing process details
- Data reception and forwarding
- WiFi connection status
- UART communication logs

Monitor at 115200 baud to view debug information.

## Integration

This hub works in conjunction with:
- **Sensor Nodes**: ESP32-based devices running the fao56-sensor-node firmware
- **MQTT Hub**: The fao56-uart-mqtt-hub for cloud connectivity

## Troubleshooting

### Common Issues
1. **No sensor data received**: Check ESP-NOW pairing and channel settings
2. **UART communication failed**: Verify pin connections and baud rate
3. **WiFi connection issues**: Ensure credentials are properly received from MQTT hub
4. **Memory issues**: Monitor task stack sizes and queue lengths

### LED Indicators
- Built-in LED activity indicates various operational states
- Check serial monitor for detailed status information

## License

Part of the FAO56 IoT agricultural monitoring system.