# ESP32 Sensor and Control System

This project involves the use of an ESP32 microcontroller to read various sensors, control outputs, and communicate using ESP-NOW. The system is designed to enter deep sleep mode to save power when specific conditions are met.

## Features

- *Sensor Reading*: Reads data from multiple sensors including soil moisture, pH, temperature, and battery voltage.
- *ESP-NOW Communication*: Communicates with a gateway using ESP-NOW protocol.
- *Deep Sleep Mode*: Enters deep sleep mode to save power when not actively processing data.
- *Button Interaction*: Uses buttons to control and configure the device.
- *Data Display*: Displays information on a TM1637 4-digit 7-segment display.

## Components Used

- ESP32
- Soil Moisture Sensor
- pH Sensor
- DS18B20 Temperature Sensor
- TM1637 4-Digit 7-Segment Display
- Buttons
- LEDs

## Installation

1. *Clone the repository*
    bash
    git clone [https://github.com/txfav/node_monitoring.git]
    
2. *Open the project in Arduino IDE or PlatformIO*

3. *Install required libraries*:
    - OneButton
    - esp_wifi
    - WiFi
    - EEPROM
    - esp_now
    - OneWire
    - DallasTemperature
    - TM1637Display
    - SoftwareSerial
    - freertos/FreeRTOS.h
    - freertos/task.h
    - freertos/semphr.h
    - driver/rtc_io.h

4. *Upload the code* to your ESP32.

## Usage

### Configuration

- *Button 1 (PIN_BUTTON1)*:
  - Single click: Increase bedengan or node number.
  - Double click: Toggle between editing bedengan and node.
  - Long press: Enter or exit editing mode.

- *Button 2 (PIN_BUTTON2)*:
  - Single click: Scan for WiFi networks and attempt pairing.
  - Double click: Send pairing request if peer is not added.
  - Long press: Reset device settings and enter editing mode.

### Sleep Mode

- The ESP32 will enter deep sleep mode if:
  - A DATA message is successfully sent.
  - No data is sent within 30 seconds after pairing.

### Sensor Reading and Data Sending

- Sensor data is read periodically and sent to the gateway.
- The device communicates with the gateway using ESP-NOW protocol.

### Display

- The TM1637 4-digit 7-segment display shows the current bedengan and node values during configuration.

## Functions

### Initialization

- *setup()*: Initializes the hardware, sensors, buttons, and communication protocols.

### Main Loop

- *loop()*: Empty loop since the device relies on FreeRTOS tasks.

### Tasks

- *buttonTask()*: Handles button interactions.
- *sensorBatt()*: Reads battery voltage.
- *bacaSoilpH()*: Reads soil moisture and pH.
- *bacaSuhu()*: Reads temperature.
- *sendData()*: Sends collected sensor data via ESP-NOW.
- *segment()*: Updates the 7-segment display.

### Callbacks

- *OnDataSent()*: Callback for handling the status of data sent via ESP-NOW.
- *OnDataRecv()*: Callback for handling received data via ESP-NOW.
- *sleepTimeout()*: Timer callback for entering deep sleep mode after timeout.



## Contact

For any inquiries or issues, please contact [faisal.aviva12@gmail.com](mailto:faisal.aviva12@gmail.com).
