# Environmental Monitoring System

This project is a real-time environmental and motion monitoring system based on the ESP32 with FreeRTOS. It features a TFT display, Wi-Fi connectivity, temperature and acceleration sensing, and visual status indication via an RGB LED.

## 🔧 Features

- 📶 Wi-Fi connection with automatic reconnection
- 🌐 NTP time synchronization (UTC-3)
- 🌡️ Temperature reading (BMP280)
- 🧭 3-axis acceleration monitoring (MMA8452Q)
- 🖥️ 240x240 TFT display with 7-segment styled digital clock
- 🎨 Graphical display of data with FreeRTOS-controlled tasks
- 🔴🟡🟢 RGB LED status indicator using FastLED

## ⚙️ Architecture

- Multitasking with FreeRTOS (`TaskWiFiConnect`, `TaskDateTime`, `TaskReadSensors`, etc.)
- Inter-task communication via `QueueHandle_t`
- Display rendering protected by `SemaphoreHandle_t`

## 📚 Dependencies

- [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI)
- [FastLED](https://github.com/FastLED/FastLED)
- [Adafruit BMP280 Library](https://github.com/adafruit/Adafruit_BMP280_Library)
- [RoboCore MMA8452Q](https://github.com/RoboCore/MMA8452Q)
- `WiFi.h`, `NTPClient.h`, `Wire.h`, `SPI.h`

## 📦 Hardware

- ESP32 Dev Module
- 1x TFT 240x240 display (SPI)
- 1x BMP280 sensor (I2C)
- 1x MMA8452Q accelerometer (I2C)
- 1x RGB LED (Neopixel - GPIO 19)

## 📸 Preview

> [Add image or GIF here of the display in action]

## 🛠️ Setup

1. Install libraries listed above via Library Manager or git.
2. Connect peripherals according to pin mapping.
3. Set your Wi-Fi credentials in the code (`ssid`, `password`).
4. Upload using Arduino IDE or PlatformIO.

## 📄 License

MIT License
