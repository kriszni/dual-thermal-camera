// main_mlx.cpp — MLX90640 firmware for XIAO ESP32-S3
// Streams binary packets the dual_thermal_recorder.py viewer understands:
//   [AA 55 A5 5A][u32 frameId][768 x f32 temps][u16 checksum]
// checksum = sum of payload bytes (frameId + pixels) & 0xFFFF
// Serial commands: 'S' = start streaming, 'P' = pause

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>

// I2C pins — change these if your MLX90640 is wired to different XIAO pins
static const int kPinSda = D4;
static const int kPinScl = D5;

static const uint8_t MAGIC[4] = {0xAA, 0x55, 0xA5, 0x5A};
static const int NPIX = 32 * 24;

Adafruit_MLX90640 mlx;
static float frame[NPIX];
static uint32_t frameId = 0;
static bool streaming = true;

void setup() {
  Serial.begin(921600);
  delay(1000);

  Wire.begin(kPinSda, kPinScl);
  Wire.setClock(800000);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    while (true) {
      Serial.println("# MLX90640 not found on I2C - check wiring");
      delay(1000);
    }
  }
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_4_HZ);
}

void loop() {
  // handle start/stop commands from the Python viewer
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') streaming = true;
    else if (c == 'P') streaming = false;
  }
  if (!streaming) { delay(20); return; }

  if (mlx.getFrame(frame) != 0) return;  // frame not ready / read error

  frameId++;
  uint16_t checksum = 0;
  const uint8_t* idBytes = (const uint8_t*)&frameId;
  const uint8_t* pxBytes = (const uint8_t*)frame;
  for (int i = 0; i < 4; i++) checksum += idBytes[i];
  for (size_t i = 0; i < sizeof(frame); i++) checksum += pxBytes[i];

  Serial.write(MAGIC, 4);
  Serial.write(idBytes, 4);
  Serial.write(pxBytes, sizeof(frame));
  Serial.write((const uint8_t*)&checksum, 2);
}