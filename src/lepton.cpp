#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "lepton.h"

static const int kPinCs   = D7;
static const int kPinSck  = D8;
static const int kPinMiso = D9;
static const int kPinSda  = D4;
static const int kPinScl  = D5;
static const int kPinRst  = -1;

static const int    WIDTH       = 160;
static const int    HEIGHT      = 120;
static const size_t FRAME_BYTES = WIDTH * HEIGHT * 2;
static const uint8_t HEADER[4]  = {0xDE, 0xAD, 0xBE, 0xEF};

SPIClass spi(FSPI);
TwoWire  i2c(0);
FlirLepton lepton(i2c, spi, kPinCs, kPinRst);

static uint8_t frameBuf[FRAME_BYTES];

void setup() {
  Serial.begin(921600);
  delay(2000);

  spi.begin(kPinSck, kPinMiso, -1, -1);
  i2c.begin(kPinSda, kPinScl, 400000);

  lepton.begin();               // resets the Lepton
  while (!lepton.isReady()) {   // poll until Lepton reports ready
    delay(50);
  }
  // No setVideoParameters() call needed: the library defaults to
  // 160x120 (Lepton 3.x). Only required if using a Lepton 2.x sensor.
}

void loop() {
  bool gotFrame = lepton.readVoSpi(FRAME_BYTES, frameBuf);
  if (!gotFrame) return;   // still assembling / mid-resync, not a full frame yet

  Serial.write(HEADER, 4);
  Serial.write(frameBuf, FRAME_BYTES);
}

// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include "lepton.h"

// static const int kPinCs   = D7;
// static const int kPinSck  = D8;
// static const int kPinMiso = D9;
// static const int kPinSda  = D4;
// static const int kPinScl  = D5;
// static const int kPinRst  = -1;

// static const int    WIDTH       = 80;
// static const int    HEIGHT      = 60;
// static const size_t FRAME_BYTES = WIDTH * HEIGHT * 2;
// static const uint8_t HEADER[4]  = {0xDE, 0xAD, 0xBE, 0xEF};

// SPIClass spi(FSPI);
// TwoWire  i2c(0);
// FlirLepton lepton(i2c, spi, kPinCs, kPinRst);

// static uint8_t frameBuf[FRAME_BYTES];

// void setup() {
//   Serial.begin(921600);
//   delay(2000);
//   spi.begin(kPinSck, kPinMiso, -1, -1);
//   i2c.begin(kPinSda, kPinScl, 400000);
//   lepton.begin();
//   uint32_t start = millis();
//   while (!lepton.isReady() && (millis() - start < 5000)) delay(50);
//   lepton.setVideoParameters(2, WIDTH, HEIGHT, WIDTH * 2, HEIGHT, 1);
// }

// void loop() {
//   bool ok = lepton.readVoSpi(FRAME_BYTES, frameBuf);
//   if (!ok) return;
//   Serial.write(HEADER, 4);
//   Serial.write(frameBuf, FRAME_BYTES);
// }
