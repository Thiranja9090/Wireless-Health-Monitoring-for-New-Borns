#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#define MA_WINDOW 4

MAX30105 sensor;

// Buffers
uint32_t irBuffer[100];
uint32_t redBuffer[100];

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Sensor not found");
    while (1);
  }

  sensor.setup();
  sensor.setPulseAmplitudeIR(0x2F);
  sensor.setPulseAmplitudeRed(0x2F);
  sensor.setPulseAmplitudeProximity(0);
  sensor.setLEDMode(2);  // Red + IR
  sensor.setSampleRate(50);
  sensor.setPulseWidth(411);

  Serial.println("MAX30102 ready");
}

void loop() {

  // Collect 100 samples (~1 sec)
  uint32_t smoothIR = 0;
  uint32_t smoothRED = 0;

  // ---- INSERT THIS WHOLE BLOCK ----
  for (int i = 0; i < 100; i++) {
    while (!sensor.available()) sensor.check();

    smoothIR  = (smoothIR  * (MA_WINDOW - 1) + sensor.getIR())  / MA_WINDOW;
    smoothRED = (smoothRED * (MA_WINDOW - 1) + sensor.getRed()) / MA_WINDOW;

    irBuffer[i]  = smoothIR;
    redBuffer[i] = smoothRED;

    sensor.nextSample();
  }
  uint32_t irValue  = irBuffer[99];
  uint32_t redValue = redBuffer[99];

  if (irValue < 20000 || redValue < 20000) {
    Serial.println("Weak signal...");
    return;   // skip this loop cycle
}

  // Declare BEFORE function call
  int32_t spo2;
  int8_t  spo2Valid;
  int32_t heartRate;
  int8_t  heartRateValid;

  // Correct function call
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    100,
    redBuffer,
    &spo2,
    &spo2Valid,
    &heartRate,
    &heartRateValid
  );

  // Print raw data and results
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print("  RED=");
  Serial.print(redValue);

  Serial.print("  HeartRate=");
  if (heartRateValid) Serial.print(heartRate);
  else Serial.print("Invalid");

  Serial.print("  SpO2=");
  if (spo2Valid) Serial.print(spo2);
  else Serial.print("Invalid");

  Serial.println();
}
