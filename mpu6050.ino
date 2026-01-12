#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Use GPIO numbers on Generic ESP8266:
  // SDA = GPIO4
  // SCL = GPIO5
  Wire.begin(4, 5);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("MPU6050 connected successfully.");
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  Serial.print("Accel (X,Y,Z): ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);

  Serial.print("Gyro  (X,Y,Z): ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);

  Serial.println("----------------------");
  delay(500);
}
