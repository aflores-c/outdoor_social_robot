#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// BNO055 I2C pins on Nano ESP32
// A0 -> SCL
// A1 -> SDA

#define SDA_PIN A1
#define SCL_PIN A0

#define DEG_TO_RAD 0.017453292519943295

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

unsigned long previousMillis = 0;
const int imu_rate_hz = 100;
const int interval_ms = 1000 / imu_rate_hz;

void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    delay(10);
  }

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno.begin())
  {
    while (1)
    {
      delay(100);
    }
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval_ms)
  {
    previousMillis = currentMillis;

    imu::Quaternion quat = bno.getQuat();

    // BNO055 gyro from Adafruit is usually deg/s
    imu::Vector<3> gyro =
        bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // Raw acceleration INCLUDING gravity, required by LIO-SAM
    imu::Vector<3> accel =
        bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    float gx = gyro.x() * DEG_TO_RAD;
    float gy = gyro.y() * DEG_TO_RAD;
    float gz = gyro.z() * DEG_TO_RAD;

    Serial.print(currentMillis);
    Serial.print(",");

    Serial.print(quat.x(), 6);
    Serial.print(",");
    Serial.print(quat.y(), 6);
    Serial.print(",");
    Serial.print(quat.z(), 6);
    Serial.print(",");
    Serial.print(quat.w(), 6);
    Serial.print(",");

    Serial.print(gx, 6);
    Serial.print(",");
    Serial.print(gy, 6);
    Serial.print(",");
    Serial.print(gz, 6);
    Serial.print(",");

    Serial.print(accel.x(), 6);
    Serial.print(",");
    Serial.print(accel.y(), 6);
    Serial.print(",");
    Serial.println(accel.z(), 6);
  }
}