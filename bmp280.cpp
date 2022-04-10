#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "sensordata.h"

float temperature; // Temperature in ºC
float humidity; // Relative humidity in %
float pressure; // Pressure in Pa

// -----------------I2C-----------------
#define I2C_SDA 21 // SDA Connected to GPIO 21
#define I2C_SCL 22 // SCL Connected to GPIO 22
TwoWire I2CSensors = TwoWire(0);

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

void setupBMP(void)
{
  // Serial.begin(9600);
  Serial.println(F("BME280 Sensor event test"));

  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);

  // BME 280 (0x77 or 0x76 will be the address)
  if (!bme.begin(0x76, &I2CSensors))
  {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1)
      delay(10);
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
}

void loopBMP(void)
{
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  temperature = temp_event.temperature;
  humidity = humidity_event.relative_humidity;
  pressure = pressure_event.pressure;
}
