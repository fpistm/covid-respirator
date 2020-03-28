#include <Arduino.h>
#include <Wire.h>

#define DEBUG // décommenter pour envoyer les messages de debug en série
// #define DEBUG_VERBOSE
// #define SIMULATION // décommenter pour simuler des valeurs de capteur de pression au lieu de lire les vraies
#define HONEYWELL_I2C_ADDRESS 0x08
#define PRESSURE_MIN 0
#define PRESSURE_MAX 1
#define OUTPUT_MIN 1638 
#define OUTPUT_MAX 14745 

void read_raw_values()
{
  // Serial.print(" ");
  uint8_t x, y, s;
  Wire.requestFrom(HONEYWELL_I2C_ADDRESS, (uint8_t)2); //a est l'adresse i2C du capteur (souvent 0x28)
  int count = 0;
  // Serial.println(Wire.available());
  while (!Wire.available() && count < 100)
  {
    count++;
    delay(1);
  }
  if (Wire.available()) {
    x = Wire.read(); //Read the first byte
    y = Wire.read(); //Read the second byte
    s = x >> 6;      //s is the 2 first bits and gives the state of the sensor. Normal is s=0
    if (s == 0)
    { //Read the values from the I2C bus
      long pressure_raw = (((uint16_t)(x & 0x3f)) << 8) | y;
      double pressure = PRESSURE_MIN + ((pressure_raw - OUTPUT_MIN) * (PRESSURE_MAX - PRESSURE_MIN)) / ((OUTPUT_MAX - OUTPUT_MIN) * 1.0);
      Serial.println(pressure * 703.08893732448);
    }
  }
}

void setup()
{

  Serial.begin(9600);
  Serial.println("Hello");
  Wire.begin();
}

void loop()
{
  read_raw_values();
  delay(100);
}
/********************************************/
// Fin du cycle
/********************************************/