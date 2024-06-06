#include <DHT.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

enum SensorType {
  SENSOR_DHT,
  SENSOR_MQ9,
  SENSOR_MQ4,
  SENSOR_MQ5,
  SENSOR_MQ6,
  SENSOR_NEW_TEMP,
  SENSOR_BMP,
  SENSOR_MOTION,
  SENSOR_DB,
  SENSOR_NONE
};

void printSensorData();
void configurePins(int slot, SensorType sensorType);
void disconnectSensor(int slot);
bool isInRange(int readings[], int lower, int upper);

#define SEALEVELPRESSURE_HPA (1014.0)
#define DHTTYPE DHT22

int MQ9PIN;
int MQ4PIN;
int MQ5PIN;
int MQ6PIN;
int NEWTEMPPIN;
int MOTIONPIN;
int DBPIN;

Adafruit_BMP3XX bmp;

DHT* dht = nullptr;

const int numReadings = 5;

const int Slot1A1 = A13;
const int Slot1A2 = A14;
const int Slot1D = 4;

const int Slot2A1 = A8;
const int Slot2A2 = A6;
const int Slot2D = 6;

const int Slot3A1 = 20;
const int Slot3A2 = 21;
const int Slot3D = 9;

int R1pin = A15;
int R2pin = A7;
int R3pin = A0;

int DHTlower = 44 - 5;
int DHTupper = 44 + 5;

int MQ9lower = 180 - 5;
int MQ9upper = 180 + 5;

int BMPlower = 328 - 8;
int BMPupper = 328 + 8;

int MQ4lower = 229 - 5;
int MQ4upper = 229 + 5;

int MQ5lower = 937 - 5;  //MQ5 == MQ7
int MQ5upper = 937 + 5;

int MQ6lower = 973 - 5;
int MQ6upper = 973 + 5;

int NewTemplower = 2 - 5;
int NewTempupper = 2 + 5;

int Motionlower = 805 - 15;
int Motionupper = 805 + 10;

int DBlower = 826 - 10;
int DBupper = 826 + 10;

int readingsR1[numReadings];
int readingsR2[numReadings];
int readingsR3[numReadings];

int readIndex = 0;
int gas = 0;

SensorType currentSensorTypeInSlot1 = SENSOR_NONE;
SensorType currentSensorTypeInSlot2 = SENSOR_NONE;
SensorType currentSensorTypeInSlot3 = SENSOR_NONE;

float hum;  //Stores humidity value
float temp; //Stores temperature value

void setup() {
  Serial.begin(9600);
  // Initialize all the readings to 0
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsR1[thisReading] = 0;
    readingsR2[thisReading] = 0;
    readingsR3[thisReading] = 0;
  }
}

void loop() {
  static unsigned long lastUpdateTime = 0;

  // Update readings
  updateReadings(R1pin, readingsR1);
  updateReadings(R2pin, readingsR2);
  updateReadings(R3pin, readingsR3);

  // Check and update sensor connections
  currentSensorTypeInSlot1 = checkAndUpdateSensor(1, readingsR1);
  currentSensorTypeInSlot2 = checkAndUpdateSensor(2, readingsR2);
  currentSensorTypeInSlot3 = checkAndUpdateSensor(3, readingsR3);

  // Print sensor data every second
  if (millis() - lastUpdateTime >= 1000) {
    lastUpdateTime = millis();
    printSensorData();
  }

  delay(100); // Small delay for stability
}

void updateReadings(int pin, int readings[]) {
  // Shift readings and add the new reading at the end
  for (int i = 0; i < numReadings - 1; i++) {
    readings[i] = readings[i + 1];
  }
  readings[numReadings - 1] = analogRead(pin);
}

SensorType checkAndUpdateSensor(int slot, int readings[]) {
  SensorType sensorType = detectSensorType(readings);
  if (sensorType != SENSOR_NONE) {
    configurePins(slot, sensorType);
  } else {
    disconnectSensor(slot);
  }
  return sensorType;
}

SensorType detectSensorType(int readings[]) {
  if (isInRange(readings, DHTlower, DHTupper)) {
    return SENSOR_DHT;
  } else if (isInRange(readings, MQ9lower, MQ9upper)) {
    return SENSOR_MQ9;
  } else if (isInRange(readings, MQ4lower, MQ4upper)) {
    return SENSOR_MQ4;
  } else if (isInRange(readings, MQ5lower, MQ5upper)) {
    return SENSOR_MQ5;
  } else if (isInRange(readings, MQ6lower, MQ6upper)) {
    return SENSOR_MQ6;
  } else if (isInRange(readings, NewTemplower, NewTempupper)) {
    return SENSOR_NEW_TEMP;
  } else if (isInRange(readings, BMPlower, BMPupper)) {
    return SENSOR_BMP;
  } else if (isInRange(readings, Motionlower, Motionupper)) {
    return SENSOR_MOTION;
  } else if (isInRange(readings, DBlower, DBupper)) {
    return SENSOR_DB;
  } else {
    return SENSOR_NONE;
  }
}


bool isInRange(int readings[], int lower, int upper) {
  for (int i = 0; i < numReadings; i++) {
    if (readings[i] < lower || readings[i] > upper) {
      return false;
    }
  }
  return true;
}

void configurePins(int slot, SensorType sensorType) {
  switch (slot) {
    case 1:
      configurePinsForSlot(Slot1A1, Slot1A2, Slot1D, sensorType);
      break;
    case 2:
      configurePinsForSlot(Slot2A1, Slot2A2, Slot2D, sensorType);
      break;
    case 3:
      configurePinsForSlot(Slot3A1, Slot3A2, Slot3D, sensorType);
      break;
  }
}

void configurePinsForSlot(int analogPin1, int analogPin2, int digitalPin, SensorType sensorType) {
  switch (sensorType) {
    case SENSOR_DHT:
      // Delete existing DHT object if it exists
      if (dht != nullptr) {
        delete dht;
        dht = nullptr;  // Set to nullptr to avoid dangling pointer
      }
      // Allocate new DHT object
      dht = new DHT(digitalPin, DHTTYPE);
      dht->begin();
      break;
    case SENSOR_MQ9:
      // Assuming MQ9 uses an analog pin for readings
      MQ9PIN = analogPin1;
      break;
    case SENSOR_MQ4:
      MQ4PIN = analogPin1;
      break;
    case SENSOR_MQ5:
      MQ5PIN = analogPin1;
      break;
    case SENSOR_MQ6:
      MQ6PIN = analogPin1;
      break;
    case SENSOR_NEW_TEMP:
      NEWTEMPPIN = analogPin1;
      break;
    case SENSOR_MOTION:
      MOTIONPIN = digitalPin;
      pinMode(MOTIONPIN, INPUT);
      break;
    case SENSOR_DB:
      DBPIN = analogPin1;
      break;
    case SENSOR_BMP:
      if (!bmp.begin_I2C()) {
        Serial.println("BMP sensor not found!");
      } else {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
      }
      break;
  }
}

void disconnectSensor(int slot) {
  switch (slot) {
    case 1:
      delete dht; dht = nullptr; break;
    case 2:
      // Add logic if any special handling needed for MQ9, MQ4, MQ5, MQ6 disconnection
      break;
    case 3:
      // Add logic if any special handling needed for BMP disconnection
      break;
  }
}

void printSensorData() {
  printSlotData(1, currentSensorTypeInSlot1, readingsR1[numReadings - 1]);
  printSlotData(2, currentSensorTypeInSlot2, readingsR2[numReadings - 1]);
  printSlotData(3, currentSensorTypeInSlot3, readingsR3[numReadings - 1]);
  Serial.println();
}

void printSlotData(int slot, SensorType sensorType, int currentIdentifierReading) {
  Serial.print("Slot");
  Serial.print(slot);
  Serial.print(" ");

  Serial.print("Id: ");
  Serial.print(currentIdentifierReading);
  Serial.print(", ");

  switch (sensorType) {
    case SENSOR_DHT:
      if (dht != nullptr) {
        Serial.print("DHT, Humidity: ");
        Serial.print(dht->readHumidity());
        Serial.print(", Temp: ");
        Serial.print(dht->readTemperature());
      }
      break;
    case SENSOR_MQ9:
      Serial.print("MQ9, Gas Value: ");
      Serial.print(analogRead(MQ9PIN));
      break;
    case SENSOR_MQ4:
      Serial.print("MQ4, Gas Value: ");
      Serial.print(analogRead(MQ4PIN));
      break;
    case SENSOR_MQ5:
      Serial.print("MQ5, Gas Value: ");
      Serial.print(analogRead(MQ5PIN));
      break;
    case SENSOR_MQ6:
      Serial.print("MQ6, Gas Value: ");
      Serial.print(analogRead(MQ6PIN));
      break;
    case SENSOR_NEW_TEMP:
      Serial.print("New Temp, Value: ");
      
      Serial.print(analogRead(NEWTEMPPIN)); // Replace with appropriate function if different
      break;
    case SENSOR_BMP:
      if (bmp.performReading()) {
        Serial.print("BMP, Pressure: ");
        Serial.print(bmp.pressure / 100.0);
        Serial.print(", Temp: ");
        Serial.print(bmp.temperature);
      } else {
        Serial.print("BMP, Failed to read");
      }
      break;
    case SENSOR_MOTION:
      Serial.print("Motion, Value: ");
      Serial.print(digitalRead(MOTIONPIN));
      break;
    case SENSOR_DB:
      Serial.print("DB, Value: ");
      Serial.print(analogRead(DBPIN));
      break;
    default:
      Serial.print("None");
  }
  Serial.print("; ");
}
