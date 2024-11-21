#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PulseSensorPlayground.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoBLE.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 20, 4);  // I2C address 0x27, 20x4 LCD

// Pulse sensor setup
const int PulseWire = A0;  // Pulse sensor connected to analog pin A0
int myBPM, Threshold = 550;
PulseSensorPlayground pulseSensor;

// Temperature sensor setup
#define ONE_WIRE_BUS 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

// BLE setup
BLEService healthService("181A");  // Environmental Sensing Service UUID
BLECharacteristic  heartRateCharacteristic("2A37", BLERead | BLENotify, 2);  // Heart Rate Characteristic
// Correct the constructor to use the appropriate parameters
BLECharacteristic temperatureCharacteristic("2A6E", BLERead | BLENotify, sizeof(float));  // Temperature Characteristic

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  while (!Serial);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Display startup message
  lcd.setCursor(0, 0);
  lcd.print("  Health Monitor");
  delay(2000);
  lcd.clear();

  // Initialize Pulse Sensor
  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);
  pulseSensor.begin();

  // Initialize Temperature Sensor
  sensors.begin();
  if (!sensors.getAddress(tempDeviceAddress, 0)) {
    Serial.println("No temperature sensor found!");
    lcd.print("Temp Sensor Error");
    while (1);  // Halt if the sensor is not found
  }

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    lcd.print("BLE Init Failed!");
    while (1);  // Halt if BLE initialization fails
  }

  // Configure BLE device
  BLE.setLocalName("HealthMonitor");
  BLE.setDeviceName("HealthMonitor");
  BLE.setAdvertisedService(healthService);
  healthService.addCharacteristic(heartRateCharacteristic);
  healthService.addCharacteristic(temperatureCharacteristic);
  BLE.addService(healthService);

  // Start advertising
  BLE.advertise();
  Serial.println("BLE device is now advertising...");
  lcd.setCursor(0, 0);
  lcd.print("BLE Advertising...");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    lcd.setCursor(0, 1);
    lcd.print("Connected!");

    while (central.connected()) {
      // Read heart rate
      myBPM = pulseSensor.getBeatsPerMinute();
      if (pulseSensor.sawStartOfBeat() && myBPM > 0) {
        uint8_t data[3];  // Total of 3 bytes: 1 byte for flags + 2 bytes for BPM
        data[0] = 0x00;  // Flags byte, 0x00 means 16-bit heart rate value
        data[1] = (myBPM & 0xFF);  // Lower byte of BPM
        data[2] = (myBPM >> 8);    // Upper byte of BPM

        // Send the heart rate data via BLE
        heartRateCharacteristic.writeValue(data, sizeof(data));  // Send the byte array and its size

        lcd.setCursor(0, 2);
        lcd.print("Heart: ");
        lcd.print(myBPM);
        lcd.print(" BPM  ");
        Serial.print("Heart Rate: ");
        Serial.println(myBPM);
      } else {
        lcd.setCursor(0, 2);
        lcd.print("Heart: Waiting  ");
        Serial.println("Heart Rate not valid");
      }

      // Read temperature
      sensors.requestTemperatures();
      float tempC = sensors.getTempC(tempDeviceAddress);

      // Check if temperature is valid
      if (tempC != DEVICE_DISCONNECTED_C) {
        // // Convert temperature to byte array
        // byte tempBytes[4];
        // memcpy(tempBytes, &tempC, sizeof(tempC));  // Copy the float into byte array

        // // Send temperature as byte array
        // temperatureCharacteristic.writeValue(tempBytes, sizeof(tempBytes));
        int tempCInt = (int)(tempC * 100);  // Temperature in tenths of a degree

        // Convert the integer to a byte array (2 bytes for int16_t)
        byte tempBytes[2];
        tempBytes[0] = (tempCInt & 0xFF);        // Lower byte
        tempBytes[1] = (tempCInt >> 8) & 0xFF;   // Upper byte

        // Send the temperature as byte array
        temperatureCharacteristic.writeValue(tempBytes, sizeof(tempBytes));
       
        // Update LCD with the new temperature value
        lcd.setCursor(0, 3);  // Ensure the cursor is at the start of the 4th row
        lcd.print("Temp: ");  
        lcd.print(tempC, 1);  // Print temperature with one decimal place
        lcd.print(" C    ");  // Add extra spaces to clear any old text
        Serial.print("Temperature: ");
        Serial.println(tempC);
      } else {
        Serial.println("Error: Could not read temperature.");
        lcd.setCursor(0, 3);
        lcd.print("Temp Error  ");
      }

      delay(1000);  // Small delay to prevent overloading the LCD
    }

    Serial.println("Disconnected from central");
    lcd.setCursor(0, 1);
    lcd.print("Disconnected    ");
  }
}
