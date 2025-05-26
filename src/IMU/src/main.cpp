// Includes
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

// Set to 1 for serial debugging
bool DEBUGGING = 0;

// Create bluetooth service:
BLEService IMUService("082b91ae-e83c-11e8-9f32-f2801f1b9fd1");

// Create bluetooth characteristics to write readings to
BLEIntCharacteristic accelX("082b9438-e83c-11e8-9f32-f2801f1b9fd1", BLEWrite);
BLEIntCharacteristic accelY("082b9622-e83c-11e8-9f32-f2801f1b9fd1", BLEWrite);
BLEIntCharacteristic accelZ("082b976c-e83c-11e8-9f32-f2801f1b9fd1", BLEWrite);
BLEIntCharacteristic gyroX("082b9439-e83c-11e8-9f32-f2801f1b9fd1", BLEWrite);
BLEIntCharacteristic gyroY("082b9623-e83c-11e8-9f32-f2801f1b9fd1", BLEWrite);
BLEIntCharacteristic gyroZ("082b976d-e83c-11e8-9f32-f2801f1b9fd1", BLEWrite);

// IMU floats
float accX, accY, accZ; 
float gyrX, gyrY, gyrZ;

void setup() {

    if (DEBUGGING) {
        Serial.begin(9600);
    }
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize IMU
    if (!IMU.begin()) {
        if (DEBUGGING) {
            Serial.println("Failed to initialize IMU!");
        }
        while (1);
    }

    // Initialize bluetooth
    while (!BLE.begin()) {
        if (DEBUGGING) {
            Serial.println("Waiting for BLE to start");
        }
        delay(1);
    }

    // Set the name of the bluetooth service
    BLE.setLocalName("BLE_IMU");

    // Set UUID
    BLE.setAdvertisedService(IMUService);

    // Create characteristics to be transmitted
    IMUService.addCharacteristic(accelX);
    IMUService.addCharacteristic(accelY);
    IMUService.addCharacteristic(accelZ);
    IMUService.addCharacteristic(gyroX);
    IMUService.addCharacteristic(gyroY);
    IMUService.addCharacteristic(gyroZ);

    // Add service
    BLE.addService(IMUService);

    // Start advertising bluetooth service
    BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();

  // If connected to bluetooth
  if (central) {

    // Turn on LED to show a bluetooth connection has been made
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {

        // Get accelerometer values
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(accX, accY, accZ);
            if (DEBUGGING) {
                Serial.print("Accel x: ");
                Serial.println(accX);
                Serial.print("Accel y: ");
                Serial.println(accY);
                Serial.print("Accel z: ");
                Serial.println(accZ);
            }
        }

        // Get gyroscope values
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(gyrX, gyrY, gyrZ);
            if (DEBUGGING) {
                Serial.print("Gyro x: ");
                Serial.println(gyrX);
                Serial.print("Gyro y: ");
                Serial.println(gyrY);
                Serial.print("Gyro z: ");
                Serial.println(gyrZ);
            }
        }

        // Write readings to bluetooth
        accelX.writeValue(accX);
        accelY.writeValue(accY);
        accelZ.writeValue(accZ);
        gyroX.writeValue(gyrX);
        gyroY.writeValue(gyrY);
        gyroZ.writeValue(gyrZ);
    }
  } else {
    // Turn off LED
    digitalWrite(LED_BUILTIN, LOW);
  }
}