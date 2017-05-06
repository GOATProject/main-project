// Haptic Arm Robotic Control Glove
// Developer: James McGinness
// Team: Thomas Sleigh, Ian Weber, Tayeh Tang
// Class: CPP Senior Project 2016-2017
// Project Name: Haptic Arm TeleRobotic System
/*
 *
 *
 *
 *
 *
 *
 */

// Basic Include list
#include <Wire.h>
#include <stdio.h>

// ESP 32 Includes
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs.h"

// BNO055 IMU Includes
// #include "IMU_functions.cpp"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Declarations
#define BNO055_SAMPLERATE_DELAY_MS (10) // Set the delay between fresh samples
#define STORAGE_NAMESPACE "storage" // Set the Non-Volatile Storage Key
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensor_t sensor;
sensors_event_t event;
adafruit_bno055_offsets_t activeCalibration;
esp_err_t err;
nvs_handle my_handle;

void setup() {
    /*
     * Device Setup
     */
    Serial.begin(115200);
    Serial.println(F("setup()"));

    pinMode(4, INPUT);
    pinMode(15, INPUT);

    // Initial Non Volatile Storage, NVS as an EEPROM Replacement
    err = nvs_flash_init();
    if (err != ESP_OK) {
      Serial.print("Fail init nvs");
      Serial.println(err, HEX);
    } else {
        Serial.println(F("Flash Init"));
    }


    // Setup inputs and reads for the IMU
    //   Configure Interrupt pin and the function to read the data in to the main control loop that will transmit the bluetooth data
    // pinMode(interruptPin, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(interruptPin), readIMU, RISING);

    Serial.println(F("Init IMU"));
    // Connect to IMU, Load stored calibration data, **disable and ignore compass.**
    delay(1);
    if (!bno.begin(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS))
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print(F("Fail"));
        while (1);
    } else {
        // BNO055 did Successfully begin
        Serial.println(F("Began"));

        // Set BNO055 to only be an IMU, not an NDOF, since we dont want the compass to affect our measurements
        // bno.setMode(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);

        //      Check if Device is fully calibrated
        //      if it is not calibrated check if there is a stored calibration, if there is load that to the device
        //      If there isn't a saved calibration, calibrate the device and save that data to Non-Volatile storage.
        if (bno.isFullyCalibrated() == false)
        {
            // check for saved calibration
                // Open Non-Volatile Storage for reading
                Serial.println(F("Checking for previous sensor ID"));
                err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
                if (err != ESP_OK) {
                    Serial.print("Error Occurred opening NVS for reading");
                    Serial.println(err, HEX);
                    while(1){};
                } else {
                    Serial.println(F("Successfully opened NVS for reading and writing"));
                }

                uint8_t offsets;
                err = nvs_get_u8(my_handle, "sensor_offsets", &offsets);

                // Check if the offsets were found, or if an error occurred
                if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                    // Log and Hang due to error.
                    Serial.print(F("Error Occurred reading stored sensor ID. Error:"));
                    Serial.println(err, HEX);
                    while(1){};
                } else if (err == ESP_ERR_NVS_NOT_FOUND) {
                    // The offsets weren't saved, the device also isn't calibrated. based on parent if()
                    Serial.println("Offsets weren't saved in Non-Volatile Storage");

                    while (bno.isFullyCalibrated() == false)
                    {
                        Serial.println(F("Device not calibrated, waiting for calibration"));
                        bno.getEvent(&event);

                        /* Print event values and details */
                        printEventDetails(event);

                        /* Optional: Display calibration status */
                        displayCalStatus();

                        /* New line for the next sample */
                        Serial.println("");

                        /* Wait the specified delay before requesting new data */
                        Serial.println("----------------------------------\n");
                        delay(BNO055_SAMPLERATE_DELAY_MS);
                        if (digitalRead(0)) {
                            Serial.println(F("Button Pressed, saving existing calibration"));
                            delay(1000);
                            break;
                        }
                    }

                    Serial.println(F("\nFully calibrated!"));


                    bno.getSensorOffsets(&offsets);
                    Serial.println(F("\n\nStoring calibration data to NVS..."));
                    err = nvs_set_u8(my_handle, "sensor_offsets", offsets);
                    if (err != ESP_OK) {
                        Serial.println(F("Error saving sensor offsets"));
                        while(1){};
                    }

                } else if (err != ESP_OK) { // Unusual Error, catch here
                    Serial.print(F("Unknown error occurred attempting to read stored offsets"));
                    Serial.println(err, HEX);
                    while(1){};
                } else if (err == ESP_OK) { // Was able to read stored offsets. upload them to the device
                    // Found the offsets in Non-Volatile Storage set the device up.
                    Serial.println("Found Saved Offsets");
                    delay(1);
                    bno.setSensorOffsets(&offsets);
                } else {
                    Serial.print("I have no idea how you got here. Good luck.");
                    Serial.println(err, HEX);
                    while(1){};
                }// end if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)



        } // end if (bno.isFullyCalibrated() == false)
    } // end if BNO055 did begin
    nvs_commit(my_handle);
    nvs_close(my_handle);

    // Set BNO055 to use external Clock source
    bno.setExtCrystalUse(true);
    Serial.println("Ending setup()");
}

void loop() {
    // If dataHasChanged is true, Calculate position from IMU data
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    /* Display the floating point data */
    printEventDetails(event);
    Serial.print("\t\t");
    int thumb = analogRead(4);
    int finger = analogRead(15);
    Serial.print(thumb);
    Serial.print(",");
    Serial.print(finger);
    Serial.println("");
    delay(BNO055_SAMPLERATE_DELAY_MS);

    // Transmit data to base station over bluetooth
}



/*************************************************
 *  ___ _   _ _  _  ___ _____ ___ ___  _  _ ___  *
 * | __| | | | \| |/ __|_   _|_ _/ _ \| \| / __| *
 * | _|| |_| | .` | (__  | |  | | (_) | .` \__ \ *
 * |_|  \___/|_|\_|\___| |_| |___\___/|_|\_|___/ *
 *************************************************/

/*
 * Reads IMU Data to global data that can be used in the main loop
 */
 void readIMU(void) {
     // Read 6 Axes of IMU to global variable
     // Set dataHasChanged variable to save recalculation time
 }

/*
 * Displays some basic information on this sensor from the unified
 * sensor API sensor_t type (see Adafruit_Sensor for more information)
 */
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.print(F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print(F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print(F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" xxx"));
    Serial.print(F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" xxx"));
    Serial.print(F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" xxx"));
    Serial.println(F("------------------------------------"));
//    Serial.println("");
    delay(500);
}

/*
 * Display some basic info about the sensor status
*/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print(F("System Status: 0x"));
    Serial.println(system_status, HEX);
    Serial.print(F("Self Test:     0x"));
    Serial.println(self_test_results, HEX);
    Serial.print(F("System Error:  0x"));
    Serial.println(system_error, HEX);
    Serial.println(F(""));
    delay(500);
}

/*
 * Display sensor calibration status
 */
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print(F("\t"));
    if (!system)
    {
        Serial.print(F("! "));
    }

    /* Display the individual values */
    Serial.print(F("Sys:"));
    Serial.print(system, DEC);
    Serial.print(F(" G:"));
    Serial.print(gyro, DEC);
    Serial.print(F(" A:"));
    Serial.print(accel, DEC);
    Serial.print(F(" M:"));
    Serial.print(mag, DEC);
}


/*
 * Display the raw calibration offset and radius data
 */
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print(F("Accelerometer: "));
    Serial.print(calibData.accel_offset_x); Serial.print(F(" "));
    Serial.print(calibData.accel_offset_y); Serial.print(F(" "));
    Serial.print(calibData.accel_offset_z); Serial.print(F(" "));

    Serial.print(F("\nGyro: "));
    Serial.print(calibData.gyro_offset_x); Serial.print(F(" "));
    Serial.print(calibData.gyro_offset_y); Serial.print(F(" "));
    Serial.print(calibData.gyro_offset_z); Serial.print(F(" "));

    Serial.print(F("\nMag: "));
    Serial.print(calibData.mag_offset_x); Serial.print(F(" "));
    Serial.print(calibData.mag_offset_y); Serial.print(F(" "));
    Serial.print(calibData.mag_offset_z); Serial.print(F(" "));

    Serial.print(F("\nAccel Radius: "));
    Serial.print(calibData.accel_radius);

    Serial.print(F("\nMag Radius: "));
    Serial.print(calibData.mag_radius);
}

/*
 * Print out event data
 */
void printEventDetails(sensors_event_t event) {
    Serial.print(F(event.orientation.x), 4);
    Serial.print(F(","));
    Serial.print(F(event.orientation.y), 4);
    Serial.print(F(","));
    Serial.print(F(event.orientation.z), 4);
    Serial.print(F(","));
    Serial.print(F(event.orientation.roll));
    Serial.print(F(","));
    Serial.print(F(event.orientation.pitch));
    Serial.print(F(","));
    Serial.print(F(event.orientation.heading));
}
