/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
    Please note the long strings of data sent mean the *RTS* pin is
    required with UART to slow down data sent to the Bluefruit LE!
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

//Orientation sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */
int num, num2;
int32_t sensor1ServiceId;
int32_t sensor1MeasureCharId;
int32_t hrmLocationCharId;

int32_t OrientationServiceID;
int32_t orientationID;

int32_t AccelServiceID;
int32_t accelID;


int32_t LinAccelServiceID;
int32_t linaccelID;

//Orientation sensor
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();//(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
//  while (!Serial); // required for Flora & Micro
//  delay(500);

  boolean success;

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Heart Rate Monitor (HRM) Example"));
  Serial.println(F("---------------------------------------------------"));

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit HRM': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=POTIOC 1")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Pression of Sensor 1 Service definition (UUID = 0x280D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x280D"), &sensor1ServiceId);
  if (! success) {
    error(F("Could not add Sensor 1 service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=1, MAX_LEN=20, VALUE=4"), &sensor1MeasureCharId);
    //success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &sensor1MeasureCharId);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  /* Add Orientation service */


    Serial.println(F("Adding the 2nd Service definition (UUID = 0x280E): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x280E"), &OrientationServiceID);
  if (! success) {
    error(F("Could not add Sensor 1 service"));
  }

  Serial.println(F("Adding the axis x of Orientation characteristic (UUID = 0x2E37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2E37, PROPERTIES=0x10, MIN_LEN=1, MAX_LEN=20, VALUE=5"), &orientationID);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }


  
  /* Add Acceleration service */


    Serial.println(F("Adding the 4th Service definition (UUID = 0x280F): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x280F"), &AccelServiceID);
  if (! success) {
    error(F("Could not add Sensor 1 service"));
  }

  Serial.println(F("Adding the axis x of Lineal Acceleration characteristic (UUID = 0x2F37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2F37, PROPERTIES=0x10, MIN_LEN=1, MAX_LEN=20, VALUE=5"), &accelID);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }

    /* Add Acceleration service */


    Serial.println(F("Adding the 3th Service definition (UUID = 0x280A): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x280A"), &LinAccelServiceID);
  if (! success) {
    error(F("Could not add Sensor 1 service"));
  }

  Serial.println(F("Adding the axis x of Acceleration characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=1, MAX_LEN=20, VALUE=5"), &linaccelID);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }




ble.sendCommandCheckOK( F("AT+GATTLIST"));
  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();


   if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(500);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();


    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }


    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

   //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
//    if (foundCalib){
//        Serial.println("Move sensor slightly to calibrate magnetometers");
//        while (!bno.isFullyCalibrated())
//        {
//            bno.getEvent(&event);
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }
//    else
//    {
//        Serial.println("Please Calibrate Sensor: ");
//        while (!bno.isFullyCalibrated())
//        {
//            bno.getEvent(&event);
//
//            Serial.print("X: ");
//            Serial.print(event.orientation.x, 4);
//            Serial.print("\tY: ");
//            Serial.print(event.orientation.y, 4);
//            Serial.print("\tZ: ");
//            Serial.print(event.orientation.z, 4);
//
//            /* Optional: Display calibration status */
//            displayCalStatus();
//
//            /* New line for the next sample */
//            Serial.println("");
//
//            /* Wait the specified delay before requesting new data */
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }


pinMode(13, OUTPUT);
digitalWrite(13, HIGH);

  
}

/** Send randomized heart rate data continuously **/
void loop(void)
{
    /* Get a new orientation sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  
  int sensor = analogRead(A1);//map(analogRead(A1),0,800,16,255);//num;//random(50, 100);

  Serial.print(F("Updating HRM value to "));
  Serial.print(sensor);
  Serial.println(F(" BPM"));

  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( sensor1MeasureCharId );
  ble.print( F(",") );
  ble.print(sensor);
    ble.print( F("/") );
  ble.println(sensor);

  

//Orientation sensor
  Serial.print(F("Updating Orientation values x:"));
  Serial.print(event.orientation.x);
  Serial.print(F(" y:"));
  Serial.print(event.orientation.y);
  Serial.print(F(" z:"));
  Serial.println(event.orientation.z);
  
  ble.print( F("AT+GATTCHAR=") );
  ble.print( orientationID );
  ble.print( F(",") );
  ble.print(event.orientation.x);
  ble.print( F("/") ); 
  ble.print(event.orientation.y);
  ble.print( F("/") ); 
  ble.println(event.orientation.z);


  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  Serial.print(F("Updating Acceleration values x:"));
  Serial.print(accel.x());
  Serial.print(F(" y:"));
  Serial.print(accel.y());
  Serial.print(F(" z:"));
  Serial.println(accel.z());

  ble.print( F("AT+GATTCHAR=") );
  ble.print( accelID );
  ble.print( F(",") );
  ble.print(accel.x());
  ble.print( F("/") ); 
  ble.print(accel.y());
  ble.print( F("/") ); 
  ble.println(accel.z());

    
  Serial.print(F("Updating Linneal Acceleration values x:"));
  Serial.print(linaccel.x());
  Serial.print(F(" y:"));
  Serial.print(linaccel.y());
  Serial.print(F(" z:"));
  Serial.println(linaccel.z());

  ble.print( F("AT+GATTCHAR=") );
  ble.print( linaccelID ); //Cambiar
  ble.print( F(",") );
  ble.print(linaccel.x());
  ble.print( F("/") ); 
  ble.print(linaccel.y());
  ble.print( F("/") ); 
  ble.println(linaccel.z());

//  Serial.print(F("Updating Orientation values x:"));
//  Serial.print(event.orientation.x);
//  Serial.print(F(" y:"));
//  Serial.print(event.orientation.y);
//  Serial.print(F(" z:"));
//  Serial.println(event.orientation.z);
//  
//  ble.print( F("AT+GATTCHAR=") );
//  ble.print( orientationIDx );
//  ble.print( F(",") );
//  ble.println(event.orientation.x);
//
//  ble.print( F("AT+GATTCHAR=") );
//  ble.print( orientationIDy );
//  ble.print( F(",") );
//  ble.println(event.orientation.y);
//
//  ble.print( F("AT+GATTCHAR=") );
//  ble.print( orientationIDz );
//  ble.print( F(",") );
//  ble.println(event.orientation.z);

  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("Failed to get response!"));
  }

  /* Delay before next measurement update */
  delay(100);

  

  num++;
  num2 = num2 + 2;

  if(num==256) num = 16;
  if(num2>=256) num2 = 16;
}
