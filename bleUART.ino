/*
  Arduino BLE UART

  This code is

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU
#include <Arduino_LPS22HB.h> //Include library to read Pressure
#include <Arduino_HTS221.h> //Include library to read Temperature and Humidity
#include <Arduino_APDS9960.h> //Include library for colour, proximity and gesture recognition

//variables for storing sensor values
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
float Pressure;
float Temperature, Humidity;
int Proximity;

//sprintf formatting strings for each sensor
const char ACCEL_FORMAT[] = "A:%.2f,%.2f,%.2f";
const char GYRO_FORMAT[] = "G:%.2f,%.2f,%.2f";
const char MAG_FORMAT[] = "M:%.2f,%.2f,%.2f";
const char PRES_FORMAT[] = "P:%.2f";
const char TEMP_FORMAT[] = "T:%.2f";
const char HUM_FORMAT[] = "H:%.2f";
const char PROX_FORMAT[] = "X:%d";

const int LED_PIN = LED_BUILTIN; // set ledPin to on-board LED
const int MAX_CHAR_VAL_LEN = 256; //governs the maximum length of sent/recieved ble messages
const unsigned long SENSOR_READ_PERIOD = 1000; //sets the time between sensor readings and ble updates in milliseconds
volatile bool newRXValue = false; //flag indicating whether a new message has arrived over ble
unsigned long sensorUpdatedTime = 0; //millisecond timestamp of last sensor reading

BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); // create service

//create RX characteristic and descriptor
BLECharacteristic rxCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, MAX_CHAR_VAL_LEN);
BLEDescriptor rxDescriptor("2901", "RX - Receive Data (Write)");

//create tx characteristic and descriptor
BLECharacteristic txCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, MAX_CHAR_VAL_LEN);
BLEDescriptor txDescriptor("2901", "TX - Transfer Data (Notify)");

void setup() {
  Serial.begin(9600);
  //the line below makes it so that the program waits until you open a serial terminal
  //this is only useful for debugging so I commented it out
  //while (!Serial);

  if (!IMU.begin()) //Initialize IMU sensor
  { Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (!BARO.begin()) //Initialize Pressure sensor
  {
    Serial.println("Failed to initialize Pressure Sensor!");
    while (1);
  }
  if (!HTS.begin()) //Initialize Temperature and Humidity sensor
  {
    Serial.println("Failed to initialize Temperature and Humidity Sensor!");
    while (1);
  }
  if (!APDS.begin()) //Initialize Colour, Proximity and Gesture sensor
  {
    Serial.println("Failed to initialize Colour, Proximity and Gesture Sensor!");
    while (1);
  }

  pinMode(LED_PIN, OUTPUT); // use the LED as an output

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("bleUART");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(uartService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  txCharacteristic.addDescriptor(txDescriptor);
  rxCharacteristic.addDescriptor(rxDescriptor);

  //assign event handler for whenever the RX is written to
  rxCharacteristic.setEventHandler(BLEWritten, rxCharWrittenHandler);

  // add the characteristics to the service
  uartService.addCharacteristic(rxCharacteristic);
  uartService.addCharacteristic(txCharacteristic);

  // add the service
  BLE.addService(uartService);

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // poll for BLE events
  BLE.poll();

  //if there are new values to be read from the BLE RX Characteristic
  if (newRXValue) {
    readNewRXValue();
  }

  //if it is time to make the next sensor reading
  if (BLE.connected() && millis() - sensorUpdatedTime > SENSOR_READ_PERIOD) {
    sensorUpdatedTime = millis();
    updateSensorData();
  }
}

/*
   This function gets triggered whenever an external device connects to the Arduino BLE
*/
void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  digitalWrite(LED_PIN, HIGH);
}

/*
   This function gets triggered whenever an external device disconnects from the Arduino BLE
*/
void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  digitalWrite(LED_PIN, LOW);
}

/*
   This function gets triggered whenever an external device writes values to the RX characteristic
*/
void rxCharWrittenHandler(BLEDevice central, BLECharacteristic characteristic) {
  //set flag indicating that new value is available to be processed in the main loop
  newRXValue = true;
}

/*
   This function reads in the values from the RX characteristic and prints them out to the Serial
   monitor. It then calls the processCommand function to further deconstruct the RX data.
*/
void readNewRXValue() {
  int bufferLength = rxCharacteristic.valueLength();
  char rxBuffer[bufferLength];

  rxCharacteristic.readValue(rxBuffer, bufferLength);

  Serial.print("Received: ");
  Serial.println(rxBuffer);
  newRXValue = false;

  processCommand(rxBuffer, bufferLength);
}

/*
   This function expects a command in the format of A:N, where A is a case-agnostic letter that specifies
   the type of command, : is the symbol that specifies the beginning of the value of the command,
   and N is a decimal number that specifies the magnitude of the command.

   IE.
   S:-45 could mean (S)teering 45 degrees left
   T:100 could mean (T)hrottle 100 percent

   or something along those lines, this is for you to decide
*/
void processCommand(char cmd[], int cmdLen) {
  //use a switch statement to process RX commands
  if (cmdLen > 2 && cmd[1] == ':') {
    char switchVal = cmd[0];
    switch (switchVal) {
      case 'S':
      case 's':
        //do things with steering servo
        Serial.print("Steering Command: ");
        Serial.println(cmdToIntValue(cmd, cmdLen));
        break;

      case 'T':
      case 't':
        //do things with throttle
        Serial.print("Throttle Command: ");
        Serial.println(cmdToIntValue(cmd, cmdLen));
        break;

      default:
        //what to do if command is not recognized
        Serial.print(cmd);
        Serial.println(" is not a valid command.");
        break;
    }
  }
  else {
    Serial.print(cmd);
    Serial.println(" is not a valid command.");
    txCharacteristic.writeValue(cmd);
  }
}

/*
   This function expects a properly formatted command and returns the integer value of the magnitude
   of the command.
*/
int cmdToIntValue(char cmd[], int cmdLen) {
  //create a buffer array to store only the number characters, skipping the first two chars EX "A:"
  int buffLen = (cmdLen - 2);
  char buff[buffLen];

  //transfer chars [2 to len-1] from the cmd to the buffer
  for (int x = 2; x < cmdLen; x++) {
    buff[x - 2] = cmd[x];
  }

  //convert char array to an integer
  return atoi(buff);
}

/*
   This function polls all available sensors and updates the values stored in global variables.
   Afterwards, it sends the values to a formatSensorValue() function to be sent over BLE
*/
void updateSensorData() {
  char outputStr[50];
  //Accelerometer values
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
    sprintf(outputStr, ACCEL_FORMAT, accel_x, accel_y, accel_z);
    txCharacteristic.writeValue(outputStr);
  }

  //Gyroscope values
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    sprintf(outputStr, GYRO_FORMAT, gyro_x, gyro_y, gyro_z);
    txCharacteristic.writeValue(outputStr);
  }

  //Magnetometer values
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mag_x, mag_y, mag_z);
    sprintf(outputStr, MAG_FORMAT, mag_x, mag_y, mag_z);
    txCharacteristic.writeValue(outputStr);
  }

  //Read Pressure value
  Pressure = BARO.readPressure();
  sprintf(outputStr, PRES_FORMAT, Pressure);
  txCharacteristic.writeValue(outputStr);

  //Read Temperature value
  Temperature = HTS.readTemperature();
  sprintf(outputStr, TEMP_FORMAT, Temperature);
  txCharacteristic.writeValue(outputStr);


  //Read Humidity value
  Humidity = HTS.readHumidity();
  sprintf(outputStr, HUM_FORMAT, Humidity);
  txCharacteristic.writeValue(outputStr);

  //Proximity value
  if (APDS.proximityAvailable()) {
    Proximity = APDS.readProximity();
    sprintf(outputStr, PROX_FORMAT, Proximity);
    txCharacteristic.writeValue(outputStr);
  }

  txCharacteristic.writeValue("--------------------");
}
