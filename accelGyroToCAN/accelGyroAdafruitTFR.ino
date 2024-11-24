/*
 * 11-24-24
 * Adafruit feather m4 CAN 
 * for setting up 6 dof accel/gyro to transmit over CAN.
 * got offsets through this procedure/code: https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration/simple-gyro-calibration
 * ari reischer
 */
 
#include <Wire.h>
#include <CANSAME5x.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
 
CANSAME5x CAN;
unsigned long startTime;

void rx2Tx(int CAN_ID_RX, int CAN_ID_TX) ;

#define X_ACCEL_OFFSET -.0036
#define Y_ACCEL_OFFSET .2191
#define Z_ACCEL_OFFSET -.04

#define X_GYRO_OFFSET -.0332
#define Y_GYRO_OFFSET -.0028
#define Z_GYRO_OFFSET -.0201

#define GS_PER_MPS2 0.101971

#define DPS_PER_BIT 0.1
#define GS_PER_BIT 0.01


Adafruit_MPU6050 mpu;
 
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();  // Join the I2C bus as a master
 
  Serial.println("CAN Sender");
 
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // accel/gyro settings
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
 
  // start the CAN bus at 1 Mbps
  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10);
  }
  Serial.println("Starting CAN!");

  startTime = millis();

}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // TODO: better programming practice for the repeated stuff below please
  float x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro;
  byte accel_bytes[8];
  byte gyro_bytes[8];

  // Rotation Rates
  // Roll, Pitch, and Yaw, with 2 bytes for each in the message
  // 0.1 dps per bit
  x_gyro = convertAndTranslate(g.gyro.x, X_GYRO_OFFSET, DPS_PER_BIT, &gyro_bytes[0],  &gyro_bytes[1]); 
  y_gyro = convertAndTranslate(g.gyro.y, Y_GYRO_OFFSET, DPS_PER_BIT, &gyro_bytes[2],  &gyro_bytes[3]); 
  z_gyro = convertAndTranslate(g.gyro.z, Z_GYRO_OFFSET, DPS_PER_BIT, &gyro_bytes[4],  &gyro_bytes[5]); 
  gyro_bytes[6] = '0';
  gyro_bytes[7] = '0';
  sendData(0x4E0, gyro_bytes);

  // Acceleration
  // Longitudinal, Latitudinal, and Vertical, with 2 bytes for each in the message
  // m/s^2 0.01 g per bit * 100 
  x_accel = convertAndTranslate(a.acceleration.x, X_ACCEL_OFFSET, GS_PER_MPS2 / GS_PER_BIT, &accel_bytes[0],  &accel_bytes[1]); 
  y_accel = convertAndTranslate(a.acceleration.y, Y_ACCEL_OFFSET, GS_PER_MPS2 / GS_PER_BIT, &accel_bytes[2],  &accel_bytes[3]); 
  z_accel = convertAndTranslate(a.acceleration.z, Z_ACCEL_OFFSET, GS_PER_MPS2 / GS_PER_BIT, &accel_bytes[4],  &accel_bytes[5]); 
  accel_bytes[6] = '0';
  accel_bytes[7] = '0';
  sendData(0x4E1, accel_bytes);

  delay(1);
}

void sendData(int canID, byte msg_arr[8])
{
  CAN.beginPacket(canID);

  for(int i = 0; i < 8; i++)
  {
    CAN.write(msg_arr[i]); // write one byte of message at a time
  }

  CAN.endPacket();
}

// inputs an integer CAN thing and its offset and multiplies by a conversion factor to get to the INT we want to send over can.
int convertAndTranslate(float sensorVal, float offset, const float conversionFactor, byte *highByte, byte *lowByte)
{
  int value = int(sensorVal - offset) * conversionFactor;
  *highByte = (value >> 8) & 0xFF; // Extract the high byte
  *lowByte = value & 0xFF;         // Extract the low byte
  return value; // return value for debugging purposes
}


void rx2Tx(int CAN_ID_RX, int CAN_ID_TX) {
 
  // Run checks for CAN message:
  //    - Check if a packet can be parsed
  //    - Check if CAN message is not RTR (Remote Request Frame)
  //    - Check if CAN is avaiable
  if (CAN.parsePacket() && !CAN.packetRtr() && CAN.available()) {  
   
    // Read only the specified CAN ID
    if (CAN.packetId() == CAN_ID_RX) {
     
      // begin a message on the write CAN ID
      CAN.beginPacket(CAN_ID_TX);
     
      // echo the message from read ID to write ID
      while (CAN.available()) {
        CAN.write(CAN.read());
      }
 
      // finish and send CAN write packet
      CAN.endPacket();
    }
  }

}

void printSettings()
{
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

}
 
// ------------------------ ARI CAPSTONE ---------------------
// for telemetry capstone stuff
void parseCAN(int minCANID, int maxCANID) {
 
  // Run checks for CAN message:
  //    - Check if a packet can be parsed
  //    - Check if CAN message is not RTR (Remote Request Frame)
  //    - Check if CAN is avaiable
  if (CAN.parsePacket() && !CAN.packetRtr() && CAN.available()) {  
    
    // Only parse the message if it's within a certain range of CAN IDs
    if (CAN.packetId() > minCANID && CAN.packetId() < maxCANID) {
      
      // get current time in milliseconds
      unsigned long timestamp = millis() - startTime;

      // Start building the serial output message with the timestamp & CAN ID
      String data;

      // Add all of the message to a string
      while (CAN.available()) {
        uint8_t dataByte = CAN.read();
        data += String(dataByte, HEX) + " "; // Append each byte in hex to serial message
      }

      //Serial.print(CAN.packetId()+" ");
      Serial.println(data);

      // Send String of CAN frame to ESP32
      featherToESP(CAN.packetId(), timestamp, data);
    }
  }
}

// for telemetry capstone stuff
void featherToESP(long id, unsigned long timestamp, String data)
{
  Wire.beginTransmission(8);  // Start transmission to I2C device with address 8
  //String message = " { \"Time\": " + String(timestamp) + ", \"Data\": \"" + data + "\" }";

  Wire.write(String(id, HEX).c_str()); 
  String message = " " + String(timestamp) + " " + data; // simplifying message for i2c purposes

  Wire.write(message.c_str());
  Wire.endTransmission();     // Complete the transmission

  // Print the message over serial
  Serial.println(message);
}