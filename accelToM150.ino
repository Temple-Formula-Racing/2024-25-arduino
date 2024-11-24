/*
 * october or november 2024
 * TFR CAN Message Echo Code
 * Adafruit feather m4 CAN 
 * also uses Izze racing 6dof accelerometer
 * receives accelerometer messages and transmits them to M150 over different CAN Ids 
 * because in the old days we couldn't set the ecu to receive messages over CAN IDs that weren't multiples of 16
 * 
 * ari reischer
 */
 
/*
 */
 
#include <CANSAME5x.h>
 
void rx2Tx(int CAN_ID_RX, int CAN_ID_TX);
 
CANSAME5x CAN;
 
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
 
  Serial.println("CAN Sender");
 
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster
 
  // start the CAN bus at 1 Mbps
  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10);
  }
  Serial.println("Starting CAN!");
}
 
void loop() {
 
  // Call function to echo Rotation Rates and Acceleration from the IMU's default CAN ID to one that MoTeC can read
 
  // Rotation Rates
  // Roll, Pitch, and Yaw, with 2 bytes for each in the message
  rx2Tx(0x4EC, 0x4E0);
 
  // Acceleration
  // Longitudinal, Latitudinal, and Vertical, with 2 bytes for each in the message
  rx2Tx(0x4ED, 0x4E1);
 
  delay(1);
 
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