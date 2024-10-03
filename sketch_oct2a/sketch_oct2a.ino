#include <SPI.h>
#include <mcp2515.h>

#define FRAME_SIZE 8

struct can_frame canMsg;
struct MCP2515 mcp2515(5);  // CS pin is GPIO 5

// #define CAN_ACK_ID 0x037  // CAN ID for acknowledgment

void setup() {
  Serial.begin(115200);

  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setListenOnlyMode();
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    // Standard CAN ID is 11 bits
    uint32_t standard_can_id = canMsg.can_id & 0x7FF; // Mask for the lower 11 bits
    // the struct that is exposed from the mcp2515 library encodes the:
    // 1. can id
    // 2. (other flags)
    // and the CAN ID is encoded in the lower 11 bits.

    switch (standard_can_id) {
      case 0x4ED: { // accel
        // 1, 2 x angular
        // 3, 4 y angular
        // 5, 6 z angular
        // 7, 8 unused
        int x = (canMsg.data[0] << 8) | canMsg.data[1];
        int y = (canMsg.data[2] << 8) | canMsg.data[3];
        int z = (canMsg.data[4] << 8) | canMsg.data[5];

        Serial.print("ACCEL: ");
        Serial.print(x);
        Serial.print(" ");
        Serial.print(y);
        Serial.print(" ");
        Serial.print(z);
        Serial.println(" ");

        break;
      }

      case 0x100: { // battery

        int v = (canMsg.data[0] << 8) | canMsg.data[1];
        float scaled = (float) v * 0.01;

        Serial.print("BATTERY: ");
        Serial.print(scaled);
        Serial.println(" volts");
        delay(2 * 1000);
        break;
      }

      default:
        {
          Serial.print("Unknown ");
          Serial.print("CAN ID: ");
          Serial.print(standard_can_id, HEX);

          // Serial.print(" data: ");
          Serial.print(" data (");
          Serial.print(canMsg.can_dlc);
          Serial.print(" bytes): ");

          for (short i = 0; i < canMsg.can_dlc; i++) {
            Serial.print(canMsg.data[i]);
            Serial.print(" ");
          }
          Serial.println("");
          break;
        }
    }
  }
}