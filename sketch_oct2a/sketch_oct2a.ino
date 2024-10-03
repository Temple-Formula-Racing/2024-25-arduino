#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
struct MCP2515 mcp2515(5);  // CS pin is GPIO 5

// #define CAN_ACK_ID 0x037  // CAN ID for acknowledgment

void setup() {
  Serial.begin(115200);

  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

String toHexString(int num) {
  char hexString[9];  // Enough space for 8 digits + null terminator
  sprintf(hexString, "%08X", num);  // Converts the number to hex format
  return String(hexString);  // Returns as a String object
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Serial.println("MESSAGE RECEIVED!");
    Serial.print("CAN ID: ");
    Serial.print(toHexString(canMsg.can_id));
    Serial.print(" data: ");
    Serial.println(canMsg.data[0]);
    // if (canMsg.can_id == 0x100)  // Check if the message is from the sender
    // {
    //   int battery = (canMsg.data[0] << 8) | canMsg.data[1];  // Combine MSB and LSB
    //   // //float temperatureC = tempInt / 100.0; // Convert back to float

    //   // Serial.print("Temperature received: ");
    //   // Serial.print(battery);
    //   // Serial.println(" °C");

    //   Serial.print("Can data:");
    //   Serial.println(canMsg.data[0]);

    //   // Send acknowledgment
    //   // canMsg.can_dlc = 0;           // No data needed for ACK
    //   // mcp2515.sendMessage(&canMsg);
    //   // Serial.println("ACK sent");
    // }
  }
}