#include <mcp2515.h>

#include <CAN.h>

#define RX_GPIO_NUM 7
#define TX_GPIO_NUM 11

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // pinMode(TX_GPIO_NUM, INPUT);
  // pinMode(RX_GPIO_NUM, OUTPUT);

  Serial.println("Beginning CAN...");

  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);

  bool begin = CAN.begin(500);
  // if(begin == 0) {
  //   Serial.println("CAN failed");
  // } else {
  //   Serial.println("CAN ready!");
  // }
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
