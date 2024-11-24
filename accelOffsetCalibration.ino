// /*
// 11-23-24 attempting to get acceleration offsets
// using same procedure & copying some code from gyroCalibration
// Ari
// */

// #define NUM_SAMPLES 500

// float min_x, max_x, mid_x;
// float min_y, max_y, mid_y;
// float min_z, max_z, mid_z;

// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

// Adafruit_MPU6050 mpu;

// void setup(void) {
//   Serial.begin(115200);
//   while (!Serial)
//     delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit MPU6050 test!");

//   // Try to initialize!
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("MPU6050 Found!");

//   mpu.setAccelerometerRange(MPU6050_RANGE_4_G); // because tulin said so
//   Serial.print("Accelerometer range set to: ");
//   switch (mpu.getAccelerometerRange()) {
//   case MPU6050_RANGE_2_G:
//     Serial.println("+-2G");
//     break;
//   case MPU6050_RANGE_4_G:
//     Serial.println("+-4G");
//     break;
//   case MPU6050_RANGE_8_G:
//     Serial.println("+-8G");
//     break;
//   case MPU6050_RANGE_16_G:
//     Serial.println("+-16G");
//     break;
//   }

//   // not doing filtering

//   Serial.println("");
//   delay(100);

//   /* Get new sensor events with the readings */
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   // initialize min and max
//   min_x = max_x = a.acceleration.x;
//   min_y = max_y = a.acceleration.y;
//   min_z = max_z = a.acceleration.z - 9.8067;
//   delay(10);

//   Serial.println("Place acccelerometer on a flat surface. Waiting 3...");
//   delay(1000);
//   Serial.println("2...");
//   delay(1000);
//   Serial.println("1...");
//   delay(1000);

//   Serial.println("Beginning calibration.");

//   float x, y, z;
//   for (int sample = 0; sample < NUM_SAMPLES; sample++)
//   {
//     mpu.getEvent(&a, &g, &temp);

//     x = a.acceleration.x;
//     y = a.acceleration.y;
//     z = a.acceleration.z - 9.8067;

//     Serial.print(F("Acceleration: ("));
//     Serial.print(x); Serial.print(", ");
//     Serial.print(y); Serial.print(", ");
//     Serial.print(z); Serial.print(")");

//     min_x = min(min_x, x);
//     min_y = min(min_y, y);
//     min_z = min(min_z, z);
  
//     max_x = max(max_x, x);
//     max_y = max(max_y, y);
//     max_z = max(max_z, z);
  
//     mid_x = (max_x + min_x) / 2;
//     mid_y = (max_y + min_y) / 2;
//     mid_z = (max_z + min_z) / 2;

//     Serial.print(F(" Zero rate offset: ("));
//     Serial.print(mid_x, 4); Serial.print(", ");
//     Serial.print(mid_y, 4); Serial.print(", ");
//     Serial.print(mid_z, 4); Serial.print(")");  
  
//     Serial.print(F("max - min noise (?): ("));
//     Serial.print(max_x - min_x, 3); Serial.print(", ");
//     Serial.print(max_y - min_y, 3); Serial.print(", ");
//     Serial.print(max_z - min_z, 3); Serial.println(")");   
//     delay(10);
//   }

//   Serial.println("");
//   delay(1);
// }

// void loop() {
//   delay(10); 

//   // idk
// }
