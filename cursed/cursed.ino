void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("cursed beginning");

  long int max = 0;
  for(long int i = 0; i < (long int) 999999999; i++) {
    void *p = malloc(4);
    if(p == NULL) {
      break;
    }
    max += 4;
  }

  Serial.println("max reached:");
  Serial.print(max);
  Serial.println("bytes");

}

void loop() {
  // put your main code here, to run repeatedly:

}
