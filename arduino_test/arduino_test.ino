#define encoder1A 3
#define encoder1B 2
#define encoder1Out 11
#define ppr 100
#define baudrate 115200

int16_t countTick1 = 0;
int16_t countTick2 = 0;

void setup() {
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder1Out, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1A), computeRotation1, RISING);
  Serial.begin(baudrate);
}

void loop() {
//  Serial.write(countTick);
  char t;
  while(Serial.available()) Serial.read();
  while (1) {
    delay(500);
  }
}

void computeRotation1() {
  if (digitalRead(encoder1B)) {
    countTick1 += 1;
  } else {
    countTick1 -= 1;
  }
  writeToVex();
}

void writeToVex() {
  Serial.write((uint8_t *)&countTick1, 2);
  Serial.write((uint8_t *)&countTick2, 2);
}
