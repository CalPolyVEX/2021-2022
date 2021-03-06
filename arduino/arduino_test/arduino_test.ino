#define encoder1A 2
#define encoder1B 12
#define encoder2A 3
#define encoder2B 13
#define ppr 100
#define baudrate 115200

int16_t countTick1 = 0;
int16_t countTick2 = 0;

void setup() {
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1A), computeRotation1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2A), computeRotation2, RISING);
  Serial.begin(baudrate);
  while(Serial.available()) Serial.read();
}

void loop() {
  delay(500);
}

void computeRotation1() {
  if (digitalRead(encoder1B)) {
    countTick1 += 1;
  } else {
    countTick1 -= 1;
  }
  writeToVex();
}

void computeRotation2() {
  if (digitalRead(encoder2B)) {
    countTick2 += 1;
  } else {
    countTick2 -= 1;
  }
  writeToVex();
}

void writeToVex() {
  Serial.write((uint8_t *)&countTick1, 2);
  Serial.write((uint8_t *)&countTick2, 2);
}
